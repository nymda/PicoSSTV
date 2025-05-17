#include <stdio.h>
#include <math.h>
#include <malloc.h>
#include <cstring>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"
#include "hardware/flash.h"
#include "audio.hpp"
#include "camera.h"
#include "fontRenderer.h"
#include "pico/multicore.h"
#include "tusb.h"

/*

    This originally started as a pretty simple project, but has grown more complex over time.
    This file is really not organised or split up as it should be.
    Very few good programming practices have been followed.

*/

#define BYTE unsigned char
#define LED_PIN 25
#define PICO_AUDIO_PACK_I2S_DATA 9
#define PICO_AUDIO_PACK_I2S_BCLK 10
#define SAMPLES_PER_BUFFER 256
#define BUTTON_PIN 20
#define BUTTON_LED_PIN 21
#define BUFFER_SIZE 256
#define FLASH_TARGET_OFFSET (PICO_FLASH_SIZE_BYTES - 4096)
#define CALLSIGN_LENGTH 12

int ledBlinkTime = 250;

enum serialPacketIds{
    idle = 0,
    frameSection = 1,
    captureStarted = 2,
    cameraInfo = 3,
    command = 4
};

enum serialCommandIds{
    reserved = 0,
    trigger = 1,
    setMotion = 2,
};

//CAM => PC
struct frameSectionPacket{
    char header[2] = { 'S', 'P' };
    short ID = 1;
    int offset;
    int dataSize;
    uint8_t data[];
};

//CAM => PC
struct captureStartedPacket{
    char header[2] = { 'S', 'P' };
    short ID = 2;
};

//CAM => PC
struct cameraInfoPacket{
    char header[2] = { 'S', 'P' };
    short ID = 3;
    bool motionDetecting;
    bool motionDetected;
    short gain;
};

//CAM <= PC
struct commandPacket{
    char header[2] = {'S', 'P'};
    short ID = 4;
    short command = 0;
    uint8_t data[16] = { 0 };
};


uint32_t getTotalHeap() {
   extern char __StackLimit, __bss_end__;
   return &__StackLimit  - &__bss_end__;
}

void printMemInfo(){
    struct mallinfo m = mallinfo();
    int heapTotal = getTotalHeap();
    int heapUsed = getTotalHeap() - m.uordblks;
    float heapUsedPercent = 100.f - ((float)heapUsed / (float)heapTotal) * 100.f;

    printf("Heap total: %i\n", heapTotal);
    printf("Heap free : %i\n", heapUsed);
    printf("Used %%    : %.2f\n", heapUsedPercent);
}

int clampUC(int input) {
    return (input) > 255 ? 255 : (input) < 0 ? 0 : input;
}

void setLED(bool enable);

double expectedDurationMS = 0;
double actualDurationMS = 0;
int balance_AddedSamples = 0;
int balance_SkippedSamples = 0;
int bytesWritten = 0;
int writeIndex = 0;
int blocks = 0;
const int LedBlinkMS = 250;
const int ampl = 25000;
const int sampleRate = 8000;
const int frequency = 1000;
const float pi = 3.1415926535;
int dataSize = 0;
double angle = 0.0;
int blocksPerLED = (int)((float)LedBlinkMS / (((float)BUFFER_SIZE / (float)sampleRate) * 1000.f));
int16_t audioData[BUFFER_SIZE] = {};
struct audio_buffer_pool* ap = 0; 
int si = 0;
int16_t s = 0;
bool motionDetection = false;
bool externalTrigger = false;
bool led = false;
uint16_t led_base = 0x0000;
uint16_t led_max = 0xFFFF;
char CALLSIGN[CALLSIGN_LENGTH] = { };
const uint8_t *flash_target_contents = (const uint8_t *) (XIP_BASE + FLASH_TARGET_OFFSET);

bool validateFlashCallsign(){
    for(int i = 0; i < CALLSIGN_LENGTH; i++){
        char c = flash_target_contents[i];
        if(!isValidCharacter(c)){ return false; }
    }
    return true;
}

void __not_in_flash_func(callsign_write_flash)(const uint8_t* buffer){
    flash_range_program(FLASH_TARGET_OFFSET, buffer, CALLSIGN_LENGTH);
}

void callsign_read_flash(){
    if(!validateFlashCallsign()){
        flash_range_erase(FLASH_TARGET_OFFSET, FLASH_SECTOR_SIZE);
    }
    memcpy(CALLSIGN, flash_target_contents, CALLSIGN_LENGTH);
}

void printDbg(){
    printf("Encode complete!\n");
    printf("Expected MS: %f\n", expectedDurationMS);
    printf("Actual MS  : %f\n", actualDurationMS);
    printf(" Added: %i, Skipped: %i\n\n", balance_AddedSamples, balance_SkippedSamples);
}

void setMotionDetection(bool enabled){
    motionDetection = enabled;
    if(motionDetection){ led_max = (0xFFFF / 10);}
    else{ led_max = (0xFFFF); }
    setLED(led);
}

void second_core_read_USB_stream(){
    while(true){
        if(stdio_usb_connected()){
            char RXbuffer[sizeof(commandPacket)] = { 0 };
            tud_cdc_read(RXbuffer, sizeof(RXbuffer));
            if(RXbuffer[0] == 'S' && RXbuffer[1] == 'P'){
                commandPacket* cmd = (commandPacket*)RXbuffer;
                if(cmd->ID != serialPacketIds::command ){ continue; }

                if(cmd->command == serialCommandIds::trigger){
                    externalTrigger = true;
                }
                if(cmd->command == serialCommandIds::setMotion){
                    setMotionDetection(cmd->data[0]);
                }
            }
        }
    }
}

static void usb_write_data(uint8_t* buf, int length) {
    static uint64_t last_avail_time;
    if (stdio_usb_connected()) {
        for (int i = 0; i < length;) {
            int n = length - i;
            int avail = (int)tud_cdc_write_available();
            if (n > avail) n = avail;
            if (n) {
                int n2 = (int) tud_cdc_write(buf + i, (uint32_t)n);
                tud_task();
                tud_cdc_write_flush();
                i += n2;
                last_avail_time = time_us_64();
            } else {
                tud_task();
                tud_cdc_write_flush();
                if (!stdio_usb_connected() ||
                    (!tud_cdc_write_available() && time_us_64() > last_avail_time + PICO_STDIO_USB_STDOUT_TIMEOUT_US)) {
                    break;
                }
            }
        }
    } else {
        // reset our timeout
        last_avail_time = 0;
    }
}

static void usb_transfer_frame(int chunks){
    short totalTransferLength = (160 * 120);
    short remainingTransferLength = totalTransferLength;
    short chunkSize = totalTransferLength / chunks;

    for(int i = 0; i < chunks; i++){
        frameSectionPacket* FSP = (frameSectionPacket*)malloc(sizeof(frameSectionPacket) + chunkSize);
        short frameOffset = totalTransferLength - remainingTransferLength;
        memcpy(FSP->data, framebuffer + frameOffset, chunkSize);
        //memset(FSP->data, 0, chunkSize);
        FSP->header[0] = 'S';
        FSP->header[1] = 'P';
        FSP->ID = serialPacketIds::frameSection;
        FSP->dataSize = chunkSize;
        FSP->offset = frameOffset;
        usb_write_data((uint8_t*)FSP, sizeof(frameSectionPacket) + FSP->dataSize);
        remainingTransferLength -= chunkSize;
        free(FSP);
        sleep_us(500);
    }
}

void setLED(bool enable){
    if(enable){
        pwm_set_gpio_level(BUTTON_LED_PIN, led_max);
    }
    else{
        pwm_set_gpio_level(BUTTON_LED_PIN, led_base);
    }
}

int reset() {
    expectedDurationMS = 0;
    actualDurationMS = 0;
    balance_AddedSamples = 0;
    balance_SkippedSamples = 0;
    bytesWritten = 0;
    writeIndex = 0;
    blocks = 0;
    return 0;
}

void tone(short frequency, float duration) {

    //number of samples required for the requested duration. sometimes.
    int sampleCount = round((sampleRate) * (duration / 1000.f));

    //balancing
    expectedDurationMS += duration;
    actualDurationMS += (sampleCount / static_cast<double>(sampleRate)) * 1000;
    float msPerSample = 1000.f / sampleRate;

    //actually calculate and add the data
    for (int i = 0; i < sampleCount; i++) {

        if(writeIndex == SAMPLES_PER_BUFFER){
            update_buffer(ap, audioData);
            writeIndex = 0;
            blocks++;
        }

        if(blocks == blocksPerLED){
            blocks = 0;
            led = !led;
            setLED(led);
        }

        audioData[writeIndex] = (int16_t)(ampl * sin(angle));    
        angle += ((2 * pi * frequency) / sampleRate);
        writeIndex++;

        //resolves issues with timing due to too many or too few samples being generated
        float diff = actualDurationMS - expectedDurationMS;
        if (diff > msPerSample) {
            sampleCount--;
            actualDurationMS -= msPerSample;
            balance_SkippedSamples++;
        }
        if (diff < -msPerSample) {
            sampleCount++;
            actualDurationMS += msPerSample;
            balance_AddedSamples++;
        }

        while (angle > 2 * pi) { angle -= 2 * pi; } //avoid floating point weirdness
    }
}

void encodeVOX(){ //long vox tone is not standard but is required for some radios
    tone(1900, 100);
    tone(1500, 100);
    tone(1900, 100);
    tone(1500, 100);
    tone(2300, 100);
    tone(1500, 100);
    tone(2300, 100);
    tone(1500, 100);
    tone(1900, 100);
    tone(1500, 100);
    tone(1900, 100);
    tone(1500, 100);
    tone(2300, 100);
    tone(1500, 100);
    tone(2300, 100);
    tone(1500, 100);
}

void encodeVIS(BYTE visCode){
    tone(1900, 300);
    tone(1200, 10);
    tone(1900, 300);
    tone(1200, 30);
    int bit = 0;
    for (int i = 0; i < 8; i++) {
        bit = (visCode >> i) & 1;
        if (bit) {
            tone(1100, 30); //1
        }
        else {
            tone(1300, 30); //0
        }
    }
    tone(1200, 30);
}

void encodeBW8(){

    encodeVIS(0x82);

    short WIDTH = 160;
    short HEIGHT = 120;
    float mspp = 58.89709f / static_cast<float>(WIDTH);
    for (int y = 0; y < HEIGHT; y++) {
        tone(1200, 6.f);
        tone(1500, 2.f);
        for (int x = 0; x < WIDTH; x++) {
            BYTE px = framebuffer[(y * WIDTH) + x];
            tone((1500.0f + (3.125f * (float)px)), mspp);
        }
    }
}

int hold(){
    if(getMotion()){
        resetMotion();
        if(motionDetection){ return 1; }
    }
    if(externalTrigger){
        externalTrigger = false;
        return 2;
    }
    if(stdio_usb_connected()){
        captureFrame();
        if(CALLSIGN[0] != 0x00){
            drawStr(framebuffer, 0, 0, CALLSIGN);
        }
        usb_transfer_frame(8);
    }
    return 0;
}

int main() {
    stdio_init_all();
    //callsign_read_flash();

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    gpio_init(BUTTON_PIN);
    gpio_set_dir(BUTTON_PIN, GPIO_IN);
    gpio_pull_up(BUTTON_PIN);

    gpio_init(BUTTON_LED_PIN);
	gpio_set_function(BUTTON_LED_PIN, GPIO_FUNC_PWM);
	pwm_set_gpio_level(BUTTON_LED_PIN, 0);
	uint slice_num = pwm_gpio_to_slice_num(BUTTON_LED_PIN);
	pwm_set_enabled(slice_num, true);

    if(!initCamera(160, 120)){ return 0; }
    ap = init_audio(sampleRate, PICO_AUDIO_PACK_I2S_DATA, PICO_AUDIO_PACK_I2S_BCLK);

    multicore_launch_core1(second_core_read_USB_stream);

    led = true;
    setLED(led);

    int held = 0;

    while(true){

        //while button is released - initial hold point
        while(gpio_get(BUTTON_PIN)){
            sleep_ms(25);
            if(hold() > 0){
                break;
            }
        }

        // Read image data into frameBuffer over SPI
        captureFrame();
        //do this before entering the while pressed loop so that the final image is captured on press rather than on release

        //while button is pressed - only used to toggle motion - completely incomprehensable
        while(!gpio_get(BUTTON_PIN)){       
            if(held < 1250 && held >= 0){
                sleep_ms(5);
                held+=5;
            }
            else if(held == -10){
                sleep_ms(5);
            }
            else{
                setMotionDetection(!motionDetection);
                held = -10;
            }
        }
        if(held < 0){ held = 0; continue; }
        held = 0;

        if(CALLSIGN[0] != 0x00){
            drawStr(framebuffer, 0, 0, CALLSIGN);
        }

        if(stdio_usb_connected()){ 
            usb_transfer_frame(8);
        }

        //VOX tone
        encodeVOX();

        //SSTV encode
        encodeBW8();

        //footer, also makes the blinking LED look nice
        tone(0, (int)actualDurationMS % ledBlinkTime);

        // Reset variables
        reset();

        //reset the motion detected flag
        resetMotion();

        // Turn the LED back on
        led = true;
        setLED(led);
    }

    return 0;
}