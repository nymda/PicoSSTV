#include <stdio.h>
#include <math.h>
#include <malloc.h>
#include "pico/stdlib.h"
#include "audio.hpp"
#include "camera.h"
//#include "pico/multicore.h"
#include <cstring>

#include "bsp/board_api.h"
#include "tusb.h"

#define BYTE unsigned char
#define LED_PIN 25
#define PICO_AUDIO_PACK_I2S_DATA 9
#define PICO_AUDIO_PACK_I2S_BCLK 10
#define SAMPLES_PER_BUFFER 256
#define BUTTON_PIN 20
#define BUTTON_LED_PIN 21
#define BUFFER_SIZE 256

int ledBlinkTime = 250;

struct serialCommand {
    char header[2] = { 'S', 'C' };
    short ID = 0;
    char payload[4] = {};
};

enum serialCommandIds{
    invalid = -1,
    idle = 0,
    requesting = 1,
    img_over_audio = 2,
    img_over_serial = 3
};

uint32_t getTotalHeap(void) {
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

void HALT(){
    while(true){
        gpio_put(BUTTON_LED_PIN, true);
        sleep_ms(ledBlinkTime);
        gpio_put(BUTTON_LED_PIN, false);
        sleep_ms(ledBlinkTime);
    }
}

int clampUC(int input) {
    return (input) > 255 ? 255 : (input) < 0 ? 0 : input;
}

double expectedDurationMS = 0;
double actualDurationMS = 0;
int balance_AddedSamples = 0;
int balance_SkippedSamples = 0;
int bytesWritten = 0;
int writeIndex = 0;
int blocks = 0;

void printDbg(){
    printf("Encode complete!\n");
    printf("Expected MS: %f\n", expectedDurationMS);
    printf("Actual MS  : %f\n", actualDurationMS);
    printf(" Added: %i, Skipped: %i\n\n", balance_AddedSamples, balance_SkippedSamples);
}

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

bool led = false;

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
            gpio_put(BUTTON_LED_PIN, led);
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

void encodeVOX(){
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

//unsafe! very!
void serialTransmit(uint8_t* data, int length){
    for(int i = 0; i < length; i++){
        putchar_raw(data[i]);
    }
}

//unsafe! very!
void serialRecieve(uint8_t* buffer, int length, int timeout){
    for(int i = 0; i < length; i++){
        buffer[i] = (uint8_t)getchar_timeout_us(timeout);
    }
}

void serialTransmitChars(const char* data){
    serialTransmit((uint8_t*)data, strlen(data));
}

int serialRecieveCommand(){

    char ping[8] = {};
    serialRecieve((uint8_t*)ping, 8, 25000);
    serialTransmit((uint8_t*)ping, 8);

    return 0;

    serialCommand recieve = {};
    recieve.ID = serialCommandIds::idle;

    serialCommand request = {};
    request.ID = serialCommandIds::requesting;

    //request instruction from connected device
    serialTransmit((uint8_t*)&request, sizeof(serialCommand));

    //recieve instruction if provided
    serialRecieve((uint8_t*)&recieve, 8, 25000);

    serialTransmit((uint8_t*)&recieve, sizeof(serialCommand));

    if(recieve.header[0] != 'S' || recieve.header[1] != 'C'){ recieve = {}; recieve.ID == serialCommandIds::idle; }

    //return instruction ID
    return recieve.ID;
}

static void cdc_task(void);

int main() {
    stdio_init_all();
    board_init();
    tud_init(BOARD_TUD_RHPORT);
    if (board_init_after_tusb) {
        board_init_after_tusb();
    }
    while (1) {
        tud_task(); // tinyusb device task
        cdc_task();
    }

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    gpio_init(BUTTON_PIN);
    gpio_set_dir(BUTTON_PIN, GPIO_IN);
    gpio_pull_up(BUTTON_PIN);

    gpio_init(BUTTON_LED_PIN);
    gpio_set_dir(BUTTON_LED_PIN, GPIO_OUT);

    if(!initCamera(160, 120, 1500)){
        HALT();
    }

    ap = init_audio(sampleRate, PICO_AUDIO_PACK_I2S_DATA, PICO_AUDIO_PACK_I2S_BCLK);

    led = true;
    gpio_put(BUTTON_LED_PIN, led);

    while(true){
        //wait for the button to be pressed again
        serialCommand next = {};
        next.ID == serialCommandIds::idle;
        bool btnPressed = !gpio_get(BUTTON_PIN);  
        while(next.ID == serialCommandIds::idle){
            btnPressed = !gpio_get(BUTTON_PIN);
            next.ID = serialRecieveCommand();
            if(btnPressed){ next.ID = serialCommandIds::img_over_audio; }
        }

        // Read image data into frameBuffer over SPI
        captureFrame();

        // Send frame to PC over serial
        if(next.ID == serialCommandIds::img_over_serial){
            led = false;
            gpio_put(BUTTON_LED_PIN, led);
            serialTransmit(framebuffer, (160 * 120));
            led = true;
            gpio_put(BUTTON_LED_PIN, led);
        }

        // Encode frame and transmit as SSTV
        if(next.ID == serialCommandIds::img_over_audio){
            //VOX tone
            encodeVOX();

            //SSTV encode
            encodeBW8();

            //footer, also makes the blinking LED look nice
            tone(0, (int)actualDurationMS % ledBlinkTime);
        }

        // Reset variables
        reset();

        // Turn the LED back on
        led = true;
        gpio_put(BUTTON_LED_PIN, led);
    }

    HALT();  

    return 0;
}

static void cdc_task(void) {
  uint8_t itf;

  for (itf = 0; itf < CFG_TUD_CDC; itf++) {
    // connected() check for DTR bit
    // Most but not all terminal client set this when making connection
    // if ( tud_cdc_n_connected(itf) )
    {
      if (tud_cdc_n_available(itf)) {
        uint8_t buf[64];

        uint32_t count = tud_cdc_n_read(itf, buf, sizeof(buf));

        // echo back to both serial ports
        echo_serial_port(0, buf, count);
        echo_serial_port(1, buf, count);
      }
    }
  }
}