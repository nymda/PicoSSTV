#include <stdio.h>
#include <math.h>
#include <malloc.h>
#include <cstring>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"
#include "audio.hpp"
#include "camera.h"
#include "fontRenderer.h"
#include "pico/multicore.h"

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

enum commandID{
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

// void HALT(){
//     while(true){
//         gpio_put(BUTTON_LED_PIN, true);
//         sleep_ms(ledBlinkTime);
//         gpio_put(BUTTON_LED_PIN, false);
//         sleep_ms(ledBlinkTime);
//     }
// }

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

const char* CALLSIGN = "M7XYZ";
bool enableCallsign = false;

int blocksPerLED = (int)((float)LedBlinkMS / (((float)BUFFER_SIZE / (float)sampleRate) * 1000.f));

int16_t audioData[BUFFER_SIZE] = {};

struct audio_buffer_pool* ap = 0; 

int si = 0;
int16_t s = 0;

bool motion = false;
bool led = false;
uint16_t led_base = 0x0000;
uint16_t led_max = 0xFFFF;

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

int main() {
    stdio_init_all();

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

    led = true;
    setLED(led);

    int held = 0;

    while(true){

        //while button is released - initial hold point
        while(gpio_get(BUTTON_PIN)){
            sleep_ms(25);
            if(getMotion()){
                resetMotion();
                if(motion){ break; }
            }
        }

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
                motion = !motion;
                if(motion){ led_max = (0xFFFF / 10);}
                else{ led_max = (0xFFFF); }
                setLED(led);
                held = -10;
            }
        }
        if(held < 0){ held = 0; continue; }
        held = 0;
        
        // Read image data into frameBuffer over SPI
        captureFrame();

        if(enableCallsign){
            drawStr(framebuffer, 0, 0, CALLSIGN);
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