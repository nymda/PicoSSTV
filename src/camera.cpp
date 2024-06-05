#include "pico/stdlib.h"
// #include "bsp/board.h"
// #include "tusb.h"
// #include "usb_descriptors.h"
#include <stdio.h>
#include <malloc.h>
extern "C"{
#include "hm01b0.h"
}

#define EXPOSURE_MAX 1500
#define EXPOSURE_MIN 150

struct hm01b0_config hm01b0_config = {
    .i2c           = PICO_DEFAULT_I2C_INSTANCE,
    .sda_pin       = PICO_DEFAULT_I2C_SDA_PIN,
    .scl_pin       = PICO_DEFAULT_I2C_SCL_PIN,
    .vsync_pin     = 16,
    .hsync_pin     = 15,
    .pclk_pin      = 14,
    .data_pin_base = 6,
    .data_bits     = 1,
    .pio           = pio0,
    .pio_sm        = 1,
    .reset_pin     = -1,   // Not connected
    .mclk_pin      = -1,   // Not connected
    .width         = 160,
    .height        = 120,
};

uint8_t* framebuffer = 0;

bool initCamera(int width, int height, int exposure){
    if(framebuffer){
        free(framebuffer);
    }

    hm01b0_config.width = width;
    hm01b0_config.height = height;

    if (hm01b0_init(&hm01b0_config) != 0) { return false; }
    framebuffer = (uint8_t*)malloc(hm01b0_config.width * hm01b0_config.height);
    
    hm01b0_set_max_DGain(0xF0);
    hm01b0_set_max_AGain(0x02);
    hm01b0_set_brightness(95);
    hm01b0_enable_auto_exposure(true);

    return true;
}

void captureFrame(){
    hm01b0_read_frame(framebuffer, (hm01b0_config.width * hm01b0_config.height));
    for(int i = 0; i < (hm01b0_config.width); i++){
        printf("%02x", framebuffer[i]);
    }
    printf("\n");
}