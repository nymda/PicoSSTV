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

// Invoked when device is mounted
void tud_mount_cb(void)
{
}

// Invoked when device is unmounted
void tud_umount_cb(void)
{
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en)
{
  (void) remote_wakeup_en;
}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{
}

void video_task();

// static unsigned frame_num = 0;
// static unsigned tx_busy = 0;
// static unsigned interval_ms = 1000 / FRAME_RATE;

uint8_t* framebuffer = 0;
// uint8_t* wframebuffer = 0;

bool locked = false;
bool capturing = false;
int exposure = 1000;

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

bool initCamera(int width, int height, int exposure){
    if(framebuffer /*|| wframebuffer*/){
        free(framebuffer);
        // free(wframebuffer);
    }

    hm01b0_config.width = width;
    hm01b0_config.height = height;

    if (hm01b0_init(&hm01b0_config) != 0) { return false; }

    framebuffer = (uint8_t*)malloc(hm01b0_config.width * hm01b0_config.height);
    //wframebuffer = (uint8_t*)malloc((hm01b0_config.width * hm01b0_config.height) * 2);

    hm01b0_set_exposure(exposure);
    //tud_init(BOARD_TUD_RHPORT);

    return true;
}

void captureFrame(){
    hm01b0_read_frame(framebuffer, (hm01b0_config.width * hm01b0_config.height));
}

// int calcAverageBrightness(int step){
//     int total = 0;
//     for(int i = 0; i < (hm01b0_config.width * hm01b0_config.height); i+=step){
//         total += framebuffer[i];
//     }
//     return total / (hm01b0_config.width * hm01b0_config.height);
// }

int _lastExposure = 0;
void cameraSecondaryThreadMain(){
    initCamera(160, 120, 100);
    while(true){
        //tud_task();
        //video_task();

        if(!locked){ 
            capturing = true;
            captureFrame();
            capturing = false;
        }

        sleep_ms(1);
    }
}

bool isCapturing(){
    return capturing;
}

// void video_task(void){
//     static unsigned start_ms = 0;
//     static unsigned already_sent = 0;

//     if (!tud_video_n_streaming(0, 0)) {
//         already_sent  = 0;
//         frame_num     = 0;
//         return;
//     }

//     if (!already_sent) {
//         already_sent = 1;
//         start_ms = board_millis();
//         tx_busy = 1;

//         tud_video_n_frame_xfer(0, 0, (void*)wframebuffer, ((hm01b0_config.width * hm01b0_config.height) * 2));
//     }

//     unsigned cur = board_millis();
//     if (cur - start_ms < interval_ms) return; // not enough time
//     if (tx_busy) return;
//     tx_busy = 1;
//     start_ms += interval_ms;

//     tud_video_n_frame_xfer(0, 0, (void*)wframebuffer, ((hm01b0_config.width * hm01b0_config.height) * 2));
// }

// void tud_video_frame_xfer_complete_cb(uint_fast8_t ctl_idx, uint_fast8_t stm_idx){
//     (void)ctl_idx; (void)stm_idx;
//     tx_busy = 0;
//     /* flip buffer */
//     ++frame_num;
// }

// int tud_video_commit_cb(uint_fast8_t ctl_idx, uint_fast8_t stm_idx, video_probe_and_commit_control_t const *parameters){
//     (void)ctl_idx; (void)stm_idx;
//     /* convert unit to ms from 100 ns */
//     interval_ms = parameters->dwFrameInterval / 10000;
//     return VIDEO_ERROR_NONE;
// }



