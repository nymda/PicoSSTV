#include "pico/multicore.h"
#define LED_PIN 25

enum class state{
    IDLE_ON = 0,
    IDLE_OFF = 1,
    BLINK = 2
};

state currentState = state::IDLE_ON;

long counter = 0;
bool blinker = false;

void core1_interrupt(){
    while (multicore_fifo_rvalid()){
        uint16_t raw = multicore_fifo_pop_blocking();
        printf("Interrupt fired\n");
    }
    multicore_fifo_clear_irq();
}

void core1_main(){
    multicore_fifo_clear_irq();
    irq_set_exclusive_handler(SIO_IRQ_PROC1, core1_interrupt);
    irq_set_enabled(SIO_IRQ_PROC1, true);

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    printf("CORE1 started\n");

    while(true){
        
    }
}
