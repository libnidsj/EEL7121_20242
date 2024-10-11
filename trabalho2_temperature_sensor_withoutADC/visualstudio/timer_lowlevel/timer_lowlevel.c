#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/timer.h"
#include "hardware/gpio.h"
#include "pico/cyw43_arch.h"

#define CHARGE_PIN 14
#define SAMPLE_PIN 15
#define R1 32.8e3
#define R2 68e3
#define C 155.6e-9
#define IOVDD 3.3
#define VIL 1.2
#define VIH 1.6
#define RX_TH 100
#define SAMPLE_IRQ_MASK (GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE)

static uint64_t dth_start_time;
static uint64_t dth_end_time;

void dth_init(void);                            // Initialize Direct-To-Hardware GPIO
void dth_eoc_callback(uint input_pin, uint32_t irq_thr);           // DTH End of Conversion Callback
float dth_conv_r();                               // DTH Conversion
float dth_conv_v();                               // DTH Conversion
float dth_t_to_r(uint64_t delta_t);  // DTH Time to Resistance Calculation
float dth_t_to_v(uint64_t delta_t, bool mode);  // DTH Time to Resistance Calculation
float dth_mode_1_tf(float RX, float t);
float dth_mode_0_tf(float RX, float t);
float dth_secant_method(float t);

int main(){
    stdio_init_all();

    busy_wait_ms(5000);
        
    if (cyw43_arch_init()) {
        printf("Wi-Fi init failed\n");
        return -1;
    }

    printf("Wi-Fi init sucess\n");

    // In order to enabe LED
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);

    dth_init();
    
    while(1){
        busy_wait_ms(1000);
        printf("conversion result: %f \n", dth_conv_r());
        dth_start_time, dth_end_time = 0;
    }
}

float dth_conv_v(){
    printf("Starting conversion");

    gpio_set_dir(CHARGE_PIN, false);
    busy_wait_us((uint64_t) 5*R2*C);
    dth_end_time = 0;

    bool mode = gpio_get(SAMPLE_PIN);
    gpio_put(CHARGE_PIN, !mode);

    gpio_set_irq_enabled(SAMPLE_PIN, SAMPLE_IRQ_MASK, true);
    dth_start_time = time_us_64();
    gpio_set_dir(CHARGE_PIN, true);

    while(dth_end_time == 0);

    printf("Received time different from zero");

    gpio_set_dir(CHARGE_PIN, false);
    return dth_mode_1_tf(dth_end_time - dth_start_time, mode);
}

float dth_conv_r(){
    printf("Starting conversion");

    gpio_set_dir(CHARGE_PIN, false);
    busy_wait_us((uint64_t) 5*R2*C);
    dth_end_time = 0;

    bool mode = gpio_get(SAMPLE_PIN);
    gpio_put(CHARGE_PIN, !mode);

    gpio_set_irq_enabled(SAMPLE_PIN, SAMPLE_IRQ_MASK, true);
    dth_start_time = time_us_64();
    gpio_set_dir(CHARGE_PIN, true);

    while(dth_end_time == 0){
        busy_wait_ms(100);
    }

    // CODE STUCK HERE

    printf("Received time different from zero\n");

    gpio_set_dir(CHARGE_PIN, false);
    printf("%llu\n", (dth_end_time - dth_start_time));
    return dth_t_to_r(dth_end_time - dth_start_time);
}

float dth_mode_1_tf(float rx, float t){
    return (C/(1/R1 + 1/rx))*log(((VIL*(R1+rx))/IOVDD - R1)/rx) - t;
}

float dth_secant_method(float t){
    float rx_1_tmp, res, rx, rx_2 = 30e3, rx_1 = 80e3;
    for(int i = 0; i < 10; i++){
        rx_1_tmp = rx;
        rx = (rx_2*dth_mode_1_tf(rx_1, t) - rx_1*dth_mode_1_tf(rx_2, t))/(dth_mode_1_tf(rx_1, t) - dth_mode_1_tf(rx_2, t));
        res = rx - rx_1_tmp;
        if(fabs(res) < RX_TH){
            break;
        }
        rx_2 = rx_1;
        rx_1 = rx_1_tmp;
    }
    return rx;
}

float dth_t_to_r(uint64_t delta_t){
    return dth_secant_method(delta_t*1e-6);
}

float dth_t_to_v(uint64_t delta_t, bool mode){
    if(mode){

    }else{

    }
}

void dth_eoc_callback(uint input_pin, uint32_t irq_thr){
    printf("Callback enter\n");
    //printf("%x\n", gpio_get_irq_event_mask(SAMPLE_PIN));
    //printf("%x\n", SAMPLE_IRQ_MASK);
    // if (gpio_get_irq_event_mask(SAMPLE_PIN) & SAMPLE_IRQ_MASK) {
        printf("Entered callback if\n");
        gpio_acknowledge_irq(SAMPLE_PIN, SAMPLE_IRQ_MASK);
        dth_end_time = time_us_64();
        printf("end_time (inside callback) = %llu \n", dth_end_time);
        gpio_set_irq_enabled(SAMPLE_PIN, SAMPLE_IRQ_MASK, false);
    //}
    printf("Callback end\n");
}

void dth_init(){
    printf("Configurating pins and interruption\n");

    gpio_init(CHARGE_PIN);
    gpio_init(SAMPLE_PIN);
    gpio_set_dir(CHARGE_PIN, false);
    gpio_set_dir(SAMPLE_PIN, false);
    // gpio_add_raw_irq_handler(SAMPLE_PIN, &dth_eoc_callback);
    gpio_set_irq_enabled_with_callback(SAMPLE_PIN, SAMPLE_IRQ_MASK, false, &dth_eoc_callback);
    printf("Finished pin configuration stage\n");
}
