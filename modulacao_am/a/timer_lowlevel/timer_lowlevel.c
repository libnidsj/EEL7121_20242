/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/timer.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"

#define LUT_SIZE 100
#define LUT_LOW_DIV 100

// Note on RP2350 timer_hw is the default timer instance (as per PICO_DEFAULT_TIMER)

/// \tag::get_time[]
// Simplest form of getting 64 bit time from the timer.
// It isn't safe when called from 2 cores because of the latching
// so isn't implemented this way in the sdk
static uint64_t get_time(void) {
    // Reading low latches the high value
    uint32_t lo = timer_hw->timelr;
    uint32_t hi = timer_hw->timehr;
    return ((uint64_t) hi << 32u) | lo;
}
/// \end::get_time[]

/// \tag::alarm_standalone[]
// Use alarm 0
#define ALARM_NUM 0
#define ALARM_IRQ hardware_alarm_get_irq_num(timer_hw, ALARM_NUM)

const float conversion_factor = 3.3f / (1 << 12);
uint16_t adc_result = 0;

uint32_t slice_num_15 = 0;
uint32_t slice_num_16 = 0;

uint32_t LUT[] = {
        255, 254, 253, 252, 250, 248, 246, 242, 239, 235, 230, 225, 220,
        214, 208, 202, 195, 188, 181, 174, 166, 159, 151, 143, 135, 127,
        119, 111, 103,  95,  88,  80,  73,  66,  59,  52,  46,  40,  34,
        29,  24,  19,  15,  12,   8,   6,   4,   2,   1,   0,   0,   0,
        1,   2,   4,   6,   8,  12,  15,  19,  24,  29,  34,  40,  46,
        52,  59,  66,  73,  80,  88,  95, 103, 111, 119, 127, 135, 143,
        151, 159, 166, 174, 181, 188, 195, 202, 208, 214, 220, 225, 230,
        235, 239, 242, 246, 248, 250, 252, 253, 254
       };

uint32_t LUTsquare[] = {
        0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
        0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
        0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
        0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 255, 255,
        255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
        255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
        255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
        255, 255, 255, 255, 255, 255, 255, 255, 255
        };

uint32_t LUTtriangle[] = {
        0,   5,  10,  15,  20,  25,  30,  35,  40,  45,  51,  56,  61,
        66,  71,  76,  81,  86,  91,  96, 102, 107, 112, 117, 122, 127,
        132, 137, 142, 147, 153, 158, 163, 168, 173, 178, 183, 188, 193,
        198, 204, 209, 214, 219, 224, 229, 234, 239, 244, 249, 255, 249,
        244, 239, 234, 229, 224, 219, 214, 209, 204, 198, 193, 188, 183,
        178, 173, 168, 163, 158, 153, 147, 142, 137, 132, 127, 122, 117,
        112, 107, 102,  96,  91,  86,  81,  76,  71,  66,  61,  56,  51,
        45,  40,  35,  30,  25,  20,  15,  10,   5
        };

uint32_t *LUTpointer = &LUT[0];

uint16_t lutPositionHigh = 0;
uint16_t lutPositionLow = 0;
uint16_t lutIncrementLow = 0;

static void alarm_irq(void);
static void alarm_in_us(uint32_t delay_us);

static void alarm_irq(void) {
    alarm_in_us(10);

    // Clear the alarm irq
    hw_clear_bits(&timer_hw->intr, 1u << ALARM_NUM);

    adc_result = adc_read();

    // a + m*cos(wt)      im = m

    float a = 0.5;
    float m = 0.25;

    float mod = m * ((float)adc_result - 2048.0)/2048.0 + a;

    LUTpointer = &LUT[0];

    // pwm_set_chan_level(slice_num_16, PWM_CHAN_A, (((((uint32_t)mod * 255))*((uint32_t)LUT[lutPositionHigh] - 128)) >> 8) + 128);
    pwm_set_chan_level(slice_num_16, PWM_CHAN_A, (uint32_t)((((mod * 255.0)*((float)(LUT[lutPositionHigh]) - 128.0))/256.0) + 128));
    pwm_set_chan_level(slice_num_15, PWM_CHAN_B, LUTpointer[lutPositionLow]);

    lutPositionHigh = lutPositionHigh + 1;
    lutPositionHigh = lutPositionHigh % LUT_SIZE;

    if(lutIncrementLow % LUT_LOW_DIV == 0) {
        lutPositionLow = lutPositionLow + 1;
        lutPositionLow = lutPositionLow % LUT_SIZE;
    }

    lutIncrementLow = lutPositionLow + 1;
    lutIncrementLow = lutIncrementLow % LUT_LOW_DIV;
}

static void alarm_in_us(uint32_t delay_us) {
    // Enable the interrupt for our alarm (the timer outputs 4 alarm irqs)
    hw_set_bits(&timer_hw->inte, 1u << ALARM_NUM);
    // Set irq handler for alarm irq
    irq_set_exclusive_handler(ALARM_IRQ, alarm_irq);
    // Enable the alarm irq
    irq_set_enabled(ALARM_IRQ, true);
    // Enable interrupt in block and at processor

    // Alarm is only 32 bits so if trying to delay more
    // than that need to be careful and keep track of the upper
    // bits
    uint64_t target = timer_hw->timerawl + delay_us;

    // Write the lower 32 bits of the target time to the alarm which
    // will arm it
    timer_hw->alarm[ALARM_NUM] = (uint32_t) target;
}

int main() {
    gpio_set_function(15, GPIO_FUNC_PWM);
    slice_num_15 = pwm_gpio_to_slice_num(15);
    pwm_set_wrap(slice_num_15, 256);

    gpio_set_function(16, GPIO_FUNC_PWM);
    slice_num_16 = pwm_gpio_to_slice_num(16);
    pwm_set_wrap(slice_num_16, 256);
    
    pwm_set_enabled(slice_num_15, true);
    pwm_set_enabled(slice_num_16, true);

    stdio_init_all();

    adc_init();
    adc_gpio_init(26);
    adc_select_input(0);

    alarm_in_us(10);

    while (1) {
        sleep_ms(1); 
    }
}

/// \end::alarm_standalone[]