#include <stdio.h>
#include "string.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_attr.h"
#include "soc/rtc.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"
#include "gpio_config_lib.h"
#include "rotary_encoder.h"

#define GPIO_PWM0A_OUT 19   //Set GPIO 19 as PWM0A
#define GPIO_PWM0B_OUT 18   //Set GPIO 18 as PWM0B
#define GPIO_PWM1A_OUT 17   //Set GPIO 17 as PWM1A
#define GPIO_PWM1B_OUT 16   //Set GPIO 16 as PWM1B
#define GPIO_PWM2A_OUT 15   //Set GPIO 15 as PWM2A
#define GPIO_PWM2B_OUT 14   //Set GPIO 14 as PWM2B

static void mcpwm_gpio_initialize(void)
{
    mcpwm_pin_config_t mcpwm_gpio_config = {
        .mcpwm0a_out_num = GPIO_PWM0A_OUT,
        .mcpwm0b_out_num = GPIO_PWM0B_OUT,
        .mcpwm1a_out_num = GPIO_PWM1A_OUT,
        .mcpwm1b_out_num = GPIO_PWM1B_OUT,
        .mcpwm2a_out_num = GPIO_PWM2A_OUT,
        .mcpwm2b_out_num = GPIO_PWM2B_OUT
    };
    mcpwm_set_pin(MCPWM_UNIT_0, &mcpwm_gpio_config);
} 

static void mcpwm_initialize(void)
{
    mcpwm_config_t mcpwm_config = {
        .frequency = 10000,
        .cmpr_a = 0,
        .cmpr_b = 0,
        .counter_mode = MCPWM_UP_COUNTER,
        .duty_mode = MCPWM_DUTY_MODE_0
    };
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &mcpwm_config);
}

static void set_duty_cycle(float duty_cycle)
{
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, duty_cycle);
    printf("Duty cycle = %f\n", duty_cycle);
}

static void mcpwmHandler(void)
{
    //1. mcpwm gpio initialization
    mcpwm_gpio_initialize();

    for(;;)
    {

    }
}

void app_main(void)
{
    printf("Testing MCPWM...\n");
    xTaskCreate(mcpwmHandler, "mcpwmHandler", 4096, NULL, 5, NULL);
}