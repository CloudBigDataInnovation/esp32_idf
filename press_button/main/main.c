#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "gpio_config_lib.h"
#include "freertos/event_groups.h"

#define BIT_FAST        ( 1 << 0 )
#define BIT_MEDIUM      ( 1 << 1 )
#define BIT_SLOW        ( 1 << 2 )
#define BIT_TIMEOUT     ( 1 << 3 )
EventGroupHandle_t intrFlagGroup;
EventBits_t uxBits, valueFlag;
BaseType_t xHigherPriorityTaskWoken = pdFALSE;
uint64_t tickstart = 0, tickcurrent = 0, time = 0;

void gpio_button_callback(uint32_t gpio_num)
{
    if(gpio_num == GPIO_NUM_18)
    {
        if(gpio_get_level(GPIO_NUM_18) == 0)
        {
            tickstart = xTaskGetTickCountFromISR();
        }
        else
        {
            tickcurrent = xTaskGetTickCountFromISR();
            time = (tickcurrent - tickstart) * portTICK_PERIOD_MS;
            if(time < 1000)
            {
                xEventGroupSetBitsFromISR(
                              intrFlagGroup,
                              BIT_FAST,
                              &xHigherPriorityTaskWoken);
            }
            else if(time >= 1000 && time < 3000)
            {
                xEventGroupSetBitsFromISR(
                              intrFlagGroup,
                              BIT_MEDIUM,
                              &xHigherPriorityTaskWoken);
            }
            else if(time >= 3000 && time < 5000)
            {
                xEventGroupSetBitsFromISR(
                              intrFlagGroup,
                              BIT_SLOW,
                              &xHigherPriorityTaskWoken);
            }
            else if(time >= 5000)
            {
                xEventGroupSetBitsFromISR(
                              intrFlagGroup,
                              BIT_TIMEOUT,
                              &xHigherPriorityTaskWoken);
            }
        }
    }
}

void vTaskPrintMode(void *pvParameter)
{
    for( ;; )
    {
        valueFlag = xEventGroupWaitBits(
            intrFlagGroup,
            BIT_FAST | BIT_MEDIUM | BIT_SLOW | BIT_TIMEOUT,
            pdTRUE,
            pdFALSE,
            portMAX_DELAY
        );
        if((valueFlag & BIT_FAST) != 0)
        {
            printf("Fast press\n");
        }
        else if((valueFlag & BIT_MEDIUM) != 0)
        {
            printf("Medium press\n");
        }
        else if((valueFlag & BIT_SLOW) != 0)
        {
            printf("Slow press\n");            
        }
        else if((valueFlag & BIT_TIMEOUT) != 0)
        {
            printf("Time out\n");
        }
    }
}

void app_main(void)
{
    gpio_input_config(GPIO_NUM_18, GPIO_PULLUP_ENABLE, GPIO_PULLDOWN_DISABLE, GPIO_INTR_ANYEDGE);
    gpio_isr_config(GPIO_NUM_18);
    // gpio_output_config(GPIO_NUM_19, GPIO_PULLUP_DISABLE, GPIO_PULLDOWN_DISABLE);
    gpio_set_callback(gpio_button_callback);
    xTaskCreate(
        vTaskPrintMode,
        "vTaskPrintMode",
        2048,
        NULL,
        4,
        NULL
    );
    intrFlagGroup = xEventGroupCreate();
    vTaskStartScheduler();
    while(1)
    {

    }
}