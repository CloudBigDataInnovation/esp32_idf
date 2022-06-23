/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "freertos/timers.h"
#define NUM_TIMERS 5

TimerHandle_t xTimer[NUM_TIMERS] = {0};

void vTimerCallback(TimerHandle_t xTimers)
{
    uint32_t ulCount = 0;
    configASSERT(xTimers);
    // ulCount = (uint32_t)pvTimerGetTimerID(xTimers);
    if(xTimers == xTimer[0])
    {
        printf("Hello\n");
    }
    else if(xTimers == xTimer[1])
    {
        printf("Bye Bye\n");
    }
}

void app_main(void)
{
    xTimer[0] = xTimerCreate(
                                "vTimerHello",
                                pdMS_TO_TICKS(500),
                                pdTRUE,
                                (void*)0,
                                vTimerCallback
    );
    xTimer[1] = xTimerCreate(
                                "vTimerBye",
                                pdMS_TO_TICKS(1000),
                                pdTRUE,
                                (void*)1,
                                vTimerCallback
    );
    for (uint32_t index = 0; index < NUM_TIMERS; index++)
    {
        if(xTimer[index] == 0)
        {

        }
        else
        {
            xTimerStart(xTimer[index], 0);
        }
    }
    

}
