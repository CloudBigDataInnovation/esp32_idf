#ifndef __COMPONENTS_SOFT_TIMER_H__
#define __COMPONENTS_SOFT_TIMER_H__

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "freertos/timers.h"

#define NUM_TIMERS 5

void vTimerCallback(TimerHandle_t xTimer);

#endif