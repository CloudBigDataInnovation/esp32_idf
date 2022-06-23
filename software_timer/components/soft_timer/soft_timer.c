#include "soft_timer.h"
TimerHandle_t xTimers[NUM_TIMERS];

void createSoftTimer(void)
{
    uint32_t index = 0;
    for(index = 0; index < NUM_TIMERS; index++)
    {
        xTimers[index] = xTimerCreate(
            "Timer",
            (100 * x) + 100,
            pdTRUE,
            (void *)0,
            vTimerCallback
        );
    }
} 

void 