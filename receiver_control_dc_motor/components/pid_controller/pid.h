#ifndef _PID_H_
#define _PID_H_

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "string.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_attr.h"
#include "soc/rtc.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"
#include "driver/pcnt.h"

typedef enum pid_control_directions 
{
	E_PID_DIRECT,
	E_PID_REVERSE,
};

typedef struct pid_controller 
{
	float *input; 
	float *output; 
	float *set_point; 

	float Kp; 
	float Ki; 
	float Kd; 

	float omin; 
	float omax; 

	float iterm;
	float lastin; 

	uint32_t last_time; 
	uint32_t sample_time; 

	uint8_t auto_mode; 
	enum pid_control_directions direction;
} pid_controller_t;


#endif
