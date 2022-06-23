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
#include "freertos/timers.h"

#define GPIO_ENCODER_PHASE_A    4
#define GPIO_ENCODER_PHASE_B    14
#define FILTER_OUT_APB_CLK      100

#define GPIO_PWM0A_OUT 19   //Set GPIO 19 as PWM0A
#define GPIO_PWM0B_OUT 18   //Set GPIO 18 as PWM0B

#define SAMPLE_TIME 10
#define KP 0.15 
#define KI 0.01
#define KD 1
#define DUTY_MAX 100
#define DUTY_MIN 0

double SPEED_RATE = 0;

typedef struct 
{
    int unit;  // the PCNT unit that originated an interrupt
    uint32_t status; // information on the event type that caused the interrupt
} pcnt_evt_t;
pcnt_evt_t event;

typedef struct 
{
    double pid_p;
    double pid_i;
    double pid_d;

    double max;
    double min;

    double kp;
    double ki; 
    double kd;

    double input;
    double output;
} pid_type_t;

TimerHandle_t xTimer1;
double speed_current_error = 0;
double speed_previous_error = 0;
double speed_previousx2_error = 0;
double output_previous = 0;

static void mcpwm_gpio_initialize(void)
{
    mcpwm_pin_config_t mcpwm_gpio_config = {
        .mcpwm0a_out_num = GPIO_PWM0A_OUT,
        .mcpwm0b_out_num = GPIO_PWM0B_OUT,
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

static void pcnt_intr_handler(void *args)
{
    int pcnt_unit = (int)args;
    event.unit = pcnt_unit;
    pcnt_get_event_status(pcnt_unit, &event.status);
    // xQueueSendFromISR(pcnt_evt_queue, &event, NULL);
}

static void pcnt_initialize(void)
{
    pcnt_config_t pcnt_config = {
        .pulse_gpio_num = GPIO_ENCODER_PHASE_A,
        .ctrl_gpio_num = -1,
        .lctrl_mode = PCNT_MODE_KEEP,
        .hctrl_mode = PCNT_MODE_KEEP,
        .pos_mode = PCNT_COUNT_INC,
        .neg_mode = PCNT_COUNT_DIS,
        .counter_h_lim = 30000,
        .counter_l_lim = -10,
        .unit = PCNT_UNIT_0,
        .channel = PCNT_CHANNEL_0
    };
    pcnt_unit_config(&pcnt_config);

    pcnt_set_filter_value(PCNT_UNIT_0, FILTER_OUT_APB_CLK);
    pcnt_filter_enable(PCNT_UNIT_0);

    pcnt_event_enable(PCNT_UNIT_0, PCNT_EVT_ZERO);
    pcnt_event_enable(PCNT_UNIT_0, PCNT_EVT_H_LIM);

    pcnt_counter_pause(PCNT_UNIT_0);
    pcnt_counter_clear(PCNT_UNIT_0);

    pcnt_isr_service_install(0);
    pcnt_isr_handler_add(PCNT_UNIT_0, pcnt_intr_handler, (void*)PCNT_UNIT_0);
}

static void pid_check_limit(pid_type_t *pid)
{
    if(pid->input > pid->max)
    {
        pid->input = pid->max;
    }
    else if(pid->input < pid->min)
    {
        pid->input = pid->min;
    }
}

static void pid_process(pid_type_t *pid, double motor_speed)
{ 
    double alpha, beta, gamma;
    speed_current_error = (double)SPEED_RATE - motor_speed;
    alpha = 2 * SAMPLE_TIME * pid->kp + pid->ki * SAMPLE_TIME * SAMPLE_TIME + 2 * pid->kd;
    beta = SAMPLE_TIME * SAMPLE_TIME * pid->ki - 4 * pid->kd - 2 * SAMPLE_TIME * pid->kp;
    gamma = 2 * pid->kd;
    pid->input = (alpha * speed_current_error + beta * speed_previous_error + gamma * speed_previousx2_error + 2 * SAMPLE_TIME * output_previous) / (2 * SAMPLE_TIME);
    pid_check_limit(pid);
    output_previous = pid->input;
    speed_previousx2_error = speed_previous_error;
    speed_previous_error = speed_current_error;
}

static void pid_config(pid_type_t *pid)
{
    pid->kp = KP;
    pid->ki = KI;
    pid->kd = KD;
    pid->max = DUTY_MAX;
    pid->min = DUTY_MIN;
}

static void read_speed_motor(double *motor_speed)
{
    int16_t encoder_value = 0;
    pcnt_counter_resume(PCNT_UNIT_0);
    vTaskDelay(SAMPLE_TIME / portTICK_PERIOD_MS);
    pcnt_counter_pause(PCNT_UNIT_0);
    pcnt_get_counter_value(PCNT_UNIT_0, (int16_t*)&encoder_value);
    *motor_speed = ((double)encoder_value / 13.36) * 60.0;
    pcnt_counter_clear(PCNT_UNIT_0);
}

void measure_pid_handler(void)
{
    double motor_speed = 0;
    pid_type_t pid;
    pid_config(&pid);
    while(1)
    {
        read_speed_motor(&motor_speed);
        pid_process(&pid, motor_speed);
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, pid.input);
        printf("$%d;", (int)motor_speed);
    }
}

void vTimerCallback(void)
{
    if(SPEED_RATE == 2000)
    {
        SPEED_RATE = 200;
    }
    else
    {
        SPEED_RATE += 200;
    }
}
void app_main(void)
{
    pcnt_initialize();
    mcpwm_gpio_initialize();
    mcpwm_initialize();
    xTaskCreate(measure_pid_handler, "pcnt_measure_handler", 4096, NULL, 5, NULL);
    xTimer1 = xTimerCreate("Timer", 1000 / portTICK_PERIOD_MS, pdTRUE, (void *)0, vTimerCallback);
    xTimerStart(xTimer1, 0);
}

