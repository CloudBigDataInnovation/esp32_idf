/**
 * @file mpu6050.c
 * @author Vanperdung (dung.nv382001@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2022-06-16
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <stdio.h>
#include <math.h>
#include "driver/i2c.h"
#include "sdkconfig.h"
#include "string.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_attr.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"
#include "driver/pcnt.h"
#include "freertos/timers.h"

//dc motor
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
//end


double SPEED_RATE = 0;

#define GPIO_I2C_SDA 21
#define GPIO_I2C_SCL 22
#define I2C_CLOCK_FREQ 100000

#define MPU6050_ADDR 0x68
#define WRITE_BIT I2C_MASTER_WRITE             
#define READ_BIT I2C_MASTER_READ  
#define ACK_CHECK_EN 0x1                        
#define ACK_CHECK_DIS 0x0                       
#define ACK_VAL 0x0                            
#define NACK_VAL 0x1

#define SMPLRT_DIV_REG 0x19
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define ACCEL_XOUT_L_REG 0x3C
#define ACCEL_YOUT_H_REG 0x3D
#define ACCEL_YOUT_L_REG 0x3E
#define ACCEL_ZOUT_H_REG 0x3F
#define ACCEL_ZOUT_L_REG 0x40
#define TEMP_OUT_H_REG 0x41
#define GYRO_XOUT_H_REG 0x43
#define GYRO_XOUT_L_REG 0x44
#define GYRO_YOUT_H_REG 0x45
#define GYRO_YOUT_L_REG 0x46
#define GYRO_ZOUT_H_REG 0x47
#define GYRO_ZOUT_L_REG 0x48
#define PWR_MGMT_1_REG 0x6B
#define WHO_AM_I_REG 0x75

#define RAD_TO_DEG 57.295779513082320876798154814105

i2c_port_t i2c_port = 0;

typedef struct
{
    int16_t accel_x_raw;
    int16_t accel_y_raw;
    int16_t accel_z_raw;
    double ax;
    double ay;
    double az;
    int16_t gyro_x_raw;
    int16_t gyro_y_raw;
    int16_t gyro_z_raw;
    double gx;
    double gy;
    double gz;
    double KalmanAngleX;
    double KalmanAngleY;
    double KalmanAngleZ;
    double angle_x;
    double angle_y;
    double angle_z
} MPU6050_t;

// Kalman structure
typedef struct
{
    double Q_angle;
    double Q_bias;
    double R_measure;
    double angle;
    double bias;
    double P[2][2];
} Kalman_t;

Kalman_t KalmanX = {
    .Q_angle = 0.001f,
    .Q_bias = 0.003f,
    .R_measure = 0.03f
};

Kalman_t KalmanY = {
    .Q_angle = 0.001f,
    .Q_bias = 0.003f,
    .R_measure = 0.03f,
};

/** Handle to an interrupt handler */
intr_handle_t i2c_master_intr_handle = NULL; // Can be use to delete an interrupt handler

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

//dc motor
TimerHandle_t Timer;
double speed_current_error = 0;
double speed_previous_error = 0;
double speed_previousx2_error = 0;
double output_previous = 0;
MPU6050_t hmpu6050 = {0};
bool mpu6050_read = false;

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

double kalman_get_angle(Kalman_t *Kalman, double newAngle, double newRate, double dt)
{
    double rate = newRate - Kalman->bias;
    Kalman->angle += dt * rate;

    Kalman->P[0][0] += dt * (dt * Kalman->P[1][1] - Kalman->P[0][1] - Kalman->P[1][0] + Kalman->Q_angle);
    Kalman->P[0][1] -= dt * Kalman->P[1][1];
    Kalman->P[1][0] -= dt * Kalman->P[1][1];
    Kalman->P[1][1] += Kalman->Q_bias * dt;

    double S = Kalman->P[0][0] + Kalman->R_measure;
    double K[2];
    K[0] = Kalman->P[0][0] / S;
    K[1] = Kalman->P[1][0] / S;

    double y = newAngle - Kalman->angle;
    Kalman->angle += K[0] * y;
    Kalman->bias += K[1] * y;

    double P00_temp = Kalman->P[0][0];
    double P01_temp = Kalman->P[0][1];

    Kalman->P[0][0] -= K[0] * P00_temp;
    Kalman->P[0][1] -= K[0] * P01_temp;
    Kalman->P[1][0] -= K[1] * P00_temp;
    Kalman->P[1][1] -= K[1] * P01_temp;

    return Kalman->angle;
};

static void i2c_master_initialize(i2c_port_t i2c_num)
{
    i2c_config_t i2c_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = GPIO_I2C_SDA,
        .scl_io_num = GPIO_I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_CLOCK_FREQ
    };
    i2c_param_config(i2c_num, &i2c_config);
    // i2c_isr_register(i2c_master_port, i2c_isr_handler, NULL, 0, i2c_master_intr_handle);
    i2c_driver_install(i2c_num, i2c_config.mode, 0, 0, 0);
}

/**
 * ___________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write n bytes + ack  | stop |
 * --------|---------------------------|----------------------|------|
 */
static esp_err_t i2c_master_write_mpu6050(i2c_port_t i2c_num, uint8_t reg_addr, uint8_t *data_wr, size_t size)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN); 
    i2c_master_write(cmd, data_wr, size, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

/**
 * _______________________________________________________________________________________
 * | start | slave_addr + rd_bit +ack | read n-1 bytes + ack | read 1 byte + nack | stop |
 * --------|--------------------------|----------------------|--------------------|------|
 */
static esp_err_t i2c_master_read_mpu6050(i2c_port_t i2c_num, uint8_t reg_addr, uint8_t *data_rd, size_t size)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    cmd = i2c_cmd_link_create();
    // i2c_master_write_mpu6050(i2c_num, reg_addr, NULL, 0);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | READ_BIT, ACK_CHECK_EN);
    if(size >  1)
    {
        i2c_master_read(cmd, data_rd, size - 1, ACK_VAL); 
    }
    i2c_master_read_byte(cmd, data_rd, NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static void mpu6050_initialize(i2c_port_t i2c_num)
{
    uint8_t data_check = 0, data_write = 0;
    i2c_master_read_mpu6050(i2c_num, WHO_AM_I_REG, &data_check, 1);
    if(data_check == 104) // device is present
    {
        printf("MPU6050 is present...\n");
        // wake the sensor up
        data_write = 0;
        i2c_master_write_mpu6050(i2c_num, PWR_MGMT_1_REG, &data_write, 1);

        // set data rate or sample rate equals 1KHz
        data_write = 7;
        i2c_master_write_mpu6050(i2c_num, SMPLRT_DIV_REG, &data_write, 1);

        // configure the full scale range of Accelerometer and Gyroscope
        data_write = 0;
        i2c_master_write_mpu6050(i2c_num, GYRO_CONFIG_REG, &data_write, 1);
        i2c_master_write_mpu6050(i2c_num, ACCEL_CONFIG_REG, &data_write, 1);
    }
}  

static void mpu6050_read_data_accelerometer(i2c_port_t i2c_num, MPU6050_t *hmpu6050)
{
    uint8_t data_read[6] = {0};
    i2c_master_read_mpu6050(i2c_num, ACCEL_XOUT_H_REG, data_read, 1);
    i2c_master_read_mpu6050(i2c_num, ACCEL_XOUT_L_REG, data_read + 1, 1);
    i2c_master_read_mpu6050(i2c_num, ACCEL_YOUT_H_REG, data_read + 2, 1);
    i2c_master_read_mpu6050(i2c_num, ACCEL_YOUT_L_REG, data_read + 3, 1);
    i2c_master_read_mpu6050(i2c_num, ACCEL_ZOUT_H_REG, data_read + 4, 1);
    i2c_master_read_mpu6050(i2c_num, ACCEL_ZOUT_L_REG, data_read + 5, 1);
    hmpu6050->accel_x_raw = (int16_t)(data_read[0] << 8 | data_read[1]);
    hmpu6050->accel_y_raw = (int16_t)(data_read[2] << 8 | data_read[3]);
    hmpu6050->accel_z_raw = (int16_t)(data_read[4] << 8 | data_read[5]);

    // convert the RAW values into acceleration in 'g'
    hmpu6050->ax = hmpu6050->accel_x_raw / 16384.0;
    hmpu6050->ay = hmpu6050->accel_y_raw / 16384.0;
    hmpu6050->az = hmpu6050->accel_z_raw / 16384.0;
}

static void mpu6050_read_data_all(i2c_port_t i2c_num, MPU6050_t *hmpu6050)
{
    double angle_x_sqrt, dt, angle_y_sqrt;
    uint32_t timer = 0;
    uint8_t accel_read[6] = {0};
    uint8_t gyro_read[6] = {0};
    uint8_t *accel_data = accel_read;
    uint8_t *gyro_data = gyro_read;

    i2c_master_read_mpu6050(i2c_num, ACCEL_XOUT_H_REG, accel_data++, 1);
    i2c_master_read_mpu6050(i2c_num, ACCEL_XOUT_L_REG, accel_data++, 1);
    i2c_master_read_mpu6050(i2c_num, ACCEL_YOUT_H_REG, accel_data++, 1);
    i2c_master_read_mpu6050(i2c_num, ACCEL_YOUT_L_REG, accel_data++, 1);
    i2c_master_read_mpu6050(i2c_num, ACCEL_ZOUT_H_REG, accel_data++, 1);
    i2c_master_read_mpu6050(i2c_num, ACCEL_ZOUT_L_REG, accel_data++, 1);
    hmpu6050->accel_x_raw = (int16_t)(accel_read[0] << 8 | accel_read[1]);
    hmpu6050->accel_y_raw = (int16_t)(accel_read[2] << 8 | accel_read[3]);
    hmpu6050->accel_z_raw = (int16_t)(accel_read[4] << 8 | accel_read[5]);

    // convert the RAW values into acceleration in 'g'
    hmpu6050->ax = hmpu6050->accel_x_raw / 16384.0;
    hmpu6050->ay = hmpu6050->accel_y_raw / 16384.0;
    hmpu6050->az = hmpu6050->accel_z_raw / 16384.0;

    i2c_master_read_mpu6050(i2c_num, GYRO_XOUT_H_REG, gyro_data++, 1);
    i2c_master_read_mpu6050(i2c_num, GYRO_XOUT_L_REG, gyro_data++, 1);
    i2c_master_read_mpu6050(i2c_num, GYRO_YOUT_H_REG, gyro_data++, 1);
    i2c_master_read_mpu6050(i2c_num, GYRO_YOUT_L_REG, gyro_data++, 1);
    i2c_master_read_mpu6050(i2c_num, GYRO_ZOUT_H_REG, gyro_data++, 1);
    i2c_master_read_mpu6050(i2c_num, GYRO_ZOUT_L_REG, gyro_data++, 1);
    hmpu6050->gyro_x_raw = (int16_t)(gyro_read[0] << 8 | gyro_read[1]);
    hmpu6050->gyro_y_raw = (int16_t)(gyro_read[2] << 8 | gyro_read[3]);
    hmpu6050->gyro_z_raw = (int16_t)(gyro_read[4] << 8 | gyro_read[5]);

    // convert the RAW values into dps (°/s)
    hmpu6050->gx = hmpu6050->gyro_x_raw / 131.0 + 2.9;
    hmpu6050->gy = hmpu6050->gyro_y_raw / 131.0 - 1.8;
    hmpu6050->gz = hmpu6050->gyro_z_raw / 131.0 - 0.7;    

    dt = (double)((xTaskGetTickCount() - timer) * portTICK_PERIOD_MS);
    timer = xTaskGetTickCount();
    angle_x_sqrt = sqrt(hmpu6050->accel_x_raw * hmpu6050->accel_x_raw + hmpu6050->accel_z_raw * hmpu6050->accel_z_raw);
    if(angle_x_sqrt != 0.0)
    {
        hmpu6050->angle_x = atan(-1 * hmpu6050->accel_y_raw / angle_x_sqrt) * RAD_TO_DEG;
    }
    else
    {
        hmpu6050->angle_x = 0.0;
    }
    angle_y_sqrt = sqrt(hmpu6050->accel_y_raw * hmpu6050->accel_y_raw + hmpu6050->accel_z_raw * hmpu6050->accel_z_raw);
    if(angle_y_sqrt != 0.0)
    {
        hmpu6050->angle_y = atan(hmpu6050->accel_x_raw / angle_y_sqrt) * RAD_TO_DEG;
    }
    else
    {
        hmpu6050->angle_y = 0.0;
    }
    
    hmpu6050->KalmanAngleX = kalman_get_angle(&KalmanX, hmpu6050->angle_x, hmpu6050->gx, dt);
    hmpu6050->KalmanAngleY = kalman_get_angle(&KalmanY, hmpu6050->angle_y, hmpu6050->gy, dt);
}

static void mpu6050_read_data_gyroscope(i2c_port_t i2c_num, MPU6050_t *hmpu6050)
{
    uint8_t data_read[6] = {0};
    i2c_master_read_mpu6050(i2c_num, GYRO_XOUT_H_REG, data_read, 1);
    i2c_master_read_mpu6050(i2c_num, GYRO_XOUT_L_REG, data_read + 1, 1);
    i2c_master_read_mpu6050(i2c_num, GYRO_YOUT_H_REG, data_read + 2, 1);
    i2c_master_read_mpu6050(i2c_num, GYRO_YOUT_L_REG, data_read + 3, 1);
    i2c_master_read_mpu6050(i2c_num, GYRO_ZOUT_H_REG, data_read + 4, 1);
    i2c_master_read_mpu6050(i2c_num, GYRO_ZOUT_L_REG, data_read + 5, 1);
    hmpu6050->gyro_x_raw = (int16_t)(data_read[0] << 8 | data_read[1]);
    hmpu6050->gyro_y_raw = (int16_t)(data_read[2] << 8 | data_read[3]);
    hmpu6050->gyro_z_raw = (int16_t)(data_read[4] << 8 | data_read[5]);

    // convert the RAW values into dps (°/s)
    hmpu6050->gx = hmpu6050->gyro_x_raw / 131.0 + 2.9;
    hmpu6050->gy = hmpu6050->gyro_y_raw / 131.0 - 1.8;
    hmpu6050->gz = hmpu6050->gyro_z_raw / 131.0 - 0.7;    
}

void read_mpu6050_handler(void)
{
    int16_t encoder_value = 0;
    double motor_speed = 0;
    pid_type_t pid;
    pid_config(&pid);
    mpu6050_read_data_all(i2c_port, &hmpu6050);
    while(1)
    {
        // mpu6050_read_data_accelerometer(i2c_port, &hmpu6050);
        // mpu6050_read_data_gyroscope(i2c_port, &hmpu6050);
        // printf("Accelerometer: Ax = %lf, Ay = %lf, Az = %lf\n", hmpu6050.ax, hmpu6050.ay, hmpu6050.az);
        // printf("Gyroscope: Gx = %lf, Gy = %lf, Gz = %lf\n", hmpu6050.gx, hmpu6050.gy, hmpu6050.gz);
        // mpu6050_read_data_all(i2c_port, &hmpu6050);
        // printf("$%lf %lf;", hmpu6050.KalmanAngleX, hmpu6050.KalmanAngleY);
        if(abs(hmpu6050.KalmanAngleY) > 0 && abs(hmpu6050.KalmanAngleY) <= 45)
        {
            SPEED_RATE = 800;
            // printf("$%d;", (int)motor_speed);
        }
        else if(abs(hmpu6050.KalmanAngleY) > 45 && abs(hmpu6050.KalmanAngleY) <= 90)
        {
            SPEED_RATE = 1600;
            // printf("$%d;", (int)motor_speed);
        }
        read_speed_motor(&motor_speed);
        pid_process(&pid, motor_speed);
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, pid.input);
        printf("$%lf %lf %lf;", hmpu6050.KalmanAngleX, hmpu6050.KalmanAngleY, motor_speed);
    }
}

void read_mpu6050_callback(void)
{
    mpu6050_read_data_all(i2c_port, &hmpu6050);
}

// void pid_process_callback(void)
// {
//     speed = true
// }

void app_main(void)
{
    //dc motor
    pcnt_initialize();
    mcpwm_gpio_initialize();
    mcpwm_initialize();
    //end
    i2c_master_initialize(i2c_port);
    mpu6050_initialize(i2c_port);

    xTaskCreate(read_mpu6050_handler, "read_mpu6050_handler", 4096, NULL, 6, NULL);
    Timer = xTimerCreate("Timer_mpu6050", 100 / portTICK_PERIOD_MS, pdTRUE, (void *)0, read_mpu6050_callback);
    // xTimer[1] = xTimerCreate("Timer2", SAMPLE_TIME / portTICK_PERIOD_MS, pdTRUE, (void *)0, pid_process_callback);
    xTimerStart(Timer, 0);
    // xTimerStart(xTimer[1], 0);
    // vTaskStartScheduler(); // ESP32 don't need to call?
}