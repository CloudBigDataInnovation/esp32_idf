#ifndef __COMPONENT_GPIO_CONFIG_LIB_H__
#define __COMPONENT_GPIO_CONFIG_LIB_H__

#include <stdio.h>
#include <esp_log.h>
#include <driver/gpio.h>
#include <hal/gpio_types.h>

#define ESP_INTR_FLAG_DEFAULT   0  

typedef void (*gpio_isr_callback_t) (uint32_t);

void gpio_input_config(gpio_num_t gpio_num, gpio_pullup_t pullup_mode, gpio_pulldown_t pulldown_mode, gpio_int_type_t intr_mode);
void gpio_isr_config(gpio_num_t gpio_num);
void gpio_output_config(gpio_num_t gpio_num, gpio_pullup_t pullup_mode, gpio_pulldown_t pulldown_mode);
void gpio_set_callback(void *addr_callback);

#endif 


