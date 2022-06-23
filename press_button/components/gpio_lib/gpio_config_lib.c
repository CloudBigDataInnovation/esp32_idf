#include "gpio_config_lib.h"

gpio_isr_callback_t gpio_isr_callback = NULL;

static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    uint32_t gpio_num = (uint32_t)arg;
    gpio_isr_callback(gpio_num);
}

void gpio_input_config(gpio_num_t gpio_num, gpio_pullup_t pullup_mode, gpio_pulldown_t pulldown_mode, gpio_int_type_t intr_mode)
{
    gpio_config_t gpio_input =
    {
        .pin_bit_mask = BIT64(gpio_num),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = pullup_mode,
        .pull_down_en = pulldown_mode,
        .intr_type = intr_mode
    };
    gpio_config(&gpio_input);
}

void gpio_isr_config(gpio_num_t gpio_num)
{
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    gpio_isr_handler_add(gpio_num, gpio_isr_handler, (void*)gpio_num);
}

void gpio_output_config(gpio_num_t gpio_num, gpio_pullup_t pullup_mode, gpio_pulldown_t pulldown_mode)
{
    gpio_config_t gpio_output =
    {
        .pin_bit_mask = BIT64(gpio_num),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = pullup_mode,
        .pull_down_en = pulldown_mode,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&gpio_output);
}

void gpio_set_callback(void *addr_callback)
{
    gpio_isr_callback = addr_callback;
}