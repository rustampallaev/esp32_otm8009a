#include "delay.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void delay_rtos(int ms)
{
    vTaskDelay(delay_ticks_to_ms(ms));
}

void delay_hw(int ms)
{
    delay_rtos(ms);
}

uint32_t delay_ticks_to_ms(uint32_t ticks)
{
    return (portTICK_PERIOD_MS / ticks);
}

uint32_t delay_ms_to_ticks(uint32_t ms)
{
    return (ms / portTICK_PERIOD_MS);
}