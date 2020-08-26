
#include "esp_system.h"

void delay_rtos(int ms);
void delay_hw(int ms);
uint32_t delay_ticks_to_ms(uint32_t ticks);
uint32_t delay_ms_to_ticks(uint32_t ms);