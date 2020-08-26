

#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "delay.h"
#include "lcd.h"
#include "gfx.h"
#include "gfx_menu.h"


static xQueueHandle gpio_evt_queue = NULL;
static void application_thread(void * pvParameters);
void application_touch_isr_handler(uint32_t pin);

void application_start(void)
{
    xTaskCreate(application_thread, "application_thread", 4096, NULL, 10, NULL);
}

static void application_thread(void * pvParameters)
{
    uint32_t io_num;
    uint8_t ts_count = 0;
    uint16_t ts_x[5] = {0}, ts_y[5] = {0};
    uint16_t colors[5] = {
        GFX_COLOR_RED,
        GFX_COLOR_GREEN,
        GFX_COLOR_BLUE,
        GFX_COLOR_BLACK,
        GFX_COLOR_LIGHT_BLUE
    };

    printf("application_thread\n");

    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));

    
    gfx_backligh(0);
    gfx_init(application_touch_isr_handler);
    gfx_menu_main();
    gfx_backligh(1);

    for(;;) {
        
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {//delay_ms_to_ticks(100))) {
            //printf("GPIO[%d] intr, val: %d\n", io_num, gpio_get_level(io_num));
            if (io_num && gpio_get_level(io_num) == 0) {
                ts_count = gfx_ts_read(ts_x, ts_y);
                if (ts_count == 1 && ts_x[0] >= 600 && ts_x[0] <= 800 && ts_y[0] >= 420 && ts_y[0] <= 480) {
                    lcd_clear(GFX_COLOR_WHITE);
                    gfx_menu_main(); 
                } else {
                    for (uint8_t i = 0 ; i < ts_count; i ++) {
                        //printTouchXY(i, ts_x[i], ts_y[i]);
                        //lcd_draw_point(ts_x[i], ts_y[i], colors[i]);
                        lcd_fill(ts_x[i] - 3, ts_y[i] - 3, ts_x[i] + 3, ts_y[i] + 3, colors[i]);
                    }
                }
            }
        }
        //delay_rtos(1);
    }
}

void application_touch_isr_handler(uint32_t pin)
{
    xQueueSendFromISR(gpio_evt_queue, &pin, NULL);
}