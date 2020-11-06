

#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "delay.h"
#include "lcd.h"
#include "gfx.h"
#include "gfx_menu.h"
#include "nrf24l01.h"


#define nRF24_WAIT_TIMEOUT         (uint32_t)0x000FFFFF


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

    uint8_t nrf_payload[32];
    nrf_rx_result nrf_pipe;
    uint8_t nrf_payload_length;

    printf("application_thread\n");

    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));


    gfx_backligh(0);
    gfx_init(application_touch_isr_handler);
    gfx_menu_main();
    gfx_backligh(1);


    nrf_init();
    uint8_t check = nrf_check();
    printf("nrf check %x\r\n", check);

    for(;;) {
        
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {//delay_ms_to_ticks(100))) {
            //printf("GPIO[%d] intr, val: %d\n", io_num, gpio_get_level(io_num));
            if (io_num && gpio_get_level(io_num) == 0) {
                ts_count = gfx_ts_read(ts_x, ts_y);
                if (ts_count == 0)
                    continue;
                
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
        if (nrf_get_status_rx_fifo() != NRF_STATUS_RXFIFO_EMPTY) {
    		nrf_pipe = nrf_read_payload(nrf_payload, &nrf_payload_length);
			nrf_clear_irq_flags();

			printf("received pipe: %d\r\n\tpayload: ", nrf_pipe);
            for(int i = 0; i < nrf_payload_length; i++)
                printf("%2x ", nrf_payload[i]);
            printf("\r\n");
    	}
        //delay_rtos(1);
    }
}

void application_touch_isr_handler(uint32_t pin)
{
    xQueueSendFromISR(gpio_evt_queue, &pin, NULL);
}