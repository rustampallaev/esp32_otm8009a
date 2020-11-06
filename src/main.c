#include <stdio.h>
#include "sdkconfig.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "application.h"

#ifdef CONFIG_IDF_TARGET_ESP32
#define CHIP_NAME "ESP32"
#endif

#ifdef CONFIG_IDF_TARGET_ESP32S2BETA
#define CHIP_NAME "ESP32-S2 Beta"
#endif

uint16_t buff[800*480] = {0};

void app_main(void)
{
        printf("app_main\n");

        esp_chip_info_t chip_info;
        esp_chip_info(&chip_info);
        printf("This is %s chip with %d CPU cores, WiFi%s%s, ",
                CHIP_NAME,
                chip_info.cores,
                (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
                (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

        printf("silicon revision %d, ", chip_info.revision);

        printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
                (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");
    


        application_start();
}
