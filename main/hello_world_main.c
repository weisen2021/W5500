/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "driver/spi_master.h"
#include "soc/gpio_struct.h"
#include "driver/gpio.h"

#include "w5500_spi.h"
#include "w5500_driver.h"
#include "Led.h"

void app_main()
{
        printf("Hello world!\n");

        /* Print chip information */
        esp_chip_info_t chip_info;
        esp_chip_info(&chip_info);
        printf("This is ESP32 chip with %d CPU cores, WiFi%s%s, ",
               chip_info.cores,
               (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
               (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

        printf("silicon revision %d, ", chip_info.revision);

        printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
               (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

        for (uint8_t j = 0; j < 4; j++)
        {
                uint8_t MAC_EFUSE[6];
                esp_read_mac(&MAC_EFUSE, j);
                printf("MAC1:");
                for (uint8_t i = 0; i < 6; i++)
                {
                        printf(" %01x ", MAC_EFUSE[i]);
                }
                printf("  \n");
        }
        Led_Init();
        spi_init();
        // spi_test();
        w5500_user_int();
}
