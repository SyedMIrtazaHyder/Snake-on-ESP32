#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"

#include "sdkconfig.h"

#define CLK 14
#define DIN 13
#define CS 15
#define HOST SPI2_HOST

esp_err_t ret; 
void vDisplayLED(void* pvParams){
    // Setting SPI communication
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = DIN,
        .miso_io_num = -1,
        .sclk_io_num = CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1
    };

    spi_device_interface_config_t dev_cfg = {
        .address_bits = 8,
        .spics_io_num = CS,
    };

    spi_device_handle_t led_handle = NULL;

    ret = spi_bus_initialize(HOST, &bus_cfg, SPI_DMA_CH_AUTO); // initializing bus
    ESP_ERROR_CHECK(ret);

    ret = spi_bus_add_device(HOST, &dev_cfg, &led_handle); // adding led device to the bus
    ESP_ERROR_CHECK(ret);

    uint8_t arr[] = {0x01};

    spi_transaction_t msg = {
        .addr = 0x01,
        .length = 16,
        .tx_buffer = arr,
    };

    // Sending the desired bits
    while (1){
        spi_device_transmit(led_handle, &msg);
        // updating the address and bits to be displayed
        arr[0] = arr[0] << 1;
        msg.addr = (msg.addr + 1)%8 + 1;
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main(void)
{
    TaskHandle_t vdisplay = NULL;

    xTaskCreate(vDisplayLED, "display_LED", 2048, NULL, 1, &vdisplay);
    while(1){
        printf("Main Task\n");
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}