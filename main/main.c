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

// reference https://www.embeddedexplorer.com/esp32-spi-master/
// https://www.analog.com/media/en/technical-documentation/data-sheets/MAX7219-MAX7221.pdf
// current code does not display anything idk why
// reference code is working as expected
// changed the config values but still no effect
// changed transmit to polling_transmit but no change

#define DECODE_MODE 0x09
#define INTENSITY 0x0A
#define SCAN_LIMIT 0x0B
#define SHUTDOWN 0x0C
#define DISPLAY_TEST 0x0F


spi_device_handle_t dot_matrix = NULL;

static void initSPI(){
    esp_err_t ret; 
    // Setting SPI communication
    // configuring SPI bus
    spi_bus_config_t buscfg={
        .miso_io_num = -1,
        .mosi_io_num = DIN,
        .sclk_io_num = CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32,
    };

    // configuring spi slave device 
    spi_device_interface_config_t devcfg={
        .clock_speed_hz = 1000000,  // 1 MHz
        .mode = 0,                  //SPI mode 0
        .spics_io_num = CS,     
        .queue_size = 1,
        .flags = SPI_DEVICE_HALFDUPLEX,
        .pre_cb = NULL,
        .post_cb = NULL,
    };


    ret = spi_bus_initialize(HOST, &buscfg, SPI_DMA_CH_AUTO); // initializing bus
    ESP_ERROR_CHECK(ret);

    ret = spi_bus_add_device(HOST, &devcfg, &dot_matrix); // adding dot matrix to the bus
    ESP_ERROR_CHECK(ret);
}

void send_data(uint8_t addr, uint8_t data){
    // if using dynamic memory we need to free this later as well
    // uint8_t *spi_send = (uint8_t *) malloc (8*2); // allocating dynamic array of 16 bits that has the address and data we need to write to
    // spi_send[0] = addr;
    // spi_send[1] = data;

    uint8_t spi_send[] = {addr, data};

    spi_transaction_t msg = {
        // .addr = 0x01,
        .length = 8*2,
        .tx_buffer = spi_send,
    };

    // Sending the desired bits
    ESP_ERROR_CHECK(spi_device_polling_transmit(dot_matrix, &msg));
}

static void initMAX7219(){
    send_data(DISPLAY_TEST, 0); // by default it is 1 idk why
    send_data(SCAN_LIMIT, 0x07); // so all 8 rows display
    send_data(DECODE_MODE, 0x00); // set to 1 when using BCD convertor
    send_data(INTENSITY, 0x07); // 0.5 duty cycle
    send_data(SHUTDOWN, 1);
}

static void clear(){
    for (uint8_t i = 1; i < 9; i++) send_data(i, 0x00);
}

static void displayDots(void *pvParameteres){
    initSPI();
    initMAX7219();
    clear(); // prevent weird glitch when initializing the sensor

    for (;;){
        uint8_t spi_data[2] = {*(uint8_t *) pvParameteres, *(uint8_t *) (pvParameteres + sizeof(uint8_t))};
        send_data(spi_data[0], spi_data[1]);
        vTaskDelay(pdMS_TO_TICKS(1000));
        clear();
        vTaskDelay(pdMS_TO_TICKS(1000));
        // addition is in bytes so sizeof also given in bytes
        printf("LED Blink %d %d\n", sizeof(uint8_t), sizeof(uint16_t));
    }
}

void app_main(void)
{
    TaskHandle_t display_handle = NULL;
    uint8_t spi_data[2] = {0x01, 0x08};

    xTaskCreatePinnedToCore(displayDots, "led_display", 2048, (void *) spi_data, 2, &display_handle, 1);

    while(1){
        printf("Running main task\n");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}