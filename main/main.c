#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "esp_random.h"

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

void sendData(uint8_t addr, uint8_t data){
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
    sendData(DISPLAY_TEST, 0); // by default it is 1 idk why
    sendData(SCAN_LIMIT, 0x07); // so all 8 rows display
    sendData(DECODE_MODE, 0x00); // set to 1 when using BCD convertor
    sendData(INTENSITY, 0x07); // 0.5 duty cycle
    sendData(SHUTDOWN, 1);
}

static void clear(){
    for (uint8_t i = 1; i < 9; i++) sendData(i, 0x00);
}

void displaySnake(){
    initSPI();
    initMAX7219();
    clear(); // prevent weird glitch when initializing the sensor

    // making the snake initially only 2 dots long
    //generating random row and column to spawn snake in
    uint8_t length = 3; // we need to track of this manually as sizeof(dynamic array) never works, because to sizeof it would always give uint8_t type...
    uint8_t *snake_position = (uint8_t *) malloc (sizeof(uint8_t) * length * 2); // as it will have x and y positions and we are testing with a snake of length 3
    int count = 0;
    uint8_t row = (uint8_t) esp_random()%7 + 1;
    uint8_t col = 1 << ((uint8_t) esp_random()%8);
    // TODO: need to fix controls so that if UP and DOWN or RIGHT and LEFT are pressed together the game ends
    enum Direction {
        UP,
        LEFT,
        DOWN,
        RIGHT};

    // snakes attributes
    snake_position[0] = row; // snake head row
    snake_position[1] = col; // snake head column

    for (int i = 1; i < length; i++){
        snake_position[2*i] = (row - i)%8;
        snake_position[2*i + 1] = col;
    }

    enum Direction dir = UP;

    for(;;){
        clear();
        count += 1;
        
        // displaying snakes position
        for(int i = 0; i < length; i++)
            sendData(snake_position[2*i], snake_position[2*i+1]);

        // updating snake's position
        // hardcoding snake movement to test movement logic
        if (!(count % 3)) dir = (dir + 1)%4;

        switch(dir){
            case UP:
                row = (row)%8 + 1;
                break;
            case DOWN:
                row = (row - 1) ? row - 1: 8; // if row no. is 1, the new row to go to would be 8 
                break;
            case RIGHT:
                col = (col >> 1) > 0 ? (col >> 1) : 128;
                break;
            case LEFT:
                col = (col << 1) > 0 ? (col << 1) : 1;
                break;
        }

        // shifting everything in the array towards the right and discarding the last element
        for(int i = length - 1; i > 0; i--){
            snake_position[2*i] = snake_position[2*i - 2];
            snake_position[2*i + 1] = snake_position[2*i - 1];
            printf("New value %d: %d, %d\n", i, snake_position[2*i], snake_position[2*i + 1]);
        }

        snake_position[0] = row;
        snake_position[1] = col;
        printf("New value 0: %d, %d\n", snake_position[0], snake_position[1]);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    // code should never come here
    free(snake_position);
}

void app_main(void)
{
    TaskHandle_t display_handle = NULL;
    uint8_t spi_data[2] = {0x01, 0x08};

    xTaskCreate(displaySnake, "snake", 4096, (void *) spi_data, 2, &display_handle);

    while(1){
        printf("Running main task\n");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}