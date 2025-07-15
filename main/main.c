#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/uart.h"
#include "esp_random.h"
#include "esp_system.h"

#include "sdkconfig.h"

#define UART_NUM UART_NUM_0
#define UART_RX 3
#define UART_TX 1
#define CTS -1
#define RTS -1

const int uart_buffer_size = 1024;
char data;

void init_uart(){
    QueueHandle_t uart_queue;
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, uart_buffer_size, uart_buffer_size, 10, &uart_queue, 0));

    // setting up communication parameters
    uart_config_t uart_cfg = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122
    };
    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_cfg));
    
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM, UART_TX, UART_RX , RTS, CTS));
}

// reference https://www.embeddedexplorer.com/esp32-spi-master/
// https://www.analog.com/media/en/technical-documentation/data-sheets/MAX7219-MAX7221.pdf
#define CLK 14
#define DIN 13
#define CS 15
#define HOST SPI2_HOST

#define DECODE_MODE 0x09
#define INTENSITY 0x0A
#define SCAN_LIMIT 0x0B
#define SHUTDOWN 0x0C
#define DISPLAY_TEST 0x0F

enum Direction {
	UP,
	LEFT,
	DOWN,
	RIGHT};

spi_device_handle_t dot_matrix = NULL;
uint8_t grid[8];
enum Direction dir = UP;

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

static void clearGrid(){
    for (uint8_t i = 0; i < 8; i++) grid[i] = 0;
}

static void displayGrid(){
    for (uint8_t i = 1; i < 9; i++) sendData(i, grid[i-1]);
}

static void clear(){
    clearGrid();
    displayGrid();
}

static void rcToLED(uint8_t *arr, uint8_t const length){
    uint8_t row, col;
    for (int i = 0; i < length; i++){
		row = arr[2*i];
		col = arr[2*i + 1];
		grid[row] = grid[row] | (1 << col);
    }
}

void setApple(uint8_t* apple_loc){
	// Oring the apple location with the grid row where the snake might exists, so if the apple overlaps the snake we regenerate it
	uint8_t row = random()%8;
	uint8_t col = random()%8;
	
	while (grid[row] == 0xFF) row = random()%8;

	while ((grid[row] | (1 << col)) == grid[row]){
		col = random()%8;
		row = random()%8;
	}

	apple_loc[0] = row;
	apple_loc[1] = col;
}

bool appleEaten(const uint8_t* apple_loc, const uint8_t* snake_head){
	return (apple_loc[0] == snake_head[0] && apple_loc[1] == snake_head[1]);
}

bool snakeOverlap(const uint8_t* length){
	// checking if collision has taken place or not
	// We are going to sum up all the ones in the grid and compare it to snake length + 1(apple)
	uint8_t total_ones = 0;
	for (int i = 0; i < 8; i++)
		for (int j = 0; j < 8; j++) total_ones += ((grid[i] >> j) & 0x01);
	return total_ones != *(length) + 1;
}

void displaySnake(){
    initSPI();
    initMAX7219();
    clear(); // prevent weird glitch when initializing the sensor

    //generating random row and column to spawn snake in
    uint8_t length = 3; // we need to track of this manually as sizeof(dynamic array) never works, because to sizeof it would always give uint8_t type...
    uint8_t snake_position[64*2] = {0}; // as it will have x and y positions and we are testing with a snake of length 3
    uint8_t head_row = (uint8_t) esp_random()%8;
    uint8_t head_col = (uint8_t) esp_random()%8;
	uint8_t apple_loc[2];
	setApple(apple_loc);

    // snakes attributes
    snake_position[0] = head_row; // snake head row
    snake_position[1] = head_col; // snake head column
								  //
	const uint8_t *snake_head[2] = {&head_row, &head_col}; // as head always constant, on ly the value in the memory is changing

    for (int i = 1; i < length; i++){
        snake_position[2*i] = (head_row - i)%8;
        snake_position[2*i + 1] = head_col;
    }

	// updating snake's position
    while (1){
        clear();
        

        // need to do seperately for - as if gets a -ve value after mod, but after storing it becomes unsigned so mod now applicable
		switch(dir){
			case UP:
				head_row = (head_row + 1)%8;
				break;
			case DOWN:
				head_row = (head_row + 7)%8;
				break;
			case LEFT:
				head_col = (head_col + 1)%8;
				break;
			case RIGHT:
				head_col = (head_col + 7)%8;
				break;
        }

        // shifting everything in the array towards the right and discarding the last element
		uint8_t snake_tail[2] = {snake_position[2*(length - 1)], snake_position[2*(length - 1) + 1]};

        // shifting everything in the array towards the right and discarding the last element
        for(int i = length - 1; i > 0; i--){
            snake_position[2*i] = snake_position[2*i - 2];
            snake_position[2*i + 1] = snake_position[2*i - 1];
        }

        snake_position[0] = head_row;
        snake_position[1] = head_col;

		if (appleEaten(apple_loc, *snake_head)){
			snake_position[2*length] = snake_tail[0];
			snake_position[2*length + 1] = snake_tail[1];
			length++;
		}

        rcToLED(snake_position, length);
		// updating apple here so that apple doesn't spawn on new tail
		if (appleEaten(apple_loc, *snake_head)) setApple(apple_loc);
		rcToLED(apple_loc, 1);
		if (snakeOverlap(&length)){
            printf("Game Over\n");
            esp_restart();
        }
		displayGrid();
		vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main(void)
{
    init_uart();

    TaskHandle_t display_handle = NULL;
    uint8_t spi_data[2] = {0x01, 0x08};

    xTaskCreate(displaySnake, "snake", 4096, (void *) spi_data, 2, &display_handle);

    while(1){
        // for now reading from idf monitor
		int rxBytes = uart_read_bytes(UART_NUM, &data, uart_buffer_size, pdMS_TO_TICKS(500));
		if (rxBytes > 0){
			switch (data){
				case 'w':
					dir = UP;
					break;
				case 'a':
					dir = LEFT;
					break;
				case 's':
					dir = DOWN;
					break;
				case 'd':
					dir = RIGHT;
					break;
			}
		}
		vTaskDelay(pdMS_TO_TICKS(500));
    }
}
