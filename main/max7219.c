#include "max7219.h"

#include "driver/spi_master.h"

uint8_t grid[8] = {0};
static spi_device_handle_t dot_matrix = NULL;

void initSPI(){
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

void initMAX7219(){
    sendData(DISPLAY_TEST, 0); // by default it is 1 idk why
    sendData(SCAN_LIMIT, 0x07); // so all 8 rows display
    sendData(DECODE_MODE, 0x00); // set to 1 when using BCD convertor
    sendData(INTENSITY, 0x07); // 0.5 duty cycle
    sendData(SHUTDOWN, 1);
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

void displayGrid(){
    for (uint8_t i = 1; i < 9; i++) sendData(i, grid[i-1]);
}

void clearGrid(){
    for (uint8_t i = 0; i < 8; i++) grid[i] = 0;
}

void rcToLED(const uint8_t *arr, const uint8_t length){
    uint8_t row, col;
    for (int i = 0; i < length; i++){
		row = arr[2*i];
		col = arr[2*i + 1];
		grid[row] = grid[row] | (1 << col);
    }
}

