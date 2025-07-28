#pragma once
#include <stdlib.h>

#include "sdkconfig.h"

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

extern uint8_t grid[8];

void init_spi();
void init_max7219();
void send_data(uint8_t addr, uint8_t data);
void display_grid();
void clear_grid();
void rc_to_led(const uint8_t *arr, const uint8_t length);
