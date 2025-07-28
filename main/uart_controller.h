#pragma once
#include <stdlib.h>

#include "driver/uart.h"
#include "sdkconfig.h"

#define UART_NUM UART_NUM_0
#define UART_RX 3
#define UART_TX 1
#define CTS -1
#define RTS -1
#define EVENT_QUEUE 2

#define INPUT_RATE 2 // Hz

extern char rx_buffer;

void init_uart();
char rx_read();
