#include "uart_controller.h"

char rx_buffer = 'w';
static int uart_buffer_size = 1024;

void init_uart(){
    QueueHandle_t uart_queue;
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, uart_buffer_size, uart_buffer_size, EVENT_QUEUE, &uart_queue, 0));

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

char rx_read(){
	int rxBytes = uart_read_bytes(UART_NUM, &rx_buffer, uart_buffer_size, pdMS_TO_TICKS(1000/INPUT_RATE));
	return rxBytes;
}
