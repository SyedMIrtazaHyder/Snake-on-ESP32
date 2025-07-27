#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include "snake.h"
#include "uart_controller.h"

void app_main(void){
    init_uart();

    TaskHandle_t display_handle = NULL;

    xTaskCreate(displayGame, "game", 2048, NULL, 2, &display_handle);

    while(1){
        // for now reading from idf monitor
		if (rx_read() > 0)
			moveSnake(rx_buffer);
    }
}
