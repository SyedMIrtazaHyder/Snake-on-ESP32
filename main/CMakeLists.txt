idf_component_register(SRCS "snake.c" "max7219.c" "uart_controller.c" "main.c"
					REQUIRES esp_driver_spi esp_driver_uart
                    INCLUDE_DIRS ".")
