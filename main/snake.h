#pragma once
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "esp_system.h"

#define REFRESH_RATE 1 // Hz

void move_snake(char c);
void display_game();
