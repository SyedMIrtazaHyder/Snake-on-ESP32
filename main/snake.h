#pragma once
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "esp_system.h"

#define REFRESH_RATE 1 // Hz

void moveSnake(char c);
void displayGame();
