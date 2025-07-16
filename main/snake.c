#include "snake.h"

#include "esp_random.h"

#include "max7219.h"

static enum Direction {
	UP,
	LEFT,
	DOWN,
	RIGHT} dir;

static void setApple(uint8_t* apple_loc){
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

static bool appleEaten(const uint8_t* apple_loc, const uint8_t* snake_head){
	return (apple_loc[0] == snake_head[0] && apple_loc[1] == snake_head[1]);
}

static bool snakeOverlap(const uint8_t* length){
	// checking if collision has taken place or not
	// We are going to sum up all the ones in the grid and compare it to snake length + 1(apple)
	uint8_t total_ones = 0;
	for (int i = 0; i < 8; i++)
		for (int j = 0; j < 8; j++) total_ones += ((grid[i] >> j) & 0x01);
	return total_ones != *(length) + 1;
}

void moveSnake(char c){
	switch (c){
		case 'w':
			dir = DOWN;
			break;
		case 'a':
			dir = LEFT;
			break;
		case 's':
			dir = UP;
			break;
		case 'd':
			dir = RIGHT;
			break;
	}
}

void displayGame(){
    initSPI();
    initMAX7219();

	// prevent weird glitch when initializing the sensor
    clearGrid(); 
	displayGrid();

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
		clearGrid();

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
            esp_restart(); // resetting esp when lose the game
        }
		displayGrid();
		vTaskDelay(pdMS_TO_TICKS(1000/REFRESH_RATE));
    }
}
