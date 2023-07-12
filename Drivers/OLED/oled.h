#ifndef OLED_DRIVER_H
#define OLED_DRIVER_H

#include "arm_math_types.h"

void start_LED_screen(void);
void turnoff_LED_screen(void);
void draw_Rectangle(uint8_t start_column, uint8_t start_row,
					uint8_t end_column, uint8_t end_row);
void clear_LED_screen(void);

#endif