#ifndef __TEST_SSD1306_H_
#define __TEST_SSD1306_H_

#include "main.h"
#include "ssd1306.h"

void TestLines(I2C_HandleTypeDef *hi2c, SSD1306_COLOR_t color);
void TestRectangles(I2C_HandleTypeDef *hi2c, SSD1306_COLOR_t color);
void TestFilledRectangles(I2C_HandleTypeDef *hi2c, SSD1306_COLOR_t color);
void TestCircles(I2C_HandleTypeDef *hi2c, uint8_t radius, SSD1306_COLOR_t color);
void TestFilledCircles(I2C_HandleTypeDef *hi2c, uint8_t radius, SSD1306_COLOR_t color);
void TestTriangles(I2C_HandleTypeDef *hi2c, SSD1306_COLOR_t color);
void TestDynamicBackground(I2C_HandleTypeDef *hi2c, SSD1306_COLOR_t color);

#endif /* __TEST_SSD1306_H_ */
