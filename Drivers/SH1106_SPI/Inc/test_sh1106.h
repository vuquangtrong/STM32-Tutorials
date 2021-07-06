#ifndef __TEST_SH1106_H_
#define __TEST_SH1106_H_

#include "main.h"
#include "SH1106.h"

void TestLines(SPI_HandleTypeDef* hspi, SH1106_COLOR_t color);
void TestRectangles(SPI_HandleTypeDef* hspi, SH1106_COLOR_t color);
void TestFilledRectangles(SPI_HandleTypeDef* hspi, SH1106_COLOR_t color);
void TestCircles(SPI_HandleTypeDef* hspi, uint8_t radius, SH1106_COLOR_t color);
void TestFilledCircles(SPI_HandleTypeDef* hspi, uint8_t radius, SH1106_COLOR_t color);
void TestTriangles(SPI_HandleTypeDef* hspi, SH1106_COLOR_t color);
void TestDynamicBackground(SPI_HandleTypeDef* hspi, SH1106_COLOR_t color);

#endif /* __TEST_SH1106_H_ */
