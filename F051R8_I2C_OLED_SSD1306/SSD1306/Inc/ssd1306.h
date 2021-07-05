/**
  ******************************************************************************
  * @file           : SSD1306.h
  * @brief          : Header for SSD1306 Driver
  ******************************************************************************
  */

#ifndef __SSD1306_H
#define __SSD1306_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <string.h>
#include "main.h"
#include "fonts.h"

#define SSD1306_ADDR      0x78 // 0x3C << 1
#define SSD1306_WIDTH     128
#define SSD1306_HEIGHT    32

#define SSD1306_BUFFER_CMD_MAX    64
#define SSD1306_BUFFER_DATA_MAX   (SSD1306_WIDTH * SSD1306_HEIGHT / 8)

#define SSD1306_CONTROL_CMD_SINGLE    0x80
#define SSD1306_CONTROL_CMD_STREAM    0x00
#define SSD1306_CONTROL_DATA_SINGLE   0xC0
#define SSD1306_CONTROL_DATA_STREAM   0x40

typedef struct {
  int16_t MaskX1;
  int16_t MaskY1;
  int16_t MaskX2;
  int16_t MaskY2;
  uint8_t Inverted;
//  uint8_t Dirty;
  int16_t Ticks;
  uint8_t Frames;
  uint8_t FPS;
} SSD1306_t;

typedef struct {
  uint8_t CmdBufferIndex;
  uint8_t CmdBuffer[SSD1306_BUFFER_CMD_MAX];
  uint8_t DataBuffer[SSD1306_BUFFER_DATA_MAX+1];
} SSD1306_Buffer_t;

typedef enum {
  SSD1306_BLACK = 0x00,
  SSD1306_WHITE = 0x01
} SSD1306_COLOR_t;

typedef enum {
  SSD1306_OVERRIDE = 0x00,
  SSD1306_TRANSPARENT = 0x01
} SSD1306_DRAW_t;

void SSD1306_CMD_Add0(uint8_t command);
void SSD1306_CMD_Add1(uint8_t command, uint8_t data);
void SSD1306_CMD_Add2(uint8_t command, uint8_t data1,  uint8_t data2);
void SSD1306_CMD_Send(I2C_HandleTypeDef* hi2c);
void SSD1306_CMD_Send_IT(I2C_HandleTypeDef* hi2c);
void SSD1306_CMD_Send_DMA(I2C_HandleTypeDef* hi2c);
void SSD1306_DATA_Send(I2C_HandleTypeDef* hi2c);
void SSD1306_DATA_Send_IT(I2C_HandleTypeDef* hi2c);
void SSD1306_DATA_Send_DMA(I2C_HandleTypeDef* hi2c);
void SSD1306_Init(I2C_HandleTypeDef* hi2c);
void SSD1306_Screen_Update(I2C_HandleTypeDef* hi2c);
void SSD1306_Screen_Update_IT(I2C_HandleTypeDef* hi2c);
void SSD1306_Screen_Update_DMA(I2C_HandleTypeDef* hi2c);
void SSD1306_Fill(SSD1306_COLOR_t color);
void SSD1306_Clear();
void SSD1306_InvertColors();
void SSD1306_SetMask(int16_t x1, int16_t y1, int16_t x2, int16_t y2);
void SSD1306_DrawPixel(int16_t x, int16_t y, SSD1306_COLOR_t color);
void SSD1306_WriteChar(int16_t x, int16_t y, char ch, FontDef_t* Font, SSD1306_COLOR_t color, SSD1306_DRAW_t mode);
void SSD1306_WriteString(int16_t x, int16_t y, char* str, FontDef_t* Font, SSD1306_COLOR_t color, SSD1306_DRAW_t mode);
void SSD1306_DrawBitmap(int16_t x, int16_t y, const uint8_t* bitmap, int16_t w, int16_t h, SSD1306_COLOR_t color, SSD1306_DRAW_t mode);
void SSD1306_DrawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, SSD1306_COLOR_t color);
void SSD1306_DrawRectangle(int16_t x, int16_t y, int16_t w, int16_t h, SSD1306_COLOR_t color);
void SSD1306_DrawFilledRectangle(int16_t x, int16_t y, int16_t w, int16_t h, SSD1306_COLOR_t color);
void SSD1306_DrawTriangle(int16_t x1, int16_t y1, int16_t x2, int16_t y2, int16_t x3, int16_t y3, SSD1306_COLOR_t color);
void SSD1306_DrawFilledTriangle(int16_t x1, int16_t y1, int16_t x2, int16_t y2, int16_t x3, int16_t y3, SSD1306_COLOR_t color);
void SSD1306_DrawCircle(int16_t x0, int16_t y0, int16_t r, SSD1306_COLOR_t color);
void SSD1306_DrawFilledCircle(int16_t x0, int16_t y0, int16_t r, SSD1306_COLOR_t color);
void SSD1306_1sec_Tick();

//
//  Export members
//
extern SSD1306_t SSD1306;

#ifdef __cplusplus
}
#endif

#endif /* __SSD1306_H */
