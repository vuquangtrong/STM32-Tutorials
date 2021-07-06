/**
  ******************************************************************************
  * @file           : SH1106.h
  * @brief          : Header for SH1106 Driver
  ******************************************************************************
  */

#ifndef __SH1106_H
#define __SH1106_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <string.h>
#include "main.h"
#include "fonts.h"

#define SH1106_WIDTH     128
#define SH1106_HEIGHT    64

#define SH1106_BUFFER_CMD_MAX    64
#define SH1106_BUFFER_DATA_MAX   (SH1106_WIDTH * SH1106_HEIGHT / 8)

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
} SH1106_t;

typedef struct {
  uint8_t CmdBufferIndex;
  uint8_t CmdBuffer[SH1106_BUFFER_CMD_MAX];
  uint8_t DataBuffer[SH1106_BUFFER_DATA_MAX];
} SH1106_Buffer_t;

typedef enum {
  SH1106_BLACK = 0x00,
  SH1106_WHITE = 0x01
} SH1106_COLOR_t;

typedef enum {
  SH1106_OVERRIDE = 0x00,
  SH1106_TRANSPARENT = 0x01
} SH1106_DRAW_t;

void SH1106_CMD_Add0(uint8_t command);
void SH1106_CMD_Add1(uint8_t command, uint8_t data);
void SH1106_CMD_Add2(uint8_t command, uint8_t data1,  uint8_t data2);
void SH1106_CMD_Send(SPI_HandleTypeDef* hspi);
void SH1106_CMD_Send_IT(SPI_HandleTypeDef* hspi);
void SH1106_CMD_Send_DMA(SPI_HandleTypeDef* hspi);
void SH1106_DATA_Send(SPI_HandleTypeDef* hspi);
void SH1106_DATA_Send_IT(SPI_HandleTypeDef* hspi);
void SH1106_DATA_Send_DMA(SPI_HandleTypeDef* hspi);
void SH1106_Init(SPI_HandleTypeDef* hspi);
void SH1106_Screen_Update(SPI_HandleTypeDef* hspi);
void SH1106_Screen_Update_IT(SPI_HandleTypeDef* hspi);
void SH1106_Screen_Update_DMA(SPI_HandleTypeDef* hspi);
void SH1106_Fill(SH1106_COLOR_t color);
void SH1106_Clear();
void SH1106_InvertColors();
void SH1106_SetMask(int16_t x1, int16_t y1, int16_t x2, int16_t y2);
void SH1106_DrawPixel(int16_t x, int16_t y, SH1106_COLOR_t color);
void SH1106_WriteChar(int16_t x, int16_t y, char ch, FontDef_t* Font, SH1106_COLOR_t color, SH1106_DRAW_t mode);
void SH1106_WriteString(int16_t x, int16_t y, char* str, FontDef_t* Font, SH1106_COLOR_t color, SH1106_DRAW_t mode);
void SH1106_DrawBitmap(int16_t x, int16_t y, const uint8_t* bitmap, int16_t w, int16_t h, SH1106_COLOR_t color, SH1106_DRAW_t mode);
void SH1106_DrawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, SH1106_COLOR_t color);
void SH1106_DrawRectangle(int16_t x, int16_t y, int16_t w, int16_t h, SH1106_COLOR_t color);
void SH1106_DrawFilledRectangle(int16_t x, int16_t y, int16_t w, int16_t h, SH1106_COLOR_t color);
void SH1106_DrawTriangle(int16_t x1, int16_t y1, int16_t x2, int16_t y2, int16_t x3, int16_t y3, SH1106_COLOR_t color);
void SH1106_DrawFilledTriangle(int16_t x1, int16_t y1, int16_t x2, int16_t y2, int16_t x3, int16_t y3, SH1106_COLOR_t color);
void SH1106_DrawCircle(int16_t x0, int16_t y0, int16_t r, SH1106_COLOR_t color);
void SH1106_DrawFilledCircle(int16_t x0, int16_t y0, int16_t r, SH1106_COLOR_t color);
void SH1106_1sec_Tick();

//
//  Export members
//
extern SH1106_t SH1106;

#ifdef __cplusplus
}
#endif

#endif /* __SH1106_H */
