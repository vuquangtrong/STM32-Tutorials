#include "test_ssd1306.h"

#define min(a, b)(((a) <(b)) ?(a) :(b))

void TestLines(I2C_HandleTypeDef *hi2c, uint8_t color) {
  uint8_t x1, y1, x2, y2;

  SSD1306_Clear();

  x1 = y1 = 0;
  y2 = SSD1306_HEIGHT - 1;
  for (x2 = 0; x2 < SSD1306_WIDTH; x2 += 6) {
    SSD1306_DrawLine(x1, y1, x2, y2, color);
    HAL_Delay(1);
    SSD1306_Screen_Update(hi2c);
  }
  x2 = SSD1306_WIDTH - 1;
  for (y2 = 0; y2 < SSD1306_HEIGHT; y2 += 6) {
    SSD1306_DrawLine(x1, y1, x2, y2, color);
    HAL_Delay(1);
    SSD1306_Screen_Update(hi2c);
  }

  HAL_Delay(10);

  SSD1306_Clear();

  x1 = SSD1306_WIDTH - 1;
  y1 = 0;
  y2 = SSD1306_HEIGHT - 1;
  for (x2 = 0; x2 < SSD1306_WIDTH; x2 += 6) {
    SSD1306_DrawLine(x1, y1, x2, y2, color);
    SSD1306_Screen_Update(hi2c);
    HAL_Delay(1);
  }
  x2 = 0;
  for (y2 = 0; y2 < SSD1306_HEIGHT; y2 += 6) {
    SSD1306_DrawLine(x1, y1, x2, y2, color);
    SSD1306_Screen_Update(hi2c);
    HAL_Delay(1);
  }

  //HAL_Delay(10);

  /*
  SSD1306_Clear();

  x1 = 0;
  y1 = SSD1306_HEIGHT - 1;
  y2 = 0;
  for (x2 = 0; x2 < SSD1306_WIDTH; x2 += 6) {
    SSD1306_DrawLine(x1, y1, x2, y2, color);
    HAL_Delay(1);
    SSD1306_Screen_Update(hi2c);
  }
  x2 = SSD1306_WIDTH - 1;
  for (y2 = 0; y2 < SSD1306_HEIGHT; y2 += 6) {
    SSD1306_DrawLine(x1, y1, x2, y2, color);
    HAL_Delay(1);
    SSD1306_Screen_Update(hi2c);
  }

  HAL_Delay(10);

  SSD1306_Clear();

  x1 = SSD1306_WIDTH - 1;
  y1 = SSD1306_HEIGHT - 1;
  y2 = 0;
  for (x2 = 0; x2 < SSD1306_WIDTH; x2 += 6) {
    SSD1306_DrawLine(x1, y1, x2, y2, color);
    HAL_Delay(1);
    SSD1306_Screen_Update(hi2c);
  }
  x2 = 0;
  for (y2 = 0; y2 < SSD1306_HEIGHT; y2 += 6) {
    SSD1306_DrawLine(x1, y1, x2, y2, color);
    HAL_Delay(1);
    SSD1306_Screen_Update(hi2c);
  }
  */
}

void TestRectangles(I2C_HandleTypeDef *hi2c, uint8_t color) {
  uint8_t n, i, i2;

  SSD1306_Clear();

  n = min(SSD1306_WIDTH, SSD1306_HEIGHT);

  for (i = 2; i < n; i += 4) {
    i2 = i / 2;
    SSD1306_DrawRectangle((SSD1306_WIDTH / 2) - i2, (SSD1306_HEIGHT / 2) - i2,
        i, i, color);
    SSD1306_Screen_Update(hi2c);
    HAL_Delay(100);
  }
}

void TestFilledRectangles(I2C_HandleTypeDef *hi2c, uint8_t color) {
  uint8_t n, i, i2, cx = SSD1306_WIDTH / 2 - 1, cy = SSD1306_HEIGHT / 2 - 1;

  SSD1306_Clear();

  n = min(SSD1306_WIDTH, SSD1306_HEIGHT);
  for (i = n; i > 0; i -= 4) {
    i2 = i / 2;
    SSD1306_DrawFilledRectangle(cx - i2, cy - i2, i, i, color);
    color = !color;
    SSD1306_Screen_Update(hi2c);
    HAL_Delay(100);
  }
}

void TestCircles(I2C_HandleTypeDef *hi2c, uint8_t radius, SSD1306_COLOR_t color) {
  SSD1306_Clear();

  uint8_t x, y, r2 = radius * 2, w = SSD1306_WIDTH + radius, h = SSD1306_HEIGHT + radius;

  for (x = 0; x < w; x += r2) {
    for (y = 0; y < h; y += r2) {
      SSD1306_DrawCircle(x, y, radius, color);
      SSD1306_Screen_Update(hi2c);
      HAL_Delay(10);
    }
  }
}

void TestFilledCircles(I2C_HandleTypeDef *hi2c, uint8_t radius, SSD1306_COLOR_t color) {
  uint8_t x, y, w = SSD1306_WIDTH, h = SSD1306_HEIGHT, r2 = radius * 2;

  SSD1306_Clear();

  for (x = radius; x < w; x += r2) {
    for (y = radius; y < h; y += r2) {
      SSD1306_DrawFilledCircle(x, y, radius, color);
      SSD1306_Screen_Update(hi2c);
      HAL_Delay(10);
    }
  }
}

void TestTriangles(I2C_HandleTypeDef *hi2c, uint8_t color) {
  uint8_t n, i, cx = SSD1306_WIDTH / 2 - 1, cy = SSD1306_HEIGHT / 2 - 1;

  SSD1306_Clear();

  n = min(cx, cy);
  for (i = 0; i < n; i += 4) {
    SSD1306_DrawTriangle(cx, cy - i, cx - i, cy + i, cx + i, cy + i, color);
    SSD1306_Screen_Update(hi2c);
    HAL_Delay(100);
  }
}

void TestDynamicBackground(I2C_HandleTypeDef *hi2c, SSD1306_COLOR_t color) {
  for(uint8_t a = 1; a<SSD1306_HEIGHT; a++) {
    for(uint8_t i=0; i<SSD1306_WIDTH; i++) {
        for(uint8_t j=0; j<SSD1306_HEIGHT; j++) {
            SSD1306_DrawPixel(i, j,(i+j)%a);
        }
    }
    SSD1306_Screen_Update(hi2c);
    HAL_Delay(10);
  }
}
