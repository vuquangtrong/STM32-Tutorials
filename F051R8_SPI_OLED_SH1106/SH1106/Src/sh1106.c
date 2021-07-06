#include "SH1106.h"

#define ABS(x)   ((x) > 0 ? (x) : -(x))

SH1106_t SH1106 = {
  .MaskX1 = 0,
  .MaskY1 = 0,
  .MaskX2 = SH1106_WIDTH,
  .MaskY2 = SH1106_HEIGHT,
  .Inverted = 0,
  // .Dirty = 0,
  .Ticks = 0,
  .Frames = 0,
  .FPS = 0
};

SH1106_Buffer_t SH1106_buffer = {
  .CmdBufferIndex = 0,
  .CmdBuffer = {0},
  .DataBuffer = {0}
};

void SH1106_CMD_Int() {
  SH1106_buffer.CmdBufferIndex = 1;
}

void SH1106_CMD_Add0(uint8_t command) {
  SH1106_buffer.CmdBuffer[SH1106_buffer.CmdBufferIndex++] = command;
}

void SH1106_CMD_Add1(uint8_t command, uint8_t data) {
  SH1106_buffer.CmdBuffer[SH1106_buffer.CmdBufferIndex++] = command;
  SH1106_buffer.CmdBuffer[SH1106_buffer.CmdBufferIndex++] = data;
}

void SH1106_CMD_Add2(uint8_t command, uint8_t data1, uint8_t data2) {
  SH1106_buffer.CmdBuffer[SH1106_buffer.CmdBufferIndex++] = command;
  SH1106_buffer.CmdBuffer[SH1106_buffer.CmdBufferIndex++] = data1;
  SH1106_buffer.CmdBuffer[SH1106_buffer.CmdBufferIndex++] = data2;
}

void SH1106_CMD_Send(SPI_HandleTypeDef* hspi) {
  HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(hspi,
                          SH1106_buffer.CmdBuffer,
                          SH1106_buffer.CmdBufferIndex,
                          HAL_MAX_DELAY
                          );
}

void SH1106_CMD_Send_IT(SPI_HandleTypeDef* hspi) {
  HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit_IT(hspi,
                          SH1106_buffer.CmdBuffer,
                          SH1106_buffer.CmdBufferIndex
                          );
}

void SH1106_DATA_Send(SPI_HandleTypeDef* hspi) {
  HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin, GPIO_PIN_SET);
  HAL_SPI_Transmit(hspi,
                          SH1106_buffer.DataBuffer,
                          SH1106_BUFFER_DATA_MAX,
                          HAL_MAX_DELAY
                          );
}

void SH1106_DATA_Send_IT(SPI_HandleTypeDef* hspi) {
  HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin, GPIO_PIN_SET);
  HAL_SPI_Transmit_IT(hspi,
                          SH1106_buffer.DataBuffer,
                          SH1106_BUFFER_DATA_MAX
                          );
}

void SH1106_DATA_Send_DMA(SPI_HandleTypeDef* hspi) {
  HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin, GPIO_PIN_SET);
  HAL_SPI_Transmit_DMA(hspi,
                          SH1106_buffer.DataBuffer,
                          SH1106_BUFFER_DATA_MAX
                          );
}

void SH1106_Init(SPI_HandleTypeDef* hspi) {
  HAL_GPIO_WritePin(OLED_RST_GPIO_Port, OLED_RST_Pin, GPIO_PIN_RESET);
  HAL_Delay(10);
  HAL_GPIO_WritePin(OLED_RST_GPIO_Port, OLED_RST_Pin, GPIO_PIN_SET);
  // startup sequence
  SH1106_CMD_Int();
  SH1106_CMD_Add0(0xAE);                     // Display off
//  SH1106_CMD_Add1(0xD5, 0x81);               // OSC default=0x81
  SH1106_CMD_Add1(0x81, 0x7F);               // Brightness in range 0~255, default=0x7F
  SH1106_CMD_Add1(0x20, 0);                  // Memory Address Mode: Horizontal=0, Vertical=1, Page=default=2
  SH1106_CMD_Add1(0xD3, 0);                  // Set Display Offset in range 0~63
  SH1106_CMD_Add0(0x40);                     // Set Display start line in range 0x40~0x7F
  SH1106_CMD_Add1(0xA8, SH1106_HEIGHT-1);    // Set multiplex number (activated rows): rows=height-1, default=63
//  SH1106_CMD_Add1(0xDA, 0x02);               // Reduce a half of height
  SH1106_CMD_Add0(0xA0);                     // Segment (Column) normal mode, Inverse=0xA1
  SH1106_CMD_Add0(0xC0);                     // Common (Row) normal mode, inverse=0xC8
  SH1106_CMD_Add0(0xA6);                     // Display normal mode, inverse=0xA7
  SH1106_CMD_Add0(0x2E);                     // Disable Scroll
  SH1106_CMD_Add1(0xD9, 2);                  // Pre-charge period, default=2
  SH1106_CMD_Add1(0x8D, 0x14);               // Charge Pump: On=0x14, Off=0x10
  SH1106_CMD_Add0(0xAF);                     // Display on
  SH1106_CMD_Send(hspi);

  // clear all GDDRAM
  SH1106_Clear();

  SH1106_CMD_Int();
  SH1106_CMD_Add2(0x21,0,127);
  SH1106_CMD_Add2(0x22,0,3); // first half 128x32
  SH1106_CMD_Send(hspi);
  SH1106_DATA_Send(hspi);

  SH1106_CMD_Int();
  SH1106_CMD_Add2(0x21,0,127);
  SH1106_CMD_Add2(0x22,4,7); // second half 128x32
  SH1106_CMD_Send(hspi);
  SH1106_DATA_Send(hspi);
}

void SH1106_Screen_Update(SPI_HandleTypeDef* hspi) {
//  if (SH1106.Dirty) {
    SH1106_CMD_Int();
    SH1106_CMD_Add2(0x21, 0, SH1106_WIDTH-1);       // Segment (column) start at 0 to 127
    SH1106_CMD_Add2(0x22, 0, (SH1106_HEIGHT/8)-1);  // Page (rows) start at 0 to 3
    SH1106_CMD_Send(hspi);
    SH1106_DATA_Send(hspi);
//    SH1106.Dirty = 0;
    SH1106.Frames++;
//  }
}

void SH1106_Screen_Update_IT(SPI_HandleTypeDef* hspi) {
//  if (SH1106.Dirty) {
    SH1106_CMD_Int();
    SH1106_CMD_Add2(0x21, 0, SH1106_WIDTH-1);       // Segment (column) start at 0 to Width
    SH1106_CMD_Add2(0x22, 0, (SH1106_HEIGHT/8)-1);  // Page (rows) start at 0 to Height
    SH1106_CMD_Send(hspi); // Wait for it, or call sending data later
    SH1106_DATA_Send_IT(hspi);
//    SH1106.Dirty = 0;
    SH1106.Frames++;
//  }
}

void SH1106_Screen_Update_DMA(SPI_HandleTypeDef* hspi) {
//  if (SH1106.Dirty) {
    SH1106_CMD_Int();
    SH1106_CMD_Add2(0x21, 0, SH1106_WIDTH-1);       // Segment (column) start at 0 to Width
    SH1106_CMD_Add2(0x22, 0, (SH1106_HEIGHT/8)-1);  // Page (rows) start at 0 to Height
    SH1106_CMD_Send(hspi); // Wait for it, or call sending data later
    SH1106_DATA_Send_DMA(hspi);
//    SH1106.Dirty = 0;
    SH1106.Frames++;
//  }
}

void SH1106_Fill(SH1106_COLOR_t color) {
  memset(SH1106_buffer.DataBuffer,
         (color == SH1106_BLACK) ? 0x00 : 0xFF,
         SH1106_BUFFER_DATA_MAX);
}

void SH1106_Clear() {
  SH1106_Fill(SH1106_BLACK);
}

void SH1106_InvertColors(void)
{
    SH1106.Inverted = !SH1106.Inverted;
}

void SH1106_SetMask(int16_t x1, int16_t y1, int16_t x2, int16_t y2) {
  SH1106.MaskX1 = (x1>=0 && x1<SH1106_WIDTH) ? x1 : 0;
  SH1106.MaskY1 = (y1>=0 && y1<SH1106_HEIGHT) ? y1 : 0;
  SH1106.MaskX2 = (x2>=0 && x2<SH1106_WIDTH) ? x2 : SH1106_WIDTH-1;
  SH1106.MaskY2 = (y2>=0 && y2<SH1106_HEIGHT) ? y2 : SH1106_HEIGHT-1;
}

void SH1106_DrawPixel(int16_t x, int16_t y, SH1106_COLOR_t color) {
  if (x < SH1106.MaskX1 ||
      y < SH1106.MaskY1 ||
      x >= SH1106.MaskX2 ||
      y >= SH1106.MaskY2) {
    /* Error */
    return;
  }

  if (SH1106.Inverted) {
    color = (SH1106_COLOR_t)!color;
  }

  if(color == SH1106_WHITE) {
    SH1106_buffer.DataBuffer[x + (y >> 3) * SH1106_WIDTH] |= (1 << (y % 8));
  } else {
    SH1106_buffer.DataBuffer[x + (y >> 3) * SH1106_WIDTH] &= ~(1 << (y % 8));
  }

//  SH1106.Dirty = 1;
}

void SH1106_WriteChar(int16_t x, int16_t y, char ch, FontDef_t* Font, SH1106_COLOR_t color, SH1106_DRAW_t mode)
{
    int16_t x0, y0, b;

    // Translate font to screen buffer
    for (y0 = 0; y0 < Font->height; y0++)
    {
        b = Font->data[(ch - 32) * Font->height + y0];
        for (x0 = 0; x0 < Font->width; x0++)
        {
            if ((b << x0) & 0x8000)
            {
              SH1106_DrawPixel(x + x0, y + y0, (SH1106_COLOR_t) color);
            }
            else if (mode == SH1106_OVERRIDE)
            {
              SH1106_DrawPixel(x + x0, y + y0, (SH1106_COLOR_t)!color);
            }
        }
    }
}

void SH1106_WriteString(int16_t x, int16_t y, char* str, FontDef_t* Font, SH1106_COLOR_t color, SH1106_DRAW_t mode)
{
    int16_t l = strlen(str);
    if (
        (x + l*Font->width < SH1106.MaskX1) ||
        (SH1106.MaskX2 < x) ||
        (y + Font->height < SH1106.MaskY1) ||
        (SH1106.MaskY2 < y)
    ){
      return;
    }

    int16_t fx = (SH1106.MaskX1 - x) / Font->width;
    int16_t rx = (x - SH1106.MaskX2) / Font->width;
    char* estr = str + l;
    int16_t n = 0;


    // cut off characters which are out of masking box
    if (fx > 0) {
        str += fx;
        x += fx*Font->width;
    }

    if (rx > 0) {
      estr -= rx;
    }



    // Write until null-byte or the first cutoff char
    while (*str && str < estr)
    {
        SH1106_WriteChar(x + n*Font->width, y, *str, Font, color, mode);
        n++;
        str++;
    }
}


void SH1106_DrawBitmap(int16_t x, int16_t y, const uint8_t* bitmap, int16_t w, int16_t h, SH1106_COLOR_t color, SH1106_DRAW_t mode)
{

    int16_t byteWidth = (w + 7) / 8; // Bitmap scan line pad = whole byte
    uint8_t byte = 0;

    for(int16_t j=0; j<h; j++, y++)
    {
        for(int16_t i=0; i<w; i++)
        {
            if(i & 7)
            {
               byte <<= 1;
            }
            else
            {
               byte = (*(const unsigned char *)(&bitmap[j * byteWidth + i / 8]));
            }

            if(byte & 0x80) {
              SH1106_DrawPixel(x+i, y, color);
            }
            else if (mode == SH1106_OVERRIDE)
            {
              SH1106_DrawPixel(x+i, y, (SH1106_COLOR_t)!color);
            }
        }
    }
}

void SH1106_DrawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, SH1106_COLOR_t color) {
  int16_t dx, dy, sx, sy, err, e2, i, tmp;

  /* Check for overflow */
  if (x0 >= SH1106_WIDTH) {
    x0 = SH1106_WIDTH - 1;
  }
  if (x1 >= SH1106_WIDTH) {
    x1 = SH1106_WIDTH - 1;
  }
  if (y0 >= SH1106_HEIGHT) {
    y0 = SH1106_HEIGHT - 1;
  }
  if (y1 >= SH1106_HEIGHT) {
    y1 = SH1106_HEIGHT - 1;
  }

  dx = (x0 < x1) ? (x1 - x0) : (x0 - x1);
  dy = (y0 < y1) ? (y1 - y0) : (y0 - y1);
  sx = (x0 < x1) ? 1 : -1;
  sy = (y0 < y1) ? 1 : -1;
  err = ((dx > dy) ? dx : -dy) / 2;

  if (dx == 0) {
    if (y1 < y0) {
      tmp = y1;
      y1 = y0;
      y0 = tmp;
    }

    if (x1 < x0) {
      tmp = x1;
      x1 = x0;
      x0 = tmp;
    }

    /* Vertical line */
    for (i = y0; i <= y1; i++) {
      SH1106_DrawPixel(x0, i, color);
    }

    /* Return from function */
    return;
  }

  if (dy == 0) {
    if (y1 < y0) {
      tmp = y1;
      y1 = y0;
      y0 = tmp;
    }

    if (x1 < x0) {
      tmp = x1;
      x1 = x0;
      x0 = tmp;
    }

    /* Horizontal line */
    for (i = x0; i <= x1; i++) {
      SH1106_DrawPixel(i, y0, color);
    }

    /* Return from function */
    return;
  }

  while (1) {
    SH1106_DrawPixel(x0, y0, color);
    if (x0 == x1 && y0 == y1) {
      break;
    }
    e2 = err;
    if (e2 > -dx) {
      err -= dy;
      x0 += sx;
    }
    if (e2 < dy) {
      err += dx;
      y0 += sy;
    }
  }
}

void SH1106_DrawRectangle(int16_t x, int16_t y, int16_t w, int16_t h, SH1106_COLOR_t color) {
  /* Check input parameters */
  if (
    x >= SH1106_WIDTH ||
    y >= SH1106_HEIGHT
  ) {
    /* Return error */
    return;
  }

  /* Check width and height */
  if ((x + w) >= SH1106_WIDTH) {
    w = SH1106_WIDTH - x;
  }
  if ((y + h) >= SH1106_HEIGHT) {
    h = SH1106_HEIGHT - y;
  }

  /* Draw 4 lines */
  SH1106_DrawLine(x, y, x + w, y, color);         /* Top line */
  SH1106_DrawLine(x, y + h, x + w, y + h, color); /* Bottom line */
  SH1106_DrawLine(x, y, x, y + h, color);         /* Left line */
  SH1106_DrawLine(x + w, y, x + w, y + h, color); /* Right line */
}

void SH1106_DrawFilledRectangle(int16_t x, int16_t y, int16_t w, int16_t h, SH1106_COLOR_t color) {
  uint8_t i;

  /* Check input parameters */
  if (
    x >= SH1106_WIDTH ||
    y >= SH1106_HEIGHT
  ) {
    /* Return error */
    return;
  }

  /* Check width and height */
  if ((x + w) >= SH1106_WIDTH) {
    w = SH1106_WIDTH - x;
  }
  if ((y + h) >= SH1106_HEIGHT) {
    h = SH1106_HEIGHT - y;
  }

  /* Draw lines */
  for (i = 0; i <= h; i++) {
    /* Draw lines */
    SH1106_DrawLine(x, y + i, x + w, y + i, color);
  }
}

void SH1106_DrawTriangle(int16_t x1, int16_t y1, int16_t x2, int16_t y2, int16_t x3, int16_t y3, SH1106_COLOR_t color) {
  /* Draw lines */
  SH1106_DrawLine(x1, y1, x2, y2, color);
  SH1106_DrawLine(x2, y2, x3, y3, color);
  SH1106_DrawLine(x3, y3, x1, y1, color);
}


void SH1106_DrawFilledTriangle(int16_t x1, int16_t y1, int16_t x2, int16_t y2, int16_t x3, int16_t y3, SH1106_COLOR_t color) {
  int16_t deltax = 0, deltay = 0, x = 0, y = 0, xinc1 = 0, xinc2 = 0,
  yinc1 = 0, yinc2 = 0, den = 0, num = 0, numadd = 0, numpixels = 0,
  curpixel = 0;

  deltax = ABS(x2 - x1);
  deltay = ABS(y2 - y1);
  x = x1;
  y = y1;

  if (x2 >= x1) {
    xinc1 = 1;
    xinc2 = 1;
  } else {
    xinc1 = -1;
    xinc2 = -1;
  }

  if (y2 >= y1) {
    yinc1 = 1;
    yinc2 = 1;
  } else {
    yinc1 = -1;
    yinc2 = -1;
  }

  if (deltax >= deltay){
    xinc1 = 0;
    yinc2 = 0;
    den = deltax;
    num = deltax / 2;
    numadd = deltay;
    numpixels = deltax;
  } else {
    xinc2 = 0;
    yinc1 = 0;
    den = deltay;
    num = deltay / 2;
    numadd = deltax;
    numpixels = deltay;
  }

  for (curpixel = 0; curpixel <= numpixels; curpixel++) {
    SH1106_DrawLine(x, y, x3, y3, color);

    num += numadd;
    if (num >= den) {
      num -= den;
      x += xinc1;
      y += yinc1;
    }
    x += xinc2;
    y += yinc2;
  }
}

void SH1106_DrawCircle(int16_t x0, int16_t y0, int16_t r, SH1106_COLOR_t color) {
  int16_t f = 1 - r;
  int16_t ddF_x = 1;
  int16_t ddF_y = -2 * r;
  int16_t x = 0;
  int16_t y = r;

    SH1106_DrawPixel(x0, y0 + r, color);
    SH1106_DrawPixel(x0, y0 - r, color);
    SH1106_DrawPixel(x0 + r, y0, color);
    SH1106_DrawPixel(x0 - r, y0, color);

    while (x < y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;

        SH1106_DrawPixel(x0 + x, y0 + y, color);
        SH1106_DrawPixel(x0 - x, y0 + y, color);
        SH1106_DrawPixel(x0 + x, y0 - y, color);
        SH1106_DrawPixel(x0 - x, y0 - y, color);

        SH1106_DrawPixel(x0 + y, y0 + x, color);
        SH1106_DrawPixel(x0 - y, y0 + x, color);
        SH1106_DrawPixel(x0 + y, y0 - x, color);
        SH1106_DrawPixel(x0 - y, y0 - x, color);
    }
}

void SH1106_DrawFilledCircle(int16_t x0, int16_t y0, int16_t r, SH1106_COLOR_t color) {
  int16_t f = 1 - r;
  int16_t ddF_x = 1;
  int16_t ddF_y = -2 * r;
  int16_t x = 0;
  int16_t y = r;

    SH1106_DrawPixel(x0, y0 + r, color);
    SH1106_DrawPixel(x0, y0 - r, color);
    SH1106_DrawPixel(x0 + r, y0, color);
    SH1106_DrawPixel(x0 - r, y0, color);
    SH1106_DrawLine(x0 - r, y0, x0 + r, y0, color);

    while (x < y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;

        SH1106_DrawLine(x0 - x, y0 + y, x0 + x, y0 + y, color);
        SH1106_DrawLine(x0 + x, y0 - y, x0 - x, y0 - y, color);

        SH1106_DrawLine(x0 + y, y0 + x, x0 - y, y0 + x, color);
        SH1106_DrawLine(x0 + y, y0 - x, x0 - y, y0 - x, color);
    }
}

void SH1106_1sec_Tick() {
  SH1106.Ticks++;
  if(SH1106.Ticks >= 1000) {
    SH1106.FPS = SH1106.Frames;
    SH1106.Frames = 0;
    SH1106.Ticks = 0;
  }
}
