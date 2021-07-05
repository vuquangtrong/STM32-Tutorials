#include "SSD1306.h"

#define ABS(x)   ((x) > 0 ? (x) : -(x))

SSD1306_t SSD1306 = {
  .MaskX1 = 0,
  .MaskY1 = 0,
  .MaskX2 = SSD1306_WIDTH,
  .MaskY2 = SSD1306_HEIGHT,
  .Inverted = 0,
  // .Dirty = 0,
  .Ticks = 0,
  .Frames = 0,
  .FPS = 0
};

SSD1306_Buffer_t SSD1306_buffer = {
  .CmdBufferIndex = 0,
  .CmdBuffer = {SSD1306_CONTROL_CMD_STREAM, 0},
  .DataBuffer = {SSD1306_CONTROL_DATA_STREAM, 0}
};

void SSD1306_CMD_Int() {
  SSD1306_buffer.CmdBufferIndex = 1;
}

void SSD1306_CMD_Add0(uint8_t command) {
  SSD1306_buffer.CmdBuffer[SSD1306_buffer.CmdBufferIndex++] = command;
}

void SSD1306_CMD_Add1(uint8_t command, uint8_t data) {
  SSD1306_buffer.CmdBuffer[SSD1306_buffer.CmdBufferIndex++] = command;
  SSD1306_buffer.CmdBuffer[SSD1306_buffer.CmdBufferIndex++] = data;
}

void SSD1306_CMD_Add2(uint8_t command, uint8_t data1, uint8_t data2) {
  SSD1306_buffer.CmdBuffer[SSD1306_buffer.CmdBufferIndex++] = command;
  SSD1306_buffer.CmdBuffer[SSD1306_buffer.CmdBufferIndex++] = data1;
  SSD1306_buffer.CmdBuffer[SSD1306_buffer.CmdBufferIndex++] = data2;
}

void SSD1306_CMD_Send(I2C_HandleTypeDef *hi2c) {
  HAL_I2C_Master_Transmit(hi2c,
                          SSD1306_ADDR,
                          SSD1306_buffer.CmdBuffer,
                          SSD1306_buffer.CmdBufferIndex,
                          HAL_MAX_DELAY
                          );
}

void SSD1306_CMD_Send_IT(I2C_HandleTypeDef *hi2c) {
  HAL_I2C_Master_Transmit_IT(hi2c,
                          SSD1306_ADDR,
                          SSD1306_buffer.CmdBuffer,
                          SSD1306_buffer.CmdBufferIndex
                          );
}

void SSD1306_DATA_Send(I2C_HandleTypeDef *hi2c) {
  HAL_I2C_Master_Transmit(hi2c,
                          SSD1306_ADDR,
                          SSD1306_buffer.DataBuffer,
                          SSD1306_BUFFER_DATA_MAX + 1,
                          HAL_MAX_DELAY
                          );
}

void SSD1306_DATA_Send_IT(I2C_HandleTypeDef *hi2c) {
  HAL_I2C_Master_Transmit_IT(hi2c,
                          SSD1306_ADDR,
                          SSD1306_buffer.DataBuffer,
                          SSD1306_BUFFER_DATA_MAX + 1
                          );
}

void SSD1306_DATA_Send_DMA(I2C_HandleTypeDef *hi2c) {
  HAL_I2C_Master_Transmit_DMA(hi2c,
                          SSD1306_ADDR,
                          SSD1306_buffer.DataBuffer,
                          SSD1306_BUFFER_DATA_MAX + 1
                          );
}

void SSD1306_Init(I2C_HandleTypeDef *hi2c) {
  // startup sequence
  SSD1306_CMD_Int();
  SSD1306_CMD_Add0(0xAE);                     // Display off
  SSD1306_CMD_Add1(0xD5, 0x81);               // OSC default=0x81
  SSD1306_CMD_Add1(0x81, 0x7F);               // Brightness in range 0~255, default=0x7F
  SSD1306_CMD_Add1(0x20, 0);                  // Memory Address Mode: Horizontal=0, Vertical=1, Page=default=2
  SSD1306_CMD_Add1(0xD3, 0);                  // Set Display Offset in range 0~63
  SSD1306_CMD_Add0(0x40);                     // Set Display start line in range 0x40~0x7F
  SSD1306_CMD_Add1(0xA8, SSD1306_HEIGHT-1);   // Set multiplex number (activated rows): rows=height-1, default=63
  SSD1306_CMD_Add1(0xDA, 0x02);               // Reduce a half of height
  SSD1306_CMD_Add0(0xA0);                     // Segment (Column) normal mode, Inverse=0xA1
  SSD1306_CMD_Add0(0xC0);                     // Common (Row) normal mode, inverse=0xC8
  SSD1306_CMD_Add0(0xA6);                     // Display normal mode, inverse=0xA7
  SSD1306_CMD_Add0(0x2E);                     // Disable Scroll
  SSD1306_CMD_Add1(0xD9, 2);                  // Pre-charge period, default=2
  SSD1306_CMD_Add1(0x8D, 0x14);               // Charge Pump: On=0x14, Off=0x10
  SSD1306_CMD_Add0(0xAF);                     // Display on
  SSD1306_CMD_Send(hi2c);

  // clear all GDDRAM
  SSD1306_Clear();

  SSD1306_CMD_Int();
  SSD1306_CMD_Add2(0x21,0,127);
  SSD1306_CMD_Add2(0x22,0,3); // first half 128x32
  SSD1306_CMD_Send(hi2c);
  SSD1306_DATA_Send(hi2c);

  SSD1306_CMD_Int();
  SSD1306_CMD_Add2(0x21,0,127);
  SSD1306_CMD_Add2(0x22,4,7); // second half 128x32
  SSD1306_CMD_Send(hi2c);
  SSD1306_DATA_Send(hi2c);
}

void SSD1306_Screen_Update(I2C_HandleTypeDef *hi2c) {
//  if (SSD1306.Dirty) {
    SSD1306_CMD_Int();
    SSD1306_CMD_Add2(0x21, 0, SSD1306_WIDTH-1);       // Segment (column) start at 0 to 127
    SSD1306_CMD_Add2(0x22, 0, (SSD1306_HEIGHT/8)-1);  // Page (rows) start at 0 to 3
    SSD1306_CMD_Send(hi2c);
    SSD1306_DATA_Send(hi2c);
//    SSD1306.Dirty = 0;
    SSD1306.Frames++;
//  }
}

void SSD1306_Screen_Update_IT(I2C_HandleTypeDef *hi2c) {
//  if (SSD1306.Dirty) {
    SSD1306_CMD_Int();
    SSD1306_CMD_Add2(0x21, 0, SSD1306_WIDTH-1);       // Segment (column) start at 0 to Width
    SSD1306_CMD_Add2(0x22, 0, (SSD1306_HEIGHT/8)-1);  // Page (rows) start at 0 to Height
    SSD1306_CMD_Send(hi2c); // Wait for it, or call sending data later
    SSD1306_DATA_Send_IT(hi2c);
//    SSD1306.Dirty = 0;
    SSD1306.Frames++;
//  }
}

void SSD1306_Screen_Update_DMA(I2C_HandleTypeDef *hi2c) {
//  if (SSD1306.Dirty) {
    SSD1306_CMD_Int();
    SSD1306_CMD_Add2(0x21, 0, SSD1306_WIDTH-1);       // Segment (column) start at 0 to Width
    SSD1306_CMD_Add2(0x22, 0, (SSD1306_HEIGHT/8)-1);  // Page (rows) start at 0 to Height
    SSD1306_CMD_Send(hi2c); // Wait for it, or call sending data later
    SSD1306_DATA_Send_DMA(hi2c);
//    SSD1306.Dirty = 0;
    SSD1306.Frames++;
//  }
}

void SSD1306_Fill(SSD1306_COLOR_t color) {
  memset(SSD1306_buffer.DataBuffer + 1,
         (color == SSD1306_BLACK) ? 0x00 : 0xFF,
         SSD1306_BUFFER_DATA_MAX);
}

void SSD1306_Clear() {
  SSD1306_Fill(SSD1306_BLACK);
}

void SSD1306_InvertColors(void)
{
    SSD1306.Inverted = !SSD1306.Inverted;
}

void SSD1306_SetMask(int16_t x1, int16_t y1, int16_t x2, int16_t y2) {
  SSD1306.MaskX1 = (x1>=0 && x1<SSD1306_WIDTH) ? x1 : 0;
  SSD1306.MaskY1 = (y1>=0 && y1<SSD1306_HEIGHT) ? y1 : 0;
  SSD1306.MaskX2 = (x2>=0 && x2<SSD1306_WIDTH) ? x2 : SSD1306_WIDTH-1;
  SSD1306.MaskY2 = (y2>=0 && y2<SSD1306_HEIGHT) ? y2 : SSD1306_HEIGHT-1;
}

void SSD1306_DrawPixel(int16_t x, int16_t y, SSD1306_COLOR_t color) {
  if (x < SSD1306.MaskX1 ||
      y < SSD1306.MaskY1 ||
      x >= SSD1306.MaskX2 ||
      y >= SSD1306.MaskY2) {
    /* Error */
    return;
  }

  if (SSD1306.Inverted) {
    color = (SSD1306_COLOR_t)!color;
  }

  if(color == SSD1306_WHITE) {
    SSD1306_buffer.DataBuffer[1+ x + (y >> 3) * SSD1306_WIDTH] |= (1 << (y % 8));
  } else {
    SSD1306_buffer.DataBuffer[1+ x + (y >> 3) * SSD1306_WIDTH] &= ~(1 << (y % 8));
  }

//  SSD1306.Dirty = 1;
}

void SSD1306_WriteChar(int16_t x, int16_t y, char ch, FontDef_t* Font, SSD1306_COLOR_t color, SSD1306_DRAW_t mode)
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
              SSD1306_DrawPixel(x + x0, y + y0, (SSD1306_COLOR_t) color);
            }
            else if (mode == SSD1306_OVERRIDE)
            {
              SSD1306_DrawPixel(x + x0, y + y0, (SSD1306_COLOR_t)!color);
            }
        }
    }
}

void SSD1306_WriteString(int16_t x, int16_t y, char* str, FontDef_t* Font, SSD1306_COLOR_t color, SSD1306_DRAW_t mode)
{
    int16_t l = strlen(str);
    if (
        (x + l*Font->width < SSD1306.MaskX1) ||
        (SSD1306.MaskX2 < x) ||
        (y + Font->height < SSD1306.MaskY1) ||
        (SSD1306.MaskY2 < y)
    ){
      return;
    }

    int16_t fx = (SSD1306.MaskX1 - x) / Font->width;
    int16_t rx = (x - SSD1306.MaskX2) / Font->width;
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
        SSD1306_WriteChar(x + n*Font->width, y, *str, Font, color, mode);
        n++;
        str++;
    }
}


void SSD1306_DrawBitmap(int16_t x, int16_t y, const uint8_t* bitmap, int16_t w, int16_t h, SSD1306_COLOR_t color, SSD1306_DRAW_t mode)
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
              SSD1306_DrawPixel(x+i, y, color);
            }
            else if (mode == SSD1306_OVERRIDE)
            {
              SSD1306_DrawPixel(x+i, y, (SSD1306_COLOR_t)!color);
            }
        }
    }
}

void SSD1306_DrawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, SSD1306_COLOR_t color) {
  int16_t dx, dy, sx, sy, err, e2, i, tmp;

  /* Check for overflow */
  if (x0 >= SSD1306_WIDTH) {
    x0 = SSD1306_WIDTH - 1;
  }
  if (x1 >= SSD1306_WIDTH) {
    x1 = SSD1306_WIDTH - 1;
  }
  if (y0 >= SSD1306_HEIGHT) {
    y0 = SSD1306_HEIGHT - 1;
  }
  if (y1 >= SSD1306_HEIGHT) {
    y1 = SSD1306_HEIGHT - 1;
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
      SSD1306_DrawPixel(x0, i, color);
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
      SSD1306_DrawPixel(i, y0, color);
    }

    /* Return from function */
    return;
  }

  while (1) {
    SSD1306_DrawPixel(x0, y0, color);
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

void SSD1306_DrawRectangle(int16_t x, int16_t y, int16_t w, int16_t h, SSD1306_COLOR_t color) {
  /* Check input parameters */
  if (
    x >= SSD1306_WIDTH ||
    y >= SSD1306_HEIGHT
  ) {
    /* Return error */
    return;
  }

  /* Check width and height */
  if ((x + w) >= SSD1306_WIDTH) {
    w = SSD1306_WIDTH - x;
  }
  if ((y + h) >= SSD1306_HEIGHT) {
    h = SSD1306_HEIGHT - y;
  }

  /* Draw 4 lines */
  SSD1306_DrawLine(x, y, x + w, y, color);         /* Top line */
  SSD1306_DrawLine(x, y + h, x + w, y + h, color); /* Bottom line */
  SSD1306_DrawLine(x, y, x, y + h, color);         /* Left line */
  SSD1306_DrawLine(x + w, y, x + w, y + h, color); /* Right line */
}

void SSD1306_DrawFilledRectangle(int16_t x, int16_t y, int16_t w, int16_t h, SSD1306_COLOR_t color) {
  uint8_t i;

  /* Check input parameters */
  if (
    x >= SSD1306_WIDTH ||
    y >= SSD1306_HEIGHT
  ) {
    /* Return error */
    return;
  }

  /* Check width and height */
  if ((x + w) >= SSD1306_WIDTH) {
    w = SSD1306_WIDTH - x;
  }
  if ((y + h) >= SSD1306_HEIGHT) {
    h = SSD1306_HEIGHT - y;
  }

  /* Draw lines */
  for (i = 0; i <= h; i++) {
    /* Draw lines */
    SSD1306_DrawLine(x, y + i, x + w, y + i, color);
  }
}

void SSD1306_DrawTriangle(int16_t x1, int16_t y1, int16_t x2, int16_t y2, int16_t x3, int16_t y3, SSD1306_COLOR_t color) {
  /* Draw lines */
  SSD1306_DrawLine(x1, y1, x2, y2, color);
  SSD1306_DrawLine(x2, y2, x3, y3, color);
  SSD1306_DrawLine(x3, y3, x1, y1, color);
}


void SSD1306_DrawFilledTriangle(int16_t x1, int16_t y1, int16_t x2, int16_t y2, int16_t x3, int16_t y3, SSD1306_COLOR_t color) {
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
    SSD1306_DrawLine(x, y, x3, y3, color);

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

void SSD1306_DrawCircle(int16_t x0, int16_t y0, int16_t r, SSD1306_COLOR_t color) {
  int16_t f = 1 - r;
  int16_t ddF_x = 1;
  int16_t ddF_y = -2 * r;
  int16_t x = 0;
  int16_t y = r;

    SSD1306_DrawPixel(x0, y0 + r, color);
    SSD1306_DrawPixel(x0, y0 - r, color);
    SSD1306_DrawPixel(x0 + r, y0, color);
    SSD1306_DrawPixel(x0 - r, y0, color);

    while (x < y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;

        SSD1306_DrawPixel(x0 + x, y0 + y, color);
        SSD1306_DrawPixel(x0 - x, y0 + y, color);
        SSD1306_DrawPixel(x0 + x, y0 - y, color);
        SSD1306_DrawPixel(x0 - x, y0 - y, color);

        SSD1306_DrawPixel(x0 + y, y0 + x, color);
        SSD1306_DrawPixel(x0 - y, y0 + x, color);
        SSD1306_DrawPixel(x0 + y, y0 - x, color);
        SSD1306_DrawPixel(x0 - y, y0 - x, color);
    }
}

void SSD1306_DrawFilledCircle(int16_t x0, int16_t y0, int16_t r, SSD1306_COLOR_t color) {
  int16_t f = 1 - r;
  int16_t ddF_x = 1;
  int16_t ddF_y = -2 * r;
  int16_t x = 0;
  int16_t y = r;

    SSD1306_DrawPixel(x0, y0 + r, color);
    SSD1306_DrawPixel(x0, y0 - r, color);
    SSD1306_DrawPixel(x0 + r, y0, color);
    SSD1306_DrawPixel(x0 - r, y0, color);
    SSD1306_DrawLine(x0 - r, y0, x0 + r, y0, color);

    while (x < y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;

        SSD1306_DrawLine(x0 - x, y0 + y, x0 + x, y0 + y, color);
        SSD1306_DrawLine(x0 + x, y0 - y, x0 - x, y0 - y, color);

        SSD1306_DrawLine(x0 + y, y0 + x, x0 - y, y0 + x, color);
        SSD1306_DrawLine(x0 + y, y0 - x, x0 - y, y0 - x, color);
    }
}

void SSD1306_1sec_Tick() {
  SSD1306.Ticks++;
  if(SSD1306.Ticks >= 1000) {
    SSD1306.FPS = SSD1306.Frames;
    SSD1306.Frames = 0;
    SSD1306.Ticks = 0;
  }
}
