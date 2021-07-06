#ifndef _FONTS_H
#define _FONTS_H

#include <stdint.h>
#include <string.h>

//
//  Structure used to define fonts
//
typedef struct {
    const uint8_t width;    /* Font width in pixels */
    const uint8_t height;   /* Font height in pixels */
    const int16_t *data;       /* Pointer to data font data array */
} FontDef_t;

//
//  Export the 3 available fonts
//
extern FontDef_t Font_7x10;
extern FontDef_t Font_11x18;
extern FontDef_t Font_16x26;
//
//  Static method
//
uint16_t Font_GetStringHeight(const char* str, const FontDef_t* Font);
uint16_t Font_GetStringWidth(const char* str, const FontDef_t* Font);

#endif  // _FONTS_H
