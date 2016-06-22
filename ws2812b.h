#ifndef __WS2812B_H__
#define __WS2812B_H__

#include <stm32f10x_tim.h>
#include <stm32f10x_dma.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>
#include <misc.h>
#include <string.h>
#include <stdbool.h>

typedef struct RgbColor
{
    unsigned char r;
    unsigned char g;
    unsigned char b;
} RgbColor;

typedef struct HsvColor
{
    unsigned char h;
    unsigned char s;
    unsigned char v;
} HsvColor;

void ws2812b_init(void);
void WS2812_send(u8 r, u8 g, u8 b);
HsvColor RgbToHsv(RgbColor rgb);
RgbColor HsvToRgb(HsvColor hsv);

#endif // __WS2812B_H__
