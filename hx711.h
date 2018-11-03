#ifndef __HX711_H__
#define __HX711_H__

#include "math.h"

#include "stm32f10x_rcc.h"

#include "common_lib/utils.h"

void hx711_init(TIM_TypeDef *timer);
int32_t hx711_single_read(void);
int32_t hx711_avg_read(uint8_t samples);
float hx711_read_gram(void);

#endif // __HX711_H__
