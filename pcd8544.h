#ifndef __PCD8544_H__
#define __PCD8544_H__

#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "stm32f10x_tim.h"

#include "common_lib/spi.h"
#include "common_lib/utils.h"

void pcd8544_init(TIM_TypeDef *timer);
void pcd8544_draw_pixel(uint8_t x, uint8_t y);
void pcd8544_clear(bool value);
void pcd8544_set_autoupdate(bool value);
void pcd8544_force_update(void);

#endif //  __PCD8544_H__
