#ifndef __HC_SR04_H__
#define __HC_SR04_H__

#include <stm32f10x.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_gpio.h>

#include "common_lib/utils.h"

void sr04_init(void);
void sr04_trigger(void);
uint16_t sr04_get_delay(void);

#endif
