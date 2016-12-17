#ifndef __AT24C64_H__
#define __AT24C64_H__

#include "stdlib.h"
#include "string.h"
#include "stdint.h"
#include "stdbool.h"

#include "stm32f10x.h"

#include "common_lib/utils.h"

#include "common_lib/i2c_dma.h"

#define AT24C64_ADDRESS 0x50

uint8_t at24c64_write_bytes(uint16_t address, uint8_t *data, uint16_t n,
                            TIM_TypeDef *timer);
uint8_t at24c64_read_bytes(uint16_t address, uint8_t *data, uint16_t n);

#endif // __AT24C64_H__
