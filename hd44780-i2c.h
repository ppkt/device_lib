#ifndef __HD44780_I2C_H__
#define __HD44780_I2C_H__
#include "utils.h"
#include "i2c_dma.h"
#include "stdbool.h"
#include <stm32f10x.h>
#include <stm32f10x_tim.h>

#define hd44780_address 0x27
void hd44780_init(TIM_TypeDef *timer);
void hd44780_print(char *string);
void hd44780_go_to_line(u8 line);
void hd44780_go_to(u8 row, u8 col);
void hd44780_cmd(u8 cmd);


#endif // __HD44780_I2C_H__
