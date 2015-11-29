#ifndef __INA291_H__
#define __INA291_H__

#include "math.h"
#include "stdbool.h"
#include "stdio.h"
#include "stdlib.h"

#include "stm32f10x.h"

#include "common_lib/i2c_dma.h"

void ina219_setup(void);
uint16_t ina219_perform_calibration(void);
uint16_t ina219_read_bus_voltage(void);
int16_t ina219_read_shunt_voltage(void);
uint16_t ina219_read_current(void);
uint16_t ina219_get_power(void);

void ina219_print_bus_voltage(void);
void ina219_print_shunt_voltage(void);
void ina219_print_current(void);
void ina219_print_power(void);

#endif // __INA291_H__
