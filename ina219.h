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
float ina219_read_bus_voltage(void);
int16_t ina219_read_shunt_voltage(void);
uint16_t ina219_read_current(uint16_t calibration_register);

#endif // __INA291_H__
