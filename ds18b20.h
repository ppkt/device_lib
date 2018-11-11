#pragma once
#ifndef __DS18B20_H__
#define __DS18B20_H__

//#include <math.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

#include "common_lib/one_wire.h"
#include "common_lib/utils.h"
#include "common_lib/usart.h"

// Structure for returning list of devices on one wire
typedef struct {
    uint8_t size;
    one_wire_device *devices;
} ds18b20_devices;

ds18b20_devices ds18b20_init(GPIO_TypeDef *gpio, uint16_t port, TIM_TypeDef *timer);

void ds18b20_set_precision(uint8_t precission);

ds18b20_devices ds18b20_get_devices(bool scan);

void ds18b20_convert_temperature_simple(void);

float ds18b20_read_temperature_simple(void);

void ds18b20_convert_temperature_all(ds18b20_devices devices);

float *ds18b20_read_temperature_all(ds18b20_devices devices);

void ds18b20_wait_for_conversion(void);

float ds18b20_decode_temperature(void);

float ds18b20_get_temperature_simple(void);

float *ds18b20_get_temperature_all(ds18b20_devices devices);

#endif // __DS18B20_H__
