#pragma once

#include "math.h"
#include "stdbool.h"
#include "stdio.h"
#include "stdlib.h"

#include "common_lib/i2c.h"
#include "common_lib/usart.h"

typedef struct {
    uint32_t i2c;
    uint8_t address;

    uint16_t calibration_register;
    // raw values
    int16_t raw_bus_voltage;
    int16_t raw_shunt_voltage;
    int16_t raw_current;
    int16_t raw_power;
    // converted values
    float bus_voltage;
    float shunt_voltage;
    float current;
    float power;
} ina219_device;


typedef enum {
    ina219_register_configuration = 0x00,
    ina219_register_shunt_voltage,
    ina219_register_bus_voltage,
    ina219_register_power,
    ina219_register_current,
    ina219_register_calibration,
} ina219_register;

typedef struct {
    union {
        uint16_t raw;
        struct __attribute__((packed)) {
            uint8_t mode:3;
            uint8_t sadc:4;
            uint8_t badc:4;
            uint8_t pga:2;
            uint8_t brng:1;
            uint8_t _:1;
            uint8_t rst:1;
        };
    };
} ina219_config;

bool ina219_setup(ina219_device **dev, uint32_t i2c, uint8_t address);
bool ina219_init(ina219_device* device);
bool ina219_perform_calibration(ina219_device *device);

void ina219_get_bus_voltage(ina219_device *device);
void ina219_get_shunt_voltage(ina219_device *device);
void ina219_get_current(ina219_device *device);
void ina219_get_power(ina219_device *device);
void ina219_get_all_readings(ina219_device *device);
