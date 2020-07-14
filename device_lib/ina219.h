#pragma once

#include "math.h"
#include "stdbool.h"
#include "stdio.h"
#include "stdlib.h"

#include "common_lib/i2c.h"
#include "common_lib/usart.h"

typedef enum {
  INA219_GAIN_1,
  INA219_GAIN_2,
  INA219_GAIN_4,
  INA219_GAIN_8,
} ina219_pga;

typedef enum {
  INA219_MODE_POWER_DOWN = 0b000,
  INA219_MODE_SHUNT_VOLTAGE_ONESHOT = 0b001,
  INA219_MODE_BUS_VOLTAGE_ONESHOT = 0b010,
  INA219_MODE_SHUNT_AND_BUS_VOLTAGE_ONESHOT = 0b011,
  INA219_MODE_ADC_OFF = 0b100,
  INA219_MODE_SHUNT_VOLTAGE_CONTINUOUS = 0b101,
  INA219_MODE_BUS_VOLTAGE_CONTINUOUS = 0b110,
  INA219_MODE_SHUNT_AND_BUS_VOLTAGE_CONTINUOUS = 0b111,
} ina219_mode;

typedef struct {
  union {
    uint16_t raw;
    struct __attribute__((packed)) {
      ina219_mode mode : 3;
      uint16_t sadc : 4;
      uint16_t badc : 4;
      ina219_pga pga : 2;
      uint16_t brng : 1;
      uint16_t _ : 1;
      uint16_t rst : 1;
    };
  };
} ina219_config;

typedef enum {
  ina219_register_configuration = 0x00,
  ina219_register_shunt_voltage,
  ina219_register_bus_voltage,
  ina219_register_power,
  ina219_register_current,
  ina219_register_calibration,
} ina219_register;

typedef struct {
  uint32_t i2c;
  uint8_t address;

  ina219_config config;
  struct {
    float current_div;
    float power_multiplier;
  } calibration;
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

error_t ina219_init(ina219_device *device, uint32_t i2c, uint8_t address);
error_t ina219_write_config(ina219_device *device);
error_t ina219_perform_calibration(ina219_device *device);
error_t ina219_calibration_32v_2a(ina219_device *device);
error_t ina219_calibration_16v_400ma(ina219_device *device);

error_t ina219_get_bus_voltage(ina219_device *device);
error_t ina219_get_shunt_voltage(ina219_device *device);
error_t ina219_get_current(ina219_device *device);
error_t ina219_get_power(ina219_device *device);
error_t ina219_get_all_readings(ina219_device *device);
