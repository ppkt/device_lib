#ifndef __BME280_H__
#define __BME280_H__

#include <stdbool.h>
#include <stdlib.h>

#include "common_lib/i2c.h"
#include "common_lib/usart.h"
#include "common_lib/utils.h"

typedef enum {
  id = 0xD0,
  reset = 0x0E,
  ctrl_hum = 0xF2,
  status = 0xF3,
  ctrl_meas = 0xF4,
  config = 0xF5,

  press = 0xF7,
  temp = 0xFA,
  hum = 0xFD,

  calib00 = 0x88,
  calib26 = 0xE1,
} bme280_register_map;

typedef enum {
  disable = 0b000,
  x1 = 0b001,
  x2 = 0b010,
  x4 = 0b011,
  x8 = 0b100,
  x16 = 0b101,
} bme280_oversampling;

typedef enum {
  t_0_5 = 0b000,
  t_62_5 = 0b001,
  t_125 = 0b010,
  t_250 = 0b011,
  t_500 = 0b100,
  t_1000 = 0b101,
  t_10 = 0b110,
  t_20 = 0b111,
} bme280_t_standby_duration; // in ms

typedef enum {
  off = 0b000,
  c2 = 0b001,
  c4 = 0b010,
  c8 = 0b011,
  c16 = 0b100,
} bme280_filter_coefficient;

typedef enum {
  sleep = 0b00,
  forced = 0b01,
  normal = 0b11,
} bme280_mode;

typedef struct {
  uint32_t pressure;
  uint32_t temperature;
  uint16_t humidity;
} bme280_raw_reading;

typedef struct {
  float temperature;
  float humidity;
  uint32_t pressure;

} bme280_reading;

typedef struct {
  int16_t T1;
  int16_t T2;
  int16_t T3;

  uint16_t P1;
  int16_t P2;
  int16_t P3;
  int16_t P4;
  int16_t P5;
  int16_t P6;
  int16_t P7;
  int16_t P8;
  int16_t P9;

  uint8_t H1;
  int16_t H2;
  uint8_t H3;
  int16_t H4;
  int16_t H5;
  int8_t H6;

} bme280_compensation_data;

typedef struct {
  i2c_device i2c_dev;

  bme280_oversampling humidity_oversampling;
  bme280_oversampling pressure_oversampling;
  bme280_oversampling temperature_oversampling;
  bme280_mode mode;

  bme280_t_standby_duration standby_duration;
  bme280_filter_coefficient filter_coefficient;

  bme280_raw_reading raw_reading;
  bme280_compensation_data compensation_data;
  bme280_reading reading;
} bme280_device;

error_t bme280_init(uint32_t i2c, bme280_device *dev);
error_t bme280_commit(bme280_device *device);
error_t bme280_load_compensation_data(bme280_device *device);
error_t bme280_read(bme280_device *device, bool humidity);
error_t bme280_calibrated_read(bme280_device *device);

#endif // __BME280_H__
