#pragma once
#include "ds_common.h"
#include <common_lib/i2c.h>
#include <time.h>

#define DS3231_ADDRESS 0x68

#define DS3231_SECONDS_REG 0x00
#define DS3231_CONTROL_REG 0x0E
#define DS3231_TEMPERATURE_REG 0x11

/**
 * Initialize device, check if device is connected to I2C bus
 */
i2c_device *ds3231_init(uint32_t i2c);

/**
 * Read date and time from device, if `convert_bcm_to_dec` is set to true,
 * perform conversion from Binary Coded Decimal to Decimal
 */
struct tm ds3231_read_date_raw(const i2c_device *dev, bool convert_bcm_to_dec);

/**
 * Read date and time from device
 * @param dev
 * @return
 */
static inline struct tm ds3231_read_date(const i2c_device *dev) {
  return ds3231_read_date_raw(dev, true);
}

/**
 * Read date and time from device in bcd format
 * @param dev
 * @return
 */
static inline struct tm ds3231_read_date_bcd(const i2c_device *dev) {
  return ds3231_read_date_raw(dev, false);
}

/**
 * Set provided date and time to device
 */
void ds3231_set_date(const i2c_device *dev, const struct tm *new_time);

/**
 * Get reading from internal temperature sensor
 * @param dev
 * @return temperature in Celcius
 */
float ds3231_get_temperature(const i2c_device *dev);
