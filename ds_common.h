#pragma once

#include <common_lib/i2c.h>
#include <stdint-gcc.h>
#include <time.h>

/**
 * Convert hex value in BCD format to decimal, eg 0x55 -> 55
 */
static inline uint8_t bcd_to_decimal(uint8_t hex) {
  return (uint8_t)((hex >> 4) * 10 + (hex & 0x0F));
}

/**
 * Convert decimal to hex BCD format, eg 55 -> 0x55
 */
static inline uint8_t decimal_to_bcd(uint8_t dec) {
  return (uint8_t)(((dec / 10) << 4) + (dec % 10));
}

/**
 * Noop function used instead of `bcd_to_decimal` and `decimal_to_bcd`
 */
static inline uint8_t noop(uint8_t x) { return x; }

/**
 * Read date from device
 * @param dev
 * @param convert_bcm_to_dec if `true`, convert from BCM format to decimal
 * @param seconds_register address of register with seconds
 * @return
 */
struct tm ds_read_date_raw(const i2c_device *dev, bool convert_bcm_to_dec,
                           uint8_t seconds_register);

/**
 * Set date on device
 * @param dev device
 * @param new_time struct with new time
 * @param seconds_register address of register with seconds
 */
void ds_set_date(const i2c_device *dev, const struct tm *new_time,
                 uint8_t seconds_register);
