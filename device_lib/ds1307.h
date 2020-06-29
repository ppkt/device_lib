#pragma once

#include "ds_common.h"

#include <common_lib/i2c.h>
#include <common_lib/usart.h>
#include <common_lib/utils.h>

#include <memory.h>
#include <stdlib.h>
#include <time.h>

#define DS1307_ADDRESS 0x68

#define DS1307_SECONDS_REG 0x00
#define DS1307_CONTROL_REG 0x07
#define DS1307_NVRAM_REG 0x08

#define DS1307_NVRAM_SIZE 56

typedef struct {
    union {
        uint8_t raw;
        struct {
            uint8_t rs:2;
            uint8_t :2;
            uint8_t sqwe:1;
            uint8_t :2;
            uint8_t out:1;
        };
    };
} ds1307_control_register_s;



/**
 * Perform device initialization, check if device is connected to I2C bus
 */
i2c_device *ds1307_init(uint32_t i2c);

/**
 * Read date and time from device, if `convert_bcm_to_dec` is set to true,
 * perform conversion from Binary Coded Decimal to Decimal
 */
struct tm ds1307_read_date_raw(const i2c_device *dev, bool convert_bcm_to_dec);

/**
 * Read date and time from device
 * @param dev
 * @return
 */
static inline struct tm ds1307_read_date(const i2c_device *dev) {
  return ds1307_read_date_raw(dev, true);
}

/**
 * Read date and time from device in bcd format
 * @param dev
 * @return
 */
static inline struct tm ds1307_read_date_bcd(const i2c_device *dev) {
  return ds1307_read_date_raw(dev, false);
}

/**
 * Set provided date and time to device
 */
void ds1307_set_date(const i2c_device *dev, const struct tm *new_time);

/**
 * Return content of Control Register
 */
ds1307_control_register_s ds1307_read_control_register(const i2c_device *dev);

/**
 * Read `bytes` bytes from device NVRAM starting from `offset` address. If
 * `offset` + `bytes` > 56, return array with len `bytes` but filled with zeroes
 * after reaching end of buffer. eg:
 * offset = 54, bytes = 4 -> return [ NVRAM(54), NVRAM(55), 0, 0 ]
 * if `offset` is greater than 54 or `bytes` 0 is passed, null pointer is
 * returned
 */
uint8_t *ds1307_read_nvram(const i2c_device *dev, uint8_t offset,
                           uint8_t bytes);

/**
 * Write `tx` bytes to NVRAM starting from `offset`. If length of array is
 * greater than size of NVRAM, exceeding elements will not be written.
 */
void ds1307_write_nvram(const i2c_device *dev, uint8_t offset, uint8_t *tx,
                        uint8_t tx_len);

/**
 * Clear entire NVRAM
 */
void ds1307_erase_nvram(const i2c_device *dev);
