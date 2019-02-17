#pragma once

#include <memory.h>
#include <stdlib.h>
#include <time.h>

#include <common_lib/i2c.h>
#include <common_lib/usart.h>
#include <common_lib/utils.h>

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
 * Convert hex value in BCD format to decimal, eg 0x55 -> 55
 */
static inline uint8_t
bcd_to_decimal(uint8_t hex) {
    return (uint8_t) ((hex >> 4) * 10 + (hex & 0x0F));
}

/**
 * Convert decimal to hex BCD format, eg 55 -> 0x55
 */
static inline uint8_t
decimal_to_bcd(uint8_t dec) {
    return (uint8_t) (((dec / 10) << 4) + (dec % 10));
}

/**
 * Perform device initialization, check if device is connected to I2C bus
 */
i2c_device *
ds1307_init(uint32_t i2c);

/**
 * Read date and time from device
 */
struct tm
ds1307_read_date(const i2c_device *dev);

/**
 * Set provided date and time to device
 */
void
ds1307_set_date(const i2c_device *dev, const struct tm *new_time);

/**
 * Return content of Control Register
 */
ds1307_control_register_s
ds1307_read_control_register(const i2c_device *dev);

/**
 * Read `bytes` bytes from device NVRAM starting from `offset` address. If
 * `offset` + `bytes` > 56, return array with len `bytes` but filled with zeroes
 * after reaching end of buffer. eg:
 * offset = 54, bytes = 4 -> return [ NVRAM(54), NVRAM(55), 0, 0 ]
 * if `offset` is greater than 54 or `bytes` 0 is passed, null pointer is
 * returned
 */
uint8_t
*ds1307_read_nvram(const i2c_device *dev, uint8_t offset, uint8_t bytes);

/**
 * Write `tx` bytes to NVRAM starting from `offset`. If length of array is
 * greater than size of NVRAM, exceeding elements will not be written.
 */
void
ds1307_write_nvram(const i2c_device *dev, uint8_t offset,
                   uint8_t *tx, uint8_t tx_len);

/**
 * Clear entire NVRAM
 */
void
ds1307_erase_nvram(const i2c_device *dev);
