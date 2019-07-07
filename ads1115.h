#pragma once

#include <common_lib/i2c.h>

#define ADS1115_ADDRESS 0x48

#define ADS1115_CONVERSION_REG 0x00
#define ADS1115_CONFIG_REG 0x01

typedef union {
  struct __attribute__((packed)) {
    uint8_t comp_que : 2;
    uint8_t comp_lat : 1;
    uint8_t comp_pol : 1;
    uint8_t comp_mode : 1;
    uint8_t dr : 3;
    uint8_t mode : 1;
    uint8_t pga : 3;
    uint8_t mux : 3;
    uint8_t os : 1;
  } reg;
  uint16_t raw;
} ads1115_config;

/**
 * Initialize device
 * @param i2c
 * @param dev
 * @return
 */
error_t ads1115_init(uint32_t i2c, i2c_device *dev);

/**
 * Read value of config register
 * @param dev : i2c device
 * @param reg : allocated structure for holding register values
 * @return
 */
error_t ads1115_read_config(const i2c_device *dev, ads1115_config *reg);

/**
 * Update device config using provided structure
 * @param dev : i2c device
 * @param reg : structure with updated values to write
 * @return
 */
error_t ads1115_write_config(const i2c_device *dev, const ads1115_config *reg);

/**
 * Perform single-shot read from device and convert reading to mV
 * @param dev : i2c device
 * @param voltage : variable to hold result (in mV)
 * @return
 */
error_t ads1115_read_single_shot(const i2c_device *dev, float *voltage);
