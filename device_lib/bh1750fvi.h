#pragma once

#include <common_lib/i2c.h>

#define BH1750FVI_ADDRESS 0x23

#define BH1750FVI_POWER_DOWN_CMD 0x00
#define BH1750FVI_POWER_ON_CMD 0x01
#define BH1750FVI_RESET_CMD 0x07
#define BH1750FVI_SINGLE_SHOT_HRES_CMD 0x20
#define BH1750FVI_SINGLE_SHOT_HRES2_CMD 0x21
#define BH1750FVI_SINGLE_SHOT_LRES_CMD 0x23

typedef enum {
  BH1750FVI_HIGH_RESOLUTION,
  BH1750FVI_HIGH_RESOLUTION_2,
  BH1750FVI_LOW_RESOLUTION,
} bh1750fvi_resolution;

/**
 * Initialize device
 * @param i2c
 * @param dev
 * @return
 */
error_t bh1750fvi_init(uint32_t i2c, i2c_device *dev);

/**
 * Read single value from device
 * @param dev : i2c device
 * @param resolution : reading resolution
 * @param val : value in lx
 * @return
 */
error_t bh1750fvi_read_single_shot(const i2c_device *dev,
                                   bh1750fvi_resolution resolution, float *val);
