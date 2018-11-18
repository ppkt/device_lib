#ifndef __MPU9250_H__
#define __MPU9250_H__

#include <math.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>

#include "common_lib/i2c_dma.h"
#include "common_lib/usart.h"
#include "common_lib/utils.h"

#define MPU9255_ADDRESS 0x68  // accel+gyro+temp
#define AK8963_ADDRESS 0x0C  // magnetometer

enum mpu9255_register_map {
    MPU9255_SELF_TEST_X_GYRO = 0x00,
    MPU9255_SELF_TEST_Y_GYRO = 0x01,
    MPU9255_SELF_TEST_Z_GYRO = 0x02,

    MPU9255_SELF_TEST_X_ACCEL = 0x0D,
    MPU9255_SELF_TEST_Y_ACCEL = 0x0E,
    MPU9255_SELF_TEST_Z_ACCEL = 0x0F,

    MPU9255_SMPLRT_DIV = 0x19,
    MPU9255_CONFIG = 0x1A,
    MPU9255_GYRO_CONFIG = 0x1B,
    MPU9255_ACCEL_CONFIG = 0x1C,
    MPU9255_ACCEL_CONFIG2 = 0x1D,

    MPU9255_INT_PIN_CFG = 0x37,
    MPU9255_INT_ENABLE = 0x38,

    MPU9255_ACCEL_XOUT_H = 0x3B,

    MPU9255_TEMP_OUT_H = 0x41,
    MPU9255_GYRO_XOUT_H = 0x43,

    MPU9255_PWR_MGMT_1 = 0x6B,

    MPU9255_WHO_AM_I = 0x75,
};

enum ak8963_register_map {
    AK8963_WIA = 0x00,
    AK8963_INFO = 0x01,
    AK8963_ST1 = 0x02,
    AK8963_HXL = 0x03,
    AK8963_ST2 = 0x09,
    AK8963_CNTL1 = 0x0A,
    AK8963_CNTL2 = 0x0B,
    AK8963_ASTC = 0x0C,
    AK8963_ASAX = 0x10,
};

union {
    uint8_t raw;
    struct __attribute__((packed)) {
        uint8_t dlpf_cfg:3;
        uint8_t ext_sync_set:3;
        uint8_t fifo_mode:1;
        uint8_t _:1;
    };
} mpu9255_config_reg;

union {
    uint8_t raw;
    struct __attribute__((packed)) {
        uint8_t fchoice_b:2;
        uint8_t _:1;
        uint8_t gyro_fs_sel:2;
        uint8_t z_gyro_cten:1;
        uint8_t y_gyro_cten:1;
        uint8_t x_gyro_cten:1;
    };
} mpu9255_gyro_config_reg;

union {
    uint8_t raw;
    struct __attribute__((packed)) {
        uint8_t _:3;
        uint8_t accel_fs_sel:2;
        uint8_t az_st_en:1;
        uint8_t ay_st_en:1;
        uint8_t ax_st_en:1;
    };
} mpu9255_accelerometer_config_reg;


union {
    uint8_t raw;
    struct __attribute__((packed)) {
        uint8_t A_DLPFCFG:3;
        uint8_t accel_fchoice_b:1;
        uint8_t __:2;
        uint8_t _:2;
    };
} mpu9255_accelerometer_config2_reg;

union {
    uint8_t raw;
    struct __attribute__((packed)) {
        uint8_t drdy:1;  // Data ready
        uint8_t dor:1;  // Data overrun
        uint8_t _:6;  // Read only!
    };
} ak8963_status1_reg;

union {
    uint8_t raw;
    struct __attribute__((packed)) {
        uint8_t _:3;
        uint8_t hofl:1;  // Magnetic sensor overflow
        uint8_t bitm:1;  // Mirror of ak8963_control1_reg.bit
        uint8_t __:3;
    };
} ak8963_status2_reg;  // Read only!

union {
    uint8_t raw;
    struct __attribute__((packed)) {
        uint8_t mode:4;  // operation mode setting
        uint8_t bit:1;  // Output bit setting - 0-14b, 1-16b
        uint8_t _:3;
    };
} ak8963_control1_reg;

union {
    uint8_t raw;
    struct __attribute__((packed)) {
        uint8_t srst:1;  // soft reset
        uint8_t _:7;
    };
} ak8963_control2_reg;

union {
    uint8_t raw;
    struct __attribute__((packed)) {
        uint8_t __:6;
        uint8_t self:1;  // 1 - generate magnetic field for self-test
        uint8_t _:1;
    };
} ak8963_self_test_reg;

typedef struct {
    bool precision;
} ak8963_settings;

typedef struct {
    uint8_t address;
    I2C_TypeDef *i2c;
    TIM_TypeDef *timer;
} i2c_device;

i2c_device *mpu9255_init(I2C_TypeDef *I2Cx, TIM_TypeDef *timer);

bool mpu9255_check_presence(const i2c_device *device);

void mpu9255_self_test(const i2c_device *device);

void mpu9255_configure(const i2c_device *device);

int16_t *mpu9255_read_accelerometer_data(const i2c_device *device);

int16_t *mpu9255_read_gyroscope_data(const i2c_device *device);

float mpu9255_read_temperature(const i2c_device *device);

i2c_device *ak8963_init(I2C_TypeDef *I2Cx, TIM_TypeDef *timer);

void ak8963_configure(const i2c_device *device,
                      const ak8963_settings *settings);

bool ak8963_check_presence(const i2c_device *device);

bool ak8963_self_test(const i2c_device *device, const ak8963_settings *settings);

float *ak8963_read_magnetometer_data(const i2c_device *device,
                                     const ak8963_settings *settings);

#endif // __MPU9250_H__
