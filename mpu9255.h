#ifndef __MPU9250_H__
#define __MPU9250_H__

#include <math.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>

#include "common_lib/i2c_dma.h"
#include "common_lib/usart.h"
#include "common_lib/utils.h"

#define MPU9255_ADDRESS 0x68

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

    MPU9255_GYRO_XOUT_H = 0x43,

    MPU9255_PWR_MGMT_1 = 0x6B,

    MPU9255_WHO_AM_I = 0x75,
};

typedef struct {
    uint8_t address;
    I2C_TypeDef *i2c;
    TIM_TypeDef *timer;
} mpu9255_device;

mpu9255_device *mpu9255_init(I2C_TypeDef *I2Cx, TIM_TypeDef *timer);

bool mpu9255_check_presence(mpu9255_device *device);

void mpu9255_self_test(mpu9255_device *device);

void mpu9255_configure(mpu9255_device *device);

int16_t *mpu9255_read_accelerometer_data(mpu9255_device *device);

int16_t *mpu9255_read_gyroscope_data(mpu9255_device *device);

#endif // __MPU9250_H__
