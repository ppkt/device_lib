#include "mpu9255.h"

static uint8_t tx[2] = {0,};
static uint8_t rx[6] = {0,};

// Send single byte to device
void i2c_sendbyte(mpu9255_device *device, uint8_t reg, uint8_t data) {
    tx[0] = reg;
    tx[1] = data;
    I2C_Master_BufferWrite(device->i2c, tx, 2, Polling, device->address << 1);
}

// Read single byte from device
uint8_t i2c_readbyte(mpu9255_device *device, uint8_t reg) {
    tx[0] = reg;
    i2c_master_transaction_write_read(device->i2c, device->address,
            tx, 1, rx, 1, Polling);
    return rx[0];
}

// Read n bytes from device
uint8_t *i2c_readbytes(mpu9255_device *device, uint8_t reg, uint8_t bytes) {
    tx[0] = reg;
    uint8_t *rx_data = calloc(bytes, sizeof(uint8_t));
    i2c_master_transaction_write_read(device->i2c, device->address,
                                      tx, 1, rx_data, bytes, Polling);
    return rx_data;
}

// Perform initialization
mpu9255_device *mpu9255_init(I2C_TypeDef *I2Cx, TIM_TypeDef *timer) {
    I2C_LowLevel_Init(I2Cx);

    mpu9255_device *dev = malloc(sizeof(mpu9255_device));
    dev->i2c = I2Cx;
    dev->address = MPU9255_ADDRESS;
    dev->timer = timer;
    return dev;
}

// Check if device is present on I2C bus
bool mpu9255_check_presence(mpu9255_device *device) {
    return i2c_readbyte(device, MPU9255_WHO_AM_I) == 0x73;
}

// Perform self test of device based on implementation from:
// https://github.com/kriswiner/MPU-9250
void mpu9255_self_test(mpu9255_device *device) {
    // set sample divider to 1 kHz
    i2c_sendbyte(device, MPU9255_SMPLRT_DIV, 0x00);

    // TODO: check whether is possible to perform single write

    // set DLPF to 92 Hz
    i2c_sendbyte(device, MPU9255_CONFIG, 0x02);

    // set gyroscope scale to 250 dfs
    i2c_sendbyte(device, MPU9255_GYRO_CONFIG, 0x00);

    // set accelerometer bandwidth to 92 Hz
    i2c_sendbyte(device, MPU9255_ACCEL_CONFIG2, 0x02);

    // set accelerometer scale to 2g
    i2c_sendbyte(device, MPU9255_ACCEL_CONFIG, 0x00);
    delay_ms(device->timer, 200);

    // fetch 200 samples
    uint8_t samples = 200;
    uint32_t acc_avg[3] = {0,};
    uint32_t gyro_avg[3] = {0,};
    uint32_t test_acc_avg[3] = {0,};
    uint32_t test_gyro_avg[3] = {0,};
    double factory_trim[6] = {0.0,};
    double destination[6] = {0.0,};
    uint8_t self_test[6] = {0,};
    uint8_t *acc_data, *gyro_data;

    for (uint8_t i = 0; i < samples; ++i) {
        // fetch accelerometer and gyroscope reading
        acc_data = i2c_readbytes(device, MPU9255_ACCEL_XOUT_H, 6);
        gyro_data = i2c_readbytes(device, MPU9255_GYRO_XOUT_H, 6);

        for (uint8_t j = 0; j < 3; ++j) {
            acc_avg[j] += acc_data[2 * j] << 8 | acc_data[2 * j + 1];
            gyro_avg[j] += gyro_data[2 * j] << 8 | gyro_data[2 * j + 1];
        }

        free(acc_data);
        free(gyro_data);
    }

    usart1_print("Average reading\r\n");
    // get average reading
    for (uint8_t i = 0; i < 3; ++i) {
        acc_avg[i] /= samples;
        gyro_avg[i] /= samples;
        usart1_printf("[acc] %i: %u\r\n", i, acc_avg[i]);
        usart1_printf("[gyro] %i: %u\r\n", i, gyro_avg[i]);
    }

    // configure gyroscope and accelerometer to perform self test
    i2c_sendbyte(device, MPU9255_GYRO_CONFIG, 0xE0);
    i2c_sendbyte(device, MPU9255_ACCEL_CONFIG, 0xE0);
    delay_ms(device->timer, 200);

    for (uint8_t i = 0; i < samples; ++i) {
        // fetch accelerometer and gyroscope reading
        acc_data = i2c_readbytes(device, MPU9255_ACCEL_XOUT_H, 6);
        gyro_data = i2c_readbytes(device, MPU9255_GYRO_XOUT_H, 6);

        for (uint8_t j = 0; j < 3; ++j) {
            test_acc_avg[j] += acc_data[2 * j] << 8 | acc_data[2 * j + 1];
            test_gyro_avg[j] += gyro_data[2 * j] << 8 | gyro_data[2 * j + 1];
        }

        free(acc_data);
        free(gyro_data);
    }

    usart1_print("Test average reading");
    // get average reading
    for (uint8_t i = 0; i < 3; ++i) {
        test_acc_avg[i] /= samples;
        test_gyro_avg[i] /= samples;

        usart1_printf("[acc] %i: %u\r\n", i, test_acc_avg[i]);
        usart1_printf("[gyro] %i: %u\r\n", i, test_gyro_avg[i]);
    }

    // configure gyroscope and accelerometer to normal operation
    i2c_sendbyte(device, MPU9255_GYRO_CONFIG, 0x00);
    i2c_sendbyte(device, MPU9255_ACCEL_CONFIG, 0x00);
    delay_ms(device->timer, 200);

    // fetch factory values
    self_test[0] = i2c_readbyte(device, MPU9255_SELF_TEST_X_ACCEL);
    self_test[1] = i2c_readbyte(device, MPU9255_SELF_TEST_Y_ACCEL);
    self_test[2] = i2c_readbyte(device, MPU9255_SELF_TEST_Z_ACCEL);
    self_test[3] = i2c_readbyte(device, MPU9255_SELF_TEST_X_GYRO);
    self_test[4] = i2c_readbyte(device, MPU9255_SELF_TEST_Y_GYRO);
    self_test[5] = i2c_readbyte(device, MPU9255_SELF_TEST_Z_GYRO);

    for (uint8_t i = 0; i < 6; ++i) {
        factory_trim[i] = 2620 * pow(1.01, self_test[i] - 1);
        usart1_printf("%u %\r\n", self_test[i], factory_trim[i]);
    }

    // get difference between factory trim and self test
    for (uint8_t i = 0; i < 3; ++i) {
        destination[i] = (float) (test_acc_avg[i] - acc_avg[i]) / factory_trim[i];
        destination[i + 3] = (float) (test_gyro_avg[i] - gyro_avg[i]) / factory_trim[i + 3];
    }

    for (uint8_t i = 0; i < 6; ++i) {
        usart1_printf("%d: %f\r\n", i, destination[i]);
    }
}

void mpu9255_configure(mpu9255_device *device) {
    i2c_sendbyte(device, MPU9255_PWR_MGMT_1, 0x00);
    delay_ms(device->timer, 100);

    i2c_sendbyte(device, MPU9255_PWR_MGMT_1, 0x01);
    delay_ms(device->timer, 200);

    i2c_sendbyte(device, MPU9255_CONFIG, 0x03);

    i2c_sendbyte(device, MPU9255_SMPLRT_DIV, 0x04);

    uint8_t gyro_config = i2c_readbyte(device, MPU9255_GYRO_CONFIG);
    // Clear Fchoice bits [1:0]
    gyro_config = gyro_config & (uint8_t) ~0x02;
    // Clear AFS bits [4:3]
    gyro_config = gyro_config & (uint8_t) ~0x18;
    i2c_sendbyte(device, MPU9255_GYRO_CONFIG, gyro_config);

    uint8_t accel_config = i2c_readbyte(device, MPU9255_ACCEL_CONFIG);
    accel_config = accel_config & (uint8_t) ~0x18;
    i2c_sendbyte(device, MPU9255_ACCEL_CONFIG, accel_config);

    accel_config = i2c_readbyte(device, MPU9255_ACCEL_CONFIG2);
    // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
    accel_config = accel_config & (uint8_t) ~0x0F;
    // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
    accel_config = accel_config | (uint8_t) 0x03;
    i2c_sendbyte(device, MPU9255_ACCEL_CONFIG2, accel_config);

    i2c_sendbyte(device, MPU9255_INT_PIN_CFG, 0x22);
    i2c_sendbyte(device, MPU9255_INT_ENABLE, 0x01);

    delay_ms(device->timer, 200);
}

int16_t *mpu9255_read_accelerometer_data(mpu9255_device *device) {
    uint8_t *raw_accelerometer_data = i2c_readbytes(device, MPU9255_ACCEL_XOUT_H, 6);
    int16_t *accelerometer_data = calloc(3, sizeof(uint16_t));

    for (uint8_t i = 0; i < 3; ++i) {
        accelerometer_data[i] += raw_accelerometer_data[2 * i] << 8 | \
                                 raw_accelerometer_data[2 * i + 1];
    }

    free(raw_accelerometer_data);
    return accelerometer_data;
}

int16_t *mpu9255_read_gyroscope_data(mpu9255_device *device) {
    uint8_t *raw_gyroscope_data = i2c_readbytes(device, MPU9255_GYRO_XOUT_H, 6);
    int16_t *gyroscope_data = calloc(3, sizeof(uint16_t));
    for (uint8_t i = 0; i < 3; ++i) {
        gyroscope_data[i] += raw_gyroscope_data[2 * i] << 8 | \
                             raw_gyroscope_data[2 * i + 1];
    }
    free(raw_gyroscope_data);
    return gyroscope_data;
}
