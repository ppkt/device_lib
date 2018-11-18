#include "mpu9255.h"

static uint8_t tx[2] = {0,};
static uint8_t rx[6] = {0,};

// Send single byte to device
void i2c_sendbyte(const i2c_device *device, uint8_t reg, uint8_t data) {
    tx[0] = reg;
    tx[1] = data;
    I2C_Master_BufferWrite(device->i2c, tx, 2, Polling, device->address << 1);
}

// Read single byte from device
uint8_t i2c_readbyte(const i2c_device *device, uint8_t reg) {
    tx[0] = reg;
    Status s = i2c_master_transaction_write_read(device->i2c, device->address,
                                                 tx, 1, rx, 1, Polling);
    if (s == Error)
        hacf();

    return rx[0];
}

// Read n bytes from device
uint8_t *i2c_readbytes(const i2c_device *device, uint8_t reg, uint8_t bytes) {
    tx[0] = reg;
    uint8_t *rx_data = calloc(bytes, sizeof(uint8_t));
    i2c_master_transaction_write_read(device->i2c, device->address,
                                      tx, 1, rx_data, bytes, Polling);
    return rx_data;
}

// Perform initialization
i2c_device *mpu9255_init(I2C_TypeDef *I2Cx, TIM_TypeDef *timer) {
    I2C_LowLevel_Init(I2Cx);

    i2c_device *dev = malloc(sizeof(i2c_device));
    dev->i2c = I2Cx;
    dev->address = MPU9255_ADDRESS;
    dev->timer = timer;
    return dev;
}

// Check if device is present on I2C bus
bool mpu9255_check_presence(const i2c_device *device) {
    return i2c_readbyte(device, MPU9255_WHO_AM_I) == 0x73;
}

// Perform self test of device based on implementation from:
// https://github.com/kriswiner/MPU-9250
void mpu9255_self_test(const i2c_device *device) {
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

void mpu9255_configure(const i2c_device *device) {
    i2c_sendbyte(device, MPU9255_PWR_MGMT_1, 0x00);
    delay_ms(device->timer, 100);

    i2c_sendbyte(device, MPU9255_PWR_MGMT_1, 0x01);
    delay_ms(device->timer, 200);

    mpu9255_config_reg.dlpf_cfg = 0b11;
    i2c_sendbyte(device, MPU9255_CONFIG, mpu9255_config_reg.raw);

    i2c_sendbyte(device, MPU9255_SMPLRT_DIV, 0x04);

    mpu9255_gyro_config_reg.raw = i2c_readbyte(device, MPU9255_GYRO_CONFIG);
    // Clear Fchoice bits
    mpu9255_gyro_config_reg.fchoice_b = 0b00;
    // Clear AFS bits
    mpu9255_gyro_config_reg.gyro_fs_sel = 0b00;
    i2c_sendbyte(device, MPU9255_GYRO_CONFIG, mpu9255_gyro_config_reg.raw);

    mpu9255_accelerometer_config_reg.raw = i2c_readbyte(device, MPU9255_ACCEL_CONFIG);
    mpu9255_accelerometer_config_reg.accel_fs_sel = 0b00;
    i2c_sendbyte(device, MPU9255_ACCEL_CONFIG, mpu9255_accelerometer_config_reg.raw);

    mpu9255_accelerometer_config2_reg.raw = i2c_readbyte(device, MPU9255_ACCEL_CONFIG2);
    mpu9255_accelerometer_config2_reg.accel_fchoice_b = 0b0;
    mpu9255_accelerometer_config2_reg.A_DLPFCFG = 0b000;
    i2c_sendbyte(device, MPU9255_ACCEL_CONFIG2, mpu9255_accelerometer_config2_reg.raw);

    i2c_sendbyte(device, MPU9255_INT_PIN_CFG, 0x22);
    i2c_sendbyte(device, MPU9255_INT_ENABLE, 0x01);

    delay_ms(device->timer, 200);
}

int16_t *mpu9255_read_accelerometer_data(const i2c_device *device) {
    uint8_t *raw_accelerometer_data = i2c_readbytes(device, MPU9255_ACCEL_XOUT_H, 6);
    int16_t *accelerometer_data = calloc(3, sizeof(uint16_t));

    for (uint8_t i = 0; i < 3; ++i) {
        accelerometer_data[i] += raw_accelerometer_data[2 * i] << 8 | \
                                 raw_accelerometer_data[2 * i + 1];
    }

    free(raw_accelerometer_data);
    return accelerometer_data;
}

int16_t *mpu9255_read_gyroscope_data(const i2c_device *device) {
    uint8_t *raw_gyroscope_data = i2c_readbytes(device, MPU9255_GYRO_XOUT_H, 6);
    int16_t *gyroscope_data = calloc(3, sizeof(uint16_t));
    for (uint8_t i = 0; i < 3; ++i) {
        gyroscope_data[i] += raw_gyroscope_data[2 * i] << 8 | \
                             raw_gyroscope_data[2 * i + 1];
    }
    free(raw_gyroscope_data);
    return gyroscope_data;
}

float mpu9255_read_temperature(const i2c_device *device) {
    static uint16_t raw_temp;

    uint8_t *raw_temp_data = i2c_readbytes(device, MPU9255_TEMP_OUT_H, 2);
    raw_temp = raw_temp_data[0] << 8 | raw_temp_data[1];
    free(raw_temp_data);
    return raw_temp / 333.87f + 21.0f;
}

i2c_device *ak8963_init(I2C_TypeDef *I2Cx, TIM_TypeDef *timer) {
    i2c_device *dev = malloc(sizeof(i2c_device));
    dev->i2c = I2Cx;
    dev->address = AK8963_ADDRESS;
    dev->timer = timer;
    return dev;
}

bool ak8963_check_presence(const i2c_device *device) {
    return i2c_readbyte(device, AK8963_WIA) == 0x48;
}

int16_t *ak8963_read_magnetometer_raw(const i2c_device *device) {
    uint8_t *data = i2c_readbytes(device, AK8963_HXL, 7);
    int16_t *raw_data = calloc(3, sizeof(float));
    raw_data[0] = data[1] << 8 | data[0];
    raw_data[1] = data[3] << 8 | data[2];
    raw_data[2] = data[5] << 8 | data[4];
    free(data);
    return raw_data;
}

bool ak8963_self_test(const i2c_device *device, const ak8963_settings *settings) {
    // 1) Power down device
    ak8963_control1_reg.raw = 0x00;
    ak8963_control1_reg.bit = (uint8_t) settings->precision;
    i2c_sendbyte(device, AK8963_CNTL1, ak8963_control1_reg.raw);

    // 2) Write 1 to SELF in ASTC register
    ak8963_self_test_reg.self = 1;
    i2c_sendbyte(device, AK8963_ASTC, ak8963_self_test_reg.raw);

    // 3) Set self-test mode
    ak8963_control1_reg.mode = 0b1000;
    i2c_sendbyte(device, AK8963_CNTL1, ak8963_control1_reg.raw);

    // 4) Check for Data Ready
    do {
        ak8963_status1_reg.raw = i2c_readbyte(device, AK8963_ST1);
    } while (ak8963_status1_reg.drdy != 1);

    // 5) Read measurements
    int16_t *data = ak8963_read_magnetometer_raw(device);
    int16_t x = data[0], y = data[1], z = data[2];
    free(data);

    // 6) Write 0 to SELF bit in ASTC
    ak8963_self_test_reg.self = 0;
    i2c_sendbyte(device, AK8963_ASTC, ak8963_self_test_reg.raw);

    // 7) Set Power-Down mode
    ak8963_control1_reg.mode = 0b0000;
    i2c_sendbyte(device, AK8963_CNTL1, ak8963_control1_reg.raw);

    // Check if measurements are correct
    if (settings->precision) {
        // 16-bit values
        if ((x <= 200 && x >= -200) &&
            (y <= 200 && y >= -200) &&
            (z <= -800 && x >= -3200)) {
            return true;
        }
    } else {
        // 14-bit values
        if ((x <= 50 && x >= -50) &&
            (y <= 50 && y >= -50) &&
            (z <= -200 && x >= -800)) {
            return true;
        }
    }
    hacf();
    return false;
}

float *ak8963_read_magnetometer_data(const i2c_device *device,
                                     const ak8963_settings *settings) {
    int16_t *data = ak8963_read_magnetometer_raw(device);
    float lsb = settings->precision ? 0.15f : 0.6f;

    float *readings = calloc(3, sizeof(float));
    readings[0] = data[0] * lsb;
    readings[1] = data[1] * lsb;
    readings[2] = data[2] * lsb;
    free(data);
    return readings;
}

void ak8963_configure(const i2c_device *device, const ak8963_settings *settings) {
    // Restart device
    ak8963_control2_reg.raw = 0x00;
    ak8963_control2_reg.srst = 1;
    i2c_sendbyte(device, AK8963_CNTL2, ak8963_control2_reg.raw);
    do {
        print_variable_hex(ak8963_control2_reg.raw);
        ak8963_control2_reg.raw = i2c_readbyte(device, AK8963_CNTL2);
    } while (ak8963_control2_reg.srst != 0);

    // Set mode to continuous measurement with provided precision
    ak8963_control1_reg.raw = 0x00;
    ak8963_control1_reg.bit = (uint8_t) settings->precision;
    ak8963_control1_reg.mode = 0b0110;
    print_variable_hex(ak8963_control1_reg.raw);
    i2c_sendbyte(device, AK8963_CNTL1, ak8963_control1_reg.raw);
}
