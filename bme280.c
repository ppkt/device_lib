#include "bme280.h"

static uint8_t tx[2];
static uint8_t rx[8];

// Returns `true` when sensor is available, `false` otherwise
bool bme280_check_presence(bme280_device* device) {

    tx[0] = id;
    I2C_Master_BufferWrite(device->i2c_bus, tx, 1, Polling,
                           device->address << 1);
    I2C_Master_BufferRead(device->i2c_bus, rx, 1, Polling,
                          device->address << 1);
    usart_printf(USART1, "Device addres %X\r\n", rx[0]);

    return rx[0] == 0x60;
}

// Perform soft reset
void bme280_reset(bme280_device* device) {
    tx[0] = reset;
    tx[1] = 0xB6;

    I2C_Master_BufferWrite(device->i2c_bus, tx, 2, Polling,
                           device->address << 1);
}

// 0. Initialization of device (prepare structure, check presence, soft reset)
bme280_device* bme280_init(I2C_TypeDef* i2c) {
    I2C_LowLevel_Init(i2c);

    bme280_device* device = malloc(sizeof(bme280_device));
    device->i2c_bus = i2c;
    device->address = 0x76;
    device->humidity_oversampling = disable;
    device->pressure_oversampling = disable;
    device->temperature_oversampling = disable;
    device->mode = sleep;
    device->filter_coefficient = off;
    device->standby_duration = t_0_5;

    device->compensation_data = malloc(sizeof(bme280_compensation_data));
    device->raw_reading = malloc(sizeof(bme280_raw_reading));
    device->reading = malloc(sizeof(bme280_reading));

    bme280_reset(device);
    usart_printf(USART1, "Reset completed\r\n");
    if (!bme280_check_presence(device)) {
        hacf();
    }
    usart_printf(USART1, "Device is present\r\n");

    return device;
}

// Write all changes from structure to device
void bme280_commit(bme280_device *device)
{
    // Set humidity oversampling
    tx[0] = ctrl_hum;
    tx[1] = device->humidity_oversampling & 0b111;
    usart_printf(USART1, "CTRL_HUM set to 0x%2X\r\n", tx[1]);
    I2C_Master_BufferWrite(device->i2c_bus, tx, 2, Polling,
                           device->address << 1);

    // Set mode, pressure and temperature oversampling
    tx[0] = ctrl_meas;
    tx[1] = (device->mode & 0b11) |
            (device->pressure_oversampling & 0b111) << 2 |
            (device->temperature_oversampling & 0b111) << 5;
    usart_printf(USART1, "CTRL_MEAS set to 0x%2X\r\n", tx[1]);
    I2C_Master_BufferWrite(device->i2c_bus, tx, 2, Polling,
                           device->address << 1);

    // Set config
    tx[0] = config;
    tx[1] = (device->filter_coefficient & 0b111) << 2 |
            (device->standby_duration & 0b111) << 5;
    usart_printf(USART1, "CONFIG set to 0x%2X\r\n", tx[1]);
    I2C_Master_BufferWrite(device->i2c_bus, tx, 2, Polling,
                           device->address << 1);
}

// 1. Loads compensation data from device to structure
void bme280_load_compensation_data(bme280_device *device) {
    uint8_t _rx[25] = {0x00, };
    tx[0] = calib00;
    // Read first part
    I2C_Master_BufferWrite(device->i2c_bus, tx, 1, Polling,
                           device->address << 1);
    I2C_Master_BufferRead(device->i2c_bus, _rx, 25, DMA,
                          device->address << 1);

    device->compensation_data->T1 = _rx[0] | _rx[1] << 8;
    device->compensation_data->T2 = _rx[2] | _rx[3] << 8;
    device->compensation_data->T3 = _rx[4] | _rx[5] << 8;

    device->compensation_data->P1 = _rx[6] | _rx[7] << 8;
    device->compensation_data->P2 = _rx[8] | _rx[9] << 8;
    device->compensation_data->P3 = _rx[10] | _rx[11] << 8;
    device->compensation_data->P4 = _rx[12] | _rx[13] << 8;
    device->compensation_data->P5 = _rx[14] | _rx[15] << 8;
    device->compensation_data->P6 = _rx[16] | _rx[17] << 8;
    device->compensation_data->P7 = _rx[18] | _rx[19] << 8;
    device->compensation_data->P8 = _rx[20] | _rx[21] << 8;
    device->compensation_data->P9 = _rx[22] | _rx[23] << 8;

    device->compensation_data->H1 = _rx[24];

    // Read second part
    tx[0] = calib26;
    I2C_Master_BufferWrite(device->i2c_bus, tx, 1, Polling,
                           device->address << 1);
    I2C_Master_BufferRead(device->i2c_bus, _rx, 7, DMA,
                          device->address << 1);

    usart_printf(USART1, "%x %x %x %x %x %x\r\n", _rx[0], _rx[1], _rx[2],
            _rx[3], _rx[4], _rx[5], _rx[6]);

    device->compensation_data->H2 = _rx[0] | _rx[1] << 8;
    device->compensation_data->H3 = _rx[2];
    device->compensation_data->H4 = _rx[3] << 4 | (_rx[4] & 0b00001111);
    device->compensation_data->H5 =((_rx[4] & 0b11110000) | _rx[5] << 8) >> 4;
    device->compensation_data->H6 = _rx[6];
}

// 2. Read uncompensated data for pressure and tempearture
// if `humidity` is set to `true`, also this value is fetched
void bme280_read(bme280_device *device, bool humidity)
{
    tx[0] = press;
    I2C_Master_BufferWrite(device->i2c_bus, tx, 1, Polling,
                           device->address << 1);

    uint8_t bytes_to_read = 6;

    if (humidity)
        bytes_to_read += 2;

    I2C_Master_BufferRead(device->i2c_bus, rx, bytes_to_read, Polling,
                          device->address << 1);

    device->raw_reading->pressure = (rx[2] | rx[1] << 8 | rx[0] << 16) >> 4;
    device->raw_reading->temperature = (rx[5] | rx[4] << 8 | rx[3] << 16) >> 4;
    device->raw_reading->humidity = rx[7] | rx[6] << 8;
}

// 3. Using uncompensated data and calibration registers calculate real values
void bme280_calibrated_read(bme280_device *device)
{
    int32_t temperature = (int32_t)device->raw_reading->temperature;
    int32_t var1, var2, tmp, T;
    static int32_t t_fine;
    bme280_compensation_data *cd = device->compensation_data;

    // compensate temperature
    var1 = ((((temperature >> 3) - (cd->T1 << 1))) * cd->T2) >> 11;
    tmp = (temperature >> 4) - cd->T1;
    var2 = (((tmp * tmp) >> 12) * cd->T3) >> 14;
    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;

    // compensate pressure
    uint32_t p, raw_p;
    raw_p = device->raw_reading->pressure;

    var1 = (t_fine >> 1) - 64000;
    tmp = (var1 >> 2) * (var1 >> 2);
    var2 = ((tmp) >> 11) * cd->P6;
    var2 = var2 + ((var1 * cd->P5) << 1);
    var2 = (var2 >> 2) + (cd->P4 << 16);
    var1 = (((cd->P3 * (tmp >> 13)) >> 3) + ((cd->P2 * var1) >> 1)) >> 18;
    var1 = ((32768 + var1) * cd->P1) >> 15;

    if (var1 == 0) {
        hacf(); // avoid exception caused by division by zero
    }
    p = ((1048576 - raw_p) - (var2 >> 12)) * 3125;
    if (p < 0x80000000) {
        p = (p << 1) / var1;
    } else {
        p = (p / var1) * 2;
    }

    var1 = (cd->P9 * (((p >> 3) * (p >> 3)) >> 13)) >> 12;
    var2 = (((int32_t)(p >> 2)) * cd->P8) >> 13;
    p = p + ((var1 + var2 + cd->P7) >> 4);

    // compensate humidity
    int32_t raw_h = device->raw_reading->humidity;
    var1 = t_fine - 76800;

    var1 = (((((raw_h << 14) -
               (cd->H4 << 20) - (cd->H5 * var1)) + 16384) >> 15) *
            (((((((var1 * cd->H6) >> 10) *
                 (((var1 * cd->H3) >> 11) + 32768)) >> 10) + 2097152) *
              cd->H2 + 8192) >> 14));

    var1 -= ((((var1 >> 15) * (var1 >> 15)) >> 7) * (cd->H1)) >> 4;

    var1 = var1 < 0 ? 0 : var1;
    var1 = var1 > 419430400 ? 419430400 : var1;

    device->reading->temperature = T/100.0;
    device->reading->pressure = p;
    device->reading->humidity = (var1 >> 12) / 1024.0;
}
