#include "bme280.h"

static uint8_t tx[2];
static uint8_t rx[8];

// Returns `true` when sensor is available, `false` otherwise
static bool bme280_check_presence(bme280_device *device) {

  tx[0] = id;
  i2c_transfer7(device->i2c_dev.i2c, device->i2c_dev.address, tx, 1, rx, 1);
  return rx[0] == 0x60;
}

// Perform soft reset
static void bme280_reset(bme280_device *device) {
  tx[0] = reset;
  tx[1] = 0xB6;

  i2c_transfer7(device->i2c_dev.i2c, device->i2c_dev.address, tx, 2, NULL, 0);
}

// 0. Initialization of device (prepare structure, check presence, soft reset)
error_t bme280_init(uint32_t i2c, bme280_device *device) {
  device->i2c_dev.i2c = i2c;
  device->i2c_dev.address = 0x76;
  device->humidity_oversampling = disable;
  device->pressure_oversampling = disable;
  device->temperature_oversampling = disable;
  device->mode = sleep;
  device->filter_coefficient = off;
  device->standby_duration = t_0_5;

  bme280_reset(device);
  usart_printf(USART1, "Reset completed\r\n");
  if (!bme280_check_presence(device)) {
    hacf();
  }
  usart_printf(USART1, "Device is present\r\n");

  return E_SUCCESS;
}

// Write all changes from structure to device
error_t bme280_commit(bme280_device *device) {
  // Set humidity oversampling
  tx[0] = ctrl_hum;
  tx[1] = device->humidity_oversampling & 0b111;
  usart1_print("CTRL_HUM: ");
  print_variable_hex(tx[1]);
  i2c_transfer7(device->i2c_dev.i2c, device->i2c_dev.address, tx, 2, NULL, 0);

  // Set mode, pressure and temperature oversampling
  tx[0] = ctrl_meas;
  tx[1] = (device->mode & 0b11u) |
          (device->pressure_oversampling & 0b111u) << 2u |
          (device->temperature_oversampling & 0b111u) << 5u;
  usart1_print("CTRL_MEAS: ");
  print_variable_hex(tx[1]);
  i2c_transfer7(device->i2c_dev.i2c, device->i2c_dev.address, tx, 2, NULL, 0);

  // Set config
  tx[0] = config;
  tx[1] = (device->filter_coefficient & 0b111u) << 2u |
          (device->standby_duration & 0b111u) << 5u;
  usart1_print("CONFIG: ");
  print_variable_hex(tx[1]);
  i2c_transfer7(device->i2c_dev.i2c, device->i2c_dev.address, tx, 2, NULL, 0);
  return E_SUCCESS;
}

// 1. Loads compensation data from device to structure
error_t bme280_load_compensation_data(bme280_device *device) {
  uint8_t _rx[25] = {0x00};
  tx[0] = calib00;
  // Read first part
  i2c_transfer7(device->i2c_dev.i2c, device->i2c_dev.address, tx, 1, _rx, 25);

  device->compensation_data.T1 = _rx[0] | _rx[1] << 8;
  device->compensation_data.T2 = _rx[2] | _rx[3] << 8;
  device->compensation_data.T3 = _rx[4] | _rx[5] << 8;

  device->compensation_data.P1 = _rx[6] | _rx[7] << 8;
  device->compensation_data.P2 = _rx[8] | _rx[9] << 8;
  device->compensation_data.P3 = _rx[10] | _rx[11] << 8;
  device->compensation_data.P4 = _rx[12] | _rx[13] << 8;
  device->compensation_data.P5 = _rx[14] | _rx[15] << 8;
  device->compensation_data.P6 = _rx[16] | _rx[17] << 8;
  device->compensation_data.P7 = _rx[18] | _rx[19] << 8;
  device->compensation_data.P8 = _rx[20] | _rx[21] << 8;
  device->compensation_data.P9 = _rx[22] | _rx[23] << 8;

  device->compensation_data.H1 = _rx[24];

  // Read second part
  tx[0] = calib26;
  i2c_transfer7(device->i2c_dev.i2c, device->i2c_dev.address, tx, 1, _rx, 7);

  usart1_printf("%x %x %x %x %x %x\r\n", _rx[0], _rx[1], _rx[2], _rx[3], _rx[4],
                _rx[5], _rx[6]);

  device->compensation_data.H2 = _rx[0] | _rx[1] << 8;
  device->compensation_data.H3 = _rx[2];
  device->compensation_data.H4 = _rx[3] << 4 | (_rx[4] & 0b00001111);
  device->compensation_data.H5 = ((_rx[4] & 0b11110000) | _rx[5] << 8) >> 4;
  device->compensation_data.H6 = _rx[6];

  return E_SUCCESS;
}

// 2. Read uncompensated data for pressure and tempearture
// if `humidity` is set to `true`, also this value is fetched
error_t bme280_read(bme280_device *device, bool humidity) {
  tx[0] = press;

  uint8_t bytes_to_read = 6;

  if (humidity)
    bytes_to_read += 2;

  i2c_transfer7(device->i2c_dev.i2c, device->i2c_dev.address, tx, 1, rx,
                bytes_to_read);

  device->raw_reading.pressure = (rx[2] + (rx[1] << 8) + (rx[0] << 16)) >> 4;
  device->raw_reading.temperature = (rx[5] | rx[4] << 8 | rx[3] << 16) >> 4;
  device->raw_reading.humidity = rx[7] | rx[6] << 8;

  return E_SUCCESS;
}

// 3. Using uncompensated data and calibration registers calculate real values
error_t bme280_calibrated_read(bme280_device *device) {
  int32_t temperature = (int32_t)device->raw_reading.temperature;
  int32_t var1, var2, tmp, T;
  static int32_t t_fine;
  bme280_compensation_data *cd = &device->compensation_data;

  // compensate temperature
  var1 = ((((temperature >> 3) - (cd->T1 << 1))) * cd->T2) >> 11;
  tmp = (temperature >> 4) - cd->T1;
  var2 = (((tmp * tmp) >> 12) * cd->T3) >> 14;
  t_fine = var1 + var2;
  T = (t_fine * 5 + 128) >> 8;

  // compensate pressure
  uint32_t p, raw_p;
  raw_p = device->raw_reading.pressure;

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
  int32_t raw_h = device->raw_reading.humidity;
  var1 = t_fine - 76800;

  var1 = (((((raw_h << 14) - (cd->H4 << 20) - (cd->H5 * var1)) + 16384) >> 15) *
          (((((((var1 * cd->H6) >> 10) * (((var1 * cd->H3) >> 11) + 32768)) >>
              10) +
             2097152) *
                cd->H2 +
            8192) >>
           14));

  var1 -= ((((var1 >> 15) * (var1 >> 15)) >> 7) * (cd->H1)) >> 4;

  var1 = var1 < 0 ? 0 : var1;
  var1 = var1 > 419430400 ? 419430400 : var1;

  device->reading.temperature = T / 100.0;
  device->reading.pressure = p;
  device->reading.humidity = (var1 >> 12) / 1024.0;

  return E_SUCCESS;
}
