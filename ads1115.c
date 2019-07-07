#include "ads1115.h"

// LSB = FSR / 2^16
float lsb[8] = {
    187.5,                  // FSR 6.144 V, LSB = 187.5 uV
    125,                    // FSR 4.096 V
    62.5,                   // FSR 2.048 V
    31.25,                  // FSR 1.024 V
    15.625,                 // FSR 0.512 V
    7.8125, 7.8125, 7.8125, // FSR 0.256 V
};

error_t ads1115_init(uint32_t i2c, i2c_device *dev) {
  if (!dev) {
    return E_NULL_PTR;
  }

  dev->i2c = i2c;
  dev->address = ADS1115_ADDRESS;

  if (!i2c_check_presence(dev->i2c, dev->address)) {
    hacf();
  }

  ads1115_config cfg;
  ads1115_read_config(dev, &cfg);
  cfg.reg.pga = 3;  // FSR = 1.024V
  cfg.reg.mode = 1; // single shot
  cfg.reg.dr = 0;   // 8 samples per second
  ads1115_write_config(dev, &cfg);

  return E_SUCCESS;
}

error_t ads1115_read_config(const i2c_device *dev, ads1115_config *reg) {
  if (!dev || !reg) {
    return E_NULL_PTR;
  }

  uint8_t tx[] = {ADS1115_CONFIG_REG}, rx[2];
  i2c_transfer7(dev->i2c, dev->address, tx, sizeof_a(tx), rx, sizeof_a(rx));
  (*reg).raw = (rx[0] << 8u) + rx[1];

  return E_SUCCESS;
}

error_t ads1115_write_config(const i2c_device *dev, const ads1115_config *reg) {
  if (!dev || !reg) {
    return E_NULL_PTR;
  }

  uint8_t tx[] = {ADS1115_CONFIG_REG, reg->raw >> 8u, reg->raw & 0xFFu};
  i2c_transfer7(dev->i2c, dev->address, tx, sizeof_a(tx), NULL, 0);

  return E_SUCCESS;
}

error_t ads1115_read_single_shot(const i2c_device *dev, float *voltage) {
  if (!dev) {
    return E_NULL_PTR;
  }

  ads1115_config cfg;
  ads1115_read_config(dev, &cfg);
  cfg.reg.os = 1;
  ads1115_write_config(dev, &cfg);
  // wait for conversion end
  while (cfg.reg.os == 1) {
    ads1115_read_config(dev, &cfg);
  }

  uint8_t tx[] = {ADS1115_CONVERSION_REG}, rx[2];
  i2c_transfer7(dev->i2c, dev->address, tx, sizeof_a(tx), rx, sizeof_a(rx));

  int16_t raw = (rx[0] << 8u) + rx[1];

  *voltage = raw * lsb[cfg.reg.pga] / 1000;

  return E_SUCCESS;
}
