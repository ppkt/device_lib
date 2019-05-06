#include "ds3231.h"

i2c_device *ds3231_init(uint32_t i2c) {
  i2c_device *dev = malloc(sizeof(i2c_device));
  dev->i2c = i2c;
  dev->address = DS3231_ADDRESS;

  if (!i2c_check_presence(dev->i2c, dev->address)) {
    hacf();
  }
  return dev;
}

struct tm ds3231_read_date_raw(const i2c_device *dev, bool convert_bcm_to_dec) {
  return ds_read_date_raw(dev, convert_bcm_to_dec, DS3231_SECONDS_REG);
}

void ds3231_set_date(const i2c_device *dev, const struct tm *new_time) {
  ds_set_date(dev, new_time, DS3231_SECONDS_REG);
}

float ds3231_get_temperature(const i2c_device *dev) {
  uint8_t tx[0], rx[2];
  tx[0] = DS3231_TEMPERATURE_REG;
  i2c_master_transaction_write_read(dev->i2c, dev->address, tx, 1, rx, 2);
  return (int8_t)rx[0] + (rx[1] >> 6) / 4.0;
}
