#include "ds1307.h"

static uint8_t tx[1];

i2c_device *ds1307_init(uint32_t i2c) {
  i2c_device *dev = malloc(sizeof(i2c_device));
  dev->i2c = i2c;
  dev->address = DS1307_ADDRESS;

  if (!i2c_check_presence(dev->i2c, dev->address)) {
    hacf();
  }
  return dev;
}

struct tm ds1307_read_date_raw(const i2c_device *dev, bool convert_bcm_to_dec) {
  return ds_read_date_raw(dev, convert_bcm_to_dec, DS1307_SECONDS_REG);
}

void ds1307_set_date(const i2c_device *dev, const struct tm *new_time) {
  ds_set_date(dev, new_time, DS1307_SECONDS_REG);
}

ds1307_control_register_s ds1307_read_control_register(const i2c_device *dev) {
  tx[0] = DS1307_CONTROL_REG;
  ds1307_control_register_s ds1307_control_register;
  i2c_master_transaction_write_read(dev->i2c, dev->address, tx, 1,
                                    &ds1307_control_register.raw, 1);
  return ds1307_control_register;
}

uint8_t *ds1307_read_nvram(const i2c_device *dev, uint8_t offset,
                           uint8_t bytes) {

  if (offset >= DS1307_NVRAM_SIZE || bytes == 0) {
    return NULL;
  }

  uint8_t bytes_to_read = (uint8_t)min(bytes, DS1307_NVRAM_SIZE - offset);
  uint8_t *buffer = calloc(bytes, sizeof(uint8_t));

  tx[0] = (uint8_t)DS1307_NVRAM_REG + offset;
  i2c_master_transaction_write_read(dev->i2c, dev->address, tx, 1, buffer,
                                    bytes_to_read);

  return buffer;
}

void ds1307_write_nvram(const i2c_device *dev, uint8_t offset, uint8_t *_tx,
                        uint8_t tx_len) {
  if (offset >= DS1307_NVRAM_SIZE || tx_len == 0) {
    return;
  }

  uint8_t *buffer = malloc((tx_len + 1) * sizeof(uint8_t));
  buffer[0] = (uint8_t)DS1307_NVRAM_REG + offset;
  memcpy(&buffer[1], _tx, tx_len);
  uint8_t bytes_to_write = (uint8_t)min(tx_len, DS1307_NVRAM_SIZE - offset);
  i2c_transfer7(dev->i2c, dev->address, buffer, bytes_to_write + 1, NULL, 0);
  free(buffer);
}

void ds1307_erase_nvram(const i2c_device *dev) {
  uint8_t *buffer = calloc(DS1307_NVRAM_SIZE + 1, sizeof(uint8_t));
  buffer[0] = (uint8_t)DS1307_NVRAM_REG;
  i2c_transfer7(dev->i2c, dev->address, buffer, DS1307_NVRAM_SIZE + 1, NULL, 0);
  free(buffer);
}
