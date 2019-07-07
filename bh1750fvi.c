#include "bh1750fvi.h"

static error_t send_command(const i2c_device *dev, uint8_t command) {
  if (!dev) {
    return E_NULL_PTR;
  }
  uint8_t tx[] = {command};
  i2c_transfer7(dev->i2c, dev->address, tx, sizeof_a(tx), NULL, 0);
  return E_SUCCESS;
}

error_t bh1750fvi_init(uint32_t i2c, i2c_device *dev) {
  if (!dev) {
    return E_NULL_PTR;
  }

  dev->i2c = i2c;
  dev->address = BH1750FVI_ADDRESS;

  if (!i2c_check_presence(dev->i2c, dev->address)) {
    hacf();
  }

  send_command(dev, BH1750FVI_POWER_ON_CMD);
  send_command(dev, BH1750FVI_RESET_CMD);
  send_command(dev, BH1750FVI_POWER_ON_CMD);

  return E_SUCCESS;
}
error_t bh1750fvi_read_single_shot(const i2c_device *dev,
                                   bh1750fvi_resolution resolution,
                                   float *val) {
  if (!dev || !val) {
    return E_NULL_PTR;
  }

  // turn on device
  send_command(dev, BH1750FVI_POWER_ON_CMD);

  // send command to perform reading and wait for result
  switch (resolution) {
  case BH1750FVI_LOW_RESOLUTION:
    send_command(dev, BH1750FVI_SINGLE_SHOT_LRES_CMD);
    delay_ms(24);
    break;
  case BH1750FVI_HIGH_RESOLUTION:
    send_command(dev, BH1750FVI_SINGLE_SHOT_HRES_CMD);
    delay_ms(180);
    break;
  case BH1750FVI_HIGH_RESOLUTION_2:
    send_command(dev, BH1750FVI_SINGLE_SHOT_HRES2_CMD);
    delay_ms(180);
    break;
  default:
    hacf();
  }

  // read result
  uint8_t rx[2];
  i2c_transfer7(dev->i2c, dev->address, NULL, 0, rx, sizeof_a(rx));
  uint16_t reading = (rx[0] << 8u) + rx[1];
  *val = reading / 1.2;

  return E_SUCCESS;
}
