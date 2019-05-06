#include "ds_common.h"

struct tm ds_read_date_raw(const i2c_device *dev, bool convert_bcm_to_dec,
                           uint8_t seconds_register) {

  uint8_t (*convert)(uint8_t) = &noop;
  if (convert_bcm_to_dec) {
    convert = &bcd_to_decimal;
  }
  uint8_t tx[1], rx[7];
  tx[0] = seconds_register;
  i2c_master_transaction_write_read(dev->i2c, dev->address, tx, 1, rx, 7);
  struct tm result;
  result.tm_sec = convert(rx[0]);
  result.tm_min = convert(rx[1]);
  result.tm_hour = convert(rx[2] & (uint8_t)0x3F);
  result.tm_mday = convert(rx[4]);
  // Month is stored in device in range 1-12, tm is using 0-11
  result.tm_mon = convert(rx[5] & (uint8_t)0x3F) - 1;
  // Year in device is measured since 2000, tm is using years since 1900
  result.tm_year = convert(rx[6]) + 100;

  // Fill rest of the struct only if struct has correct data
  if (convert_bcm_to_dec) {
    mktime(&result);
  }

  return result;
}

void ds_set_date(const i2c_device *dev, const struct tm *new_time,
                 uint8_t seconds_register) {
  uint8_t *buffer = calloc(8, sizeof(uint8_t));
  buffer[0] = seconds_register;
  buffer[1] = decimal_to_bcd((uint8_t)new_time->tm_sec);
  buffer[2] = decimal_to_bcd((uint8_t)new_time->tm_min);
  buffer[3] = decimal_to_bcd((uint8_t)new_time->tm_hour);
  buffer[5] = decimal_to_bcd((uint8_t)new_time->tm_mday);
  // Month is stored in device in range 1-12, tm is using 0-11
  buffer[6] = decimal_to_bcd((uint8_t)(new_time->tm_mon + 1));
  // Year in device is measured since 2000, tm is using years since 1900
  buffer[7] = decimal_to_bcd((uint8_t)(new_time->tm_year - 100));

  i2c_master_transaction_write_read(dev->i2c, dev->address, buffer, 8, NULL, 0);
}
