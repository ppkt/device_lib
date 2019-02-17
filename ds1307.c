#include "ds1307.h"

static uint8_t tx[1], rx[7];

i2c_device *
ds1307_init(uint32_t i2c) {
    i2c_device *dev = malloc(sizeof(i2c_device));
    dev->i2c = i2c;
    dev->address = DS1307_ADDRESS;

    if (!i2c_check_presence(dev->i2c, dev->address)) {
        hacf();
    }
    return dev;
}

struct tm
ds1307_read_date(const i2c_device *dev) {
    tx[0] = DS1307_SECONDS_REG;
    i2c_master_transaction_write_read(
            dev->i2c, dev->address, tx, 1, rx, 7);
    struct tm result;
    result.tm_sec = bcd_to_decimal(rx[0]);
    result.tm_min = bcd_to_decimal(rx[1]);
    result.tm_hour = bcd_to_decimal(rx[2] & (uint8_t) 0x3F);
    result.tm_mday = bcd_to_decimal(rx[4]);
    // Month is stored in device in range 1-12, tm is using 0-11
    result.tm_mon = bcd_to_decimal(rx[5]) - 1;
    // Year in device is measured since 2000, tm is using years since 1900
    result.tm_year = bcd_to_decimal(rx[6]) + 100;
    // Fill rest of the struct
    mktime(&result);

    return result;
}

void
ds1307_set_date(const i2c_device *dev, const struct tm *new_time) {
    uint8_t *buffer = calloc(8, sizeof(uint8_t));
    buffer[0] = DS1307_SECONDS_REG;
    buffer[1] = decimal_to_bcd((uint8_t) new_time->tm_sec);
    buffer[2] = decimal_to_bcd((uint8_t) new_time->tm_min);
    buffer[3] = decimal_to_bcd((uint8_t) new_time->tm_hour);
    buffer[5] = decimal_to_bcd((uint8_t) new_time->tm_mday);
    // Month is stored in device in range 1-12, tm is using 0-11
    buffer[6] = decimal_to_bcd((uint8_t) (new_time->tm_mon + 1));
    // Year in device is measured since 2000, tm is using years since 1900
    buffer[7] = decimal_to_bcd((uint8_t) (new_time->tm_year - 100));

    i2c_master_transaction_write_read(
            dev->i2c, dev->address, buffer, 8, NULL, 0);
}

ds1307_control_register_s
ds1307_read_control_register(const i2c_device *dev) {
    tx[0] = DS1307_CONTROL_REG;
    ds1307_control_register_s ds1307_control_register;
    i2c_master_transaction_write_read(dev->i2c, dev->address, tx, 1,
                                      &ds1307_control_register.raw, 1);
    return ds1307_control_register;
}

uint8_t *
ds1307_read_nvram(const i2c_device *dev, uint8_t offset, uint8_t bytes) {

    if (offset >= DS1307_NVRAM_SIZE || bytes == 0) {
        return NULL;
    }

    uint8_t bytes_to_read = (uint8_t) min(bytes,
                                          DS1307_NVRAM_SIZE - offset);
    uint8_t *buffer = calloc(bytes, sizeof(uint8_t));

    tx[0] = (uint8_t) DS1307_NVRAM_REG + offset;
    i2c_master_transaction_write_read(dev->i2c, dev->address, tx, 1,
                                      buffer, bytes_to_read);

    return buffer;
}

void
ds1307_write_nvram(const i2c_device *dev, uint8_t offset,
                   uint8_t *_tx, uint8_t tx_len) {
    if (offset >= DS1307_NVRAM_SIZE || tx_len == 0) {
        return;
    }

    uint8_t *buffer = malloc((tx_len + 1) * sizeof(uint8_t));
    buffer[0] = (uint8_t) DS1307_NVRAM_REG + offset;
    memcpy(&buffer[1], _tx, tx_len);
    uint8_t bytes_to_write = (uint8_t) min(tx_len,
                                           DS1307_NVRAM_SIZE - offset);
    i2c_transfer7(dev->i2c, dev->address, buffer, bytes_to_write + 1, NULL, 0);
    free(buffer);
}

void
ds1307_erase_nvram(const i2c_device *dev) {
    uint8_t *buffer = calloc(DS1307_NVRAM_SIZE + 1, sizeof(uint8_t));
    buffer[0] = (uint8_t) DS1307_NVRAM_REG;
    i2c_transfer7(dev->i2c, dev->address, buffer, DS1307_NVRAM_SIZE + 1,
                  NULL, 0);
    free(buffer);
}
