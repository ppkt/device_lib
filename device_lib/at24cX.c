#include "at24cX.h"

inline uint8_t *get_tx_buffer(uint16_t address, uint8_t extra_size);

uint8_t *
get_tx_buffer(uint16_t address, uint8_t extra_size) {
    uint8_t *tx = malloc((2 + extra_size) * sizeof(uint8_t));
    tx[0] = (uint8_t) (address >> 8);
    tx[1] = (uint8_t) (address & 0xFF);
    return tx;
}

i2c_device *
at24cX_init_address(uint32_t i2c, uint8_t address) {
    i2c_device *dev = malloc(sizeof(i2c_device));
    dev->i2c = i2c;
    dev->address = address;

    if (!i2c_check_presence(dev->i2c, dev->address)) {
        hacf();
    }

    return dev;
}

inline i2c_device *
at24cX_init(uint32_t i2c) {
    return at24cX_init_address(i2c, AT24CX_ADDRESS);
}

uint8_t *
at24cX_random_read_bytes(const i2c_device *dev, uint16_t address,
                         uint16_t bytes) {
    uint8_t *tx = get_tx_buffer(address, 0);
    uint8_t *rx = malloc(bytes * sizeof(uint8_t));

    i2c_master_transaction_write_read(dev->i2c, dev->address, tx, 2, rx, bytes);
    free(tx);
    return rx;
}

uint8_t
at24cX_random_read_byte(const i2c_device *dev, uint16_t address) {
    uint8_t *rx = at24cX_random_read_bytes(dev, address, 1);
    uint8_t ret = rx[0];
    free(rx);
    return ret;
}

uint8_t *
at24cX_current_address_read_bytes(const i2c_device *dev, uint16_t bytes) {
    uint8_t *rx = malloc(bytes * sizeof(uint8_t));
    i2c_transfer7(dev->i2c, dev->address, NULL, 0, rx, bytes);
    return rx;
}

uint8_t
at24cX_current_address_read_byte(const i2c_device *dev) {
    uint8_t *rx = at24cX_current_address_read_bytes(dev, 1);
    uint8_t ret = rx[0];
    free(rx);
    return ret;
}

void
at24cX_set_address(const i2c_device *dev, uint16_t address) {
    uint8_t *tx = get_tx_buffer(address, 0);

    i2c_master_transaction_write_read(dev->i2c, dev->address, tx, 2, NULL, 0);
    free(tx);
}

void
at24cX_page_write_bytes(const i2c_device *dev, uint16_t address,
                        const uint8_t *_tx, uint8_t bytes) {
    uint8_t *tx = get_tx_buffer(address, bytes);
    memcpy(&tx[2], _tx, bytes);

    i2c_master_write(dev->i2c, dev->address, tx, bytes + 2);
    free(tx);

    delay_ms(10);
}

void
at24cX_write_byte(const i2c_device *dev, uint16_t address, uint8_t byte) {
    uint8_t tx[] = {byte};
    at24cX_page_write_bytes(dev, address, tx, 1);
}
