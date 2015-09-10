#include "at24c64.h"

static uint8_t *tx;
//static uint8_t *rx;

uint8_t at24c64_write_bytes(uint16_t address, uint8_t *data, uint16_t n) {
    tx = (uint8_t*)malloc((n + 2) * sizeof(uint8_t));

    tx[0] = address << 8;
    tx[1] = 0x00FF & address;
    memcpy(&tx[2], data, n);

    I2C_Master_BufferWrite(I2C1, tx, n + 2, Polling, AT24C64_ADDRESS << 1);

    return 0;
}

uint8_t at24c64_read_bytes(uint16_t address, uint8_t *data, uint16_t n) {
    tx[0] = address << 8;
    tx[1] = 0x00FF & address;
    I2C_Master_BufferWrite(I2C1, tx, 2, Polling, AT24C64_ADDRESS << 1);
    I2C_Master_BufferRead(I2C1, data, n, Polling, AT24C64_ADDRESS << 1);

    return 0;
}
