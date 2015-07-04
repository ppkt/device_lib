#include "rf24l01.h"

void rf24l01_init(void) {
    // Initialize SPI1
    spi_init();
}

u8 rf24l01_status(void) {
    u8 tx[1] = {0xFF}; // NOP
    u8 rx[1] = {0x00};

    spi_send(tx, rx, 1);
    return rx[0];
}
