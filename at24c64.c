#include "at24c64.h"

static uint8_t *tx;

uint8_t at24c64_write_bytes(uint16_t address, uint8_t *data, uint16_t n,
                            TIM_TypeDef *timer) {

    uint8_t total_pages =  n / 32 + 1;

    // 32 bytes for max payload + 2 for memory address
    tx = (uint8_t*)malloc((32 + 2) * sizeof(uint8_t));

    for (uint8_t page_id = 0; page_id < total_pages; ++page_id) {
        uint8_t bytes_to_write = min(n - page_id * 32, 32);

        if (bytes_to_write == 0)
            break;

        tx[0] = address << 8;
        tx[1] = 0x00FF & address;

        memcpy(&tx[2], data + (page_id * 32), bytes_to_write);
        I2C_Master_BufferWrite(
                    I2C1, tx, bytes_to_write + 2, DMA, AT24C64_ADDRESS << 1);

        // increase address for subsequent loop
        address += bytes_to_write;

        // Note: After writing any data to module, we have to wait t_WR until
        // read will be possible (5 ms). ACK Polling is allowed
        // (check datasheet)
        delay_ms(timer, 5);
    }

    free(tx);

    return 0;
}



uint8_t at24c64_read_bytes(uint16_t address, uint8_t *data, uint16_t n) {
    uint8_t total_pages =  n / 32 + 1;

    // 2 bytes for memory address
    tx = (uint8_t*)malloc(2 * sizeof(uint8_t));

    for (uint8_t page_id = 0; page_id < total_pages; ++page_id) {
        uint8_t bytes_to_read = min(n - page_id * 32, 32);

        if (bytes_to_read == 0)
            break;

        tx[0] = address << 8;
        tx[1] = 0x00FF & address;

        // I guess there is a bug in ST I2C library and when you're trying to
        // **read** single byte using DMA it's not correctly handled, so we have
        // to use polling in this case
        static I2C_ProgrammingModel model;
        if (bytes_to_read == 1)
            model = Polling;
        else
            model = DMA;
        I2C_Master_BufferWrite(I2C1, tx, 2, DMA, AT24C64_ADDRESS << 1);
        I2C_Master_BufferRead(I2C1, data + (page_id * 32), bytes_to_read, model,
                              AT24C64_ADDRESS << 1);


        // increase address for subsequent loop
        address += bytes_to_read;

    }

    free(tx);

    return 0;
}
