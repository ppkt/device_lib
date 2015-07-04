#include "rf24l01.h"

u8 tx[32];
u8 rx[32];

void rf24l01_gpio_init() {
    GPIO_InitTypeDef GPIO_InitStructure;

    // Configure RF24 pins: CE, IRQ
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);


    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    rf24l01_ce(0);

}

void rf24l01_ce(bool new_state) {
    if (new_state) {
        GPIO_WriteBit(GPIOA, GPIO_Pin_2, Bit_SET);
    } else {
        GPIO_WriteBit(GPIOA, GPIO_Pin_2, Bit_RESET);
    }
}

void rf24l01_init(bool receiver) {
    // Initialize SPI1
    spi_init();

    // Prepare timer
    setup_delay_timer(TIM3);

    u8 init_buffer[5] = {0x00};

    init_buffer[0] = 0x01;
    rf24l01_write_register(EN_AA, init_buffer, 1); // enable auto-ack on pipe 0

    init_buffer[0] = 0x01;
    rf24l01_write_register(EN_RXADDR, init_buffer, 1); // enable data pipe 0

    init_buffer[0] = 0x03;
    rf24l01_write_register(SETUP_AW, init_buffer, 1); // 5 bytes RF address

    init_buffer[0] = 0x2F;
    rf24l01_write_register(SETUP_RETR, init_buffer, 1); // 750us retransmission delay, up to 15 retransmissions

    init_buffer[0] = 0x01;
    rf24l01_write_register(RF_CH, init_buffer, 1); // 2.401 GHz

    init_buffer[0] = 0x07;
    rf24l01_write_register(RF_SETUP, init_buffer, 1); // 1Mbps, -0dB

    // set unique address of device
    u8 *ptr = (uint8_t*)U_ID_PTR;
    init_buffer[0] = *(ptr++);
    init_buffer[1] = *(ptr++);
    init_buffer[2] = *(ptr++);
    init_buffer[3] = *(ptr++);
    init_buffer[4] = *(ptr++);
    printf("Device address: %02X %02X %02X %02X %02X\n\r", init_buffer[0], init_buffer[1],
            init_buffer[2], init_buffer[3], init_buffer[4]);
    rf24l01_write_register(RX_ADDR_P0, init_buffer, 5);
    rf24l01_write_register(TX_ADDR, init_buffer, 5); // must be the same if EN_AA is enabled
    rf24l01_read_register(RX_ADDR_P0, init_buffer, 5);

    init_buffer[0] = 5;
    rf24l01_write_register(RX_PW_P0, init_buffer, 1); // set payload size to 5 bytes

    init_buffer[0] = 0x0E;
    if (receiver) {
        init_buffer[0] |= 0x01;
    }
    rf24l01_write_register(CONFIG, init_buffer, 1); // enable crc, use 2 bytes, power up


    GPIO_WriteBit(GPIOA, GPIO_Pin_1, Bit_RESET);
    GPIO_WriteBit(GPIOA, GPIO_Pin_1, Bit_SET);
    delay_us(TIM3, 150);
    GPIO_WriteBit(GPIOA, GPIO_Pin_1, Bit_RESET);
    GPIO_WriteBit(GPIOA, GPIO_Pin_1, Bit_SET);
}

u8 rf24l01_status(void) {
    tx[0] = NOP;

    spi_send(tx, rx, 1);
    return rx[0];
}

void rf24l01_write_register(u8 addr, u8 *tx_buffer, u8 bytes) {
    u16 j;
    for (j = 0; j < 200; ++j) {}
    tx[0] = W_REGISTER | addr;
    memcpy(tx+1, tx_buffer, bytes);


    spi_send(tx, rx, bytes + 1);
}

// Reads `bytes` bytes from `addr` to `rx_buffer`. Returns Status register
u8 rf24l01_read_register(u8 addr, u8 *rx_buffer, u8 bytes) {
    u16 j;
    for (j = 0; j < 200; ++j) {}
    memset(tx, 0, bytes + 1);
    tx[0] = R_REGISTER | addr;
    spi_send(tx, rx_buffer, bytes + 1);
    return rx_buffer[0];
}
