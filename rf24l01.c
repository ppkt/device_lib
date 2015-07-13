#include "rf24l01.h"

u8 tx[32];
u8 rx[32];

// Writes 1 on all NSS pins
void rf24l01_deselect_device(rf24l01_context ctx) {
    if (ctx.device_id == 0) {
        GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_SET);
    } else if (ctx.device_id == 1) {
        GPIO_WriteBit(GPIOA, GPIO_Pin_0, Bit_SET);
    }
}

// Select device to SPI transmissions by sending 0 to its NSS
void rf24l01_select_device(rf24l01_context ctx) {
    rf24l01_deselect_device(ctx);

    if (ctx.device_id == 0) {
        GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_RESET);
    } else if (ctx.device_id == 1) {
        GPIO_WriteBit(GPIOA, GPIO_Pin_0, Bit_RESET);
    }
}

void rf24l01_ce(rf24l01_context ctx, bool new_state) {
    if (ctx.device_id == 0) {
        if (new_state) {
            GPIO_WriteBit(GPIOA, GPIO_Pin_2, Bit_SET);
        } else {
            GPIO_WriteBit(GPIOA, GPIO_Pin_2, Bit_RESET);
        }
    } else if (ctx.device_id == 1) {
        if (new_state) {
            GPIO_WriteBit(GPIOA, GPIO_Pin_8, Bit_SET);
        } else {
            GPIO_WriteBit(GPIOA, GPIO_Pin_8, Bit_RESET);
        }
    }
}

void rf24l01_gpio_init(rf24l01_context ctx) {
    GPIO_InitTypeDef GPIO_InitStructure;

    // Configure RF24 pins
    if (ctx.device_id == 0) {
        // CE
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
        GPIO_Init(GPIOA, &GPIO_InitStructure);
        GPIO_WriteBit(GPIOA, GPIO_Pin_2, Bit_SET);

        // IRQ
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
        GPIO_Init(GPIOA, &GPIO_InitStructure);

        // Soft NSS
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
        GPIO_Init(GPIOA, &GPIO_InitStructure);

    } else if (ctx.device_id == 1) {
        // CE
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
        GPIO_Init(GPIOA, &GPIO_InitStructure);
        GPIO_WriteBit(GPIOA, GPIO_Pin_8, Bit_SET);


        // Soft NSS
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
        GPIO_Init(GPIOA, &GPIO_InitStructure);
    }

    rf24l01_deselect_device(ctx);

}

rf24l01_context rf24l01_init(u8 device_id, SPI_TypeDef *spi, TIM_TypeDef *timer, rf24l01_role role) {
    rf24l01_context ctx;
    ctx.spi = spi;
    ctx.timer = timer;
    ctx.device_id = device_id;

    // Initialize SPI1
    spi_init(ctx.spi);

    // Prepare timer
    setup_delay_timer(ctx.timer);

    // SetUp GPIO
    rf24l01_gpio_init(ctx);

    u8 init_buffer[5] = {0x00};

    init_buffer[0] = 0x01;
    rf24l01_write_register(ctx, EN_AA, init_buffer, 1); // enable auto-ack on pipe 0

    init_buffer[0] = 0x01;
    rf24l01_write_register(ctx, EN_RXADDR, init_buffer, 1); // enable data pipe 0

    init_buffer[0] = 0x03;
    rf24l01_write_register(ctx, SETUP_AW, init_buffer, 1); // 5 bytes RF address

    init_buffer[0] = 0x2F;
    rf24l01_write_register(ctx, SETUP_RETR, init_buffer, 1); // 750us retransmission delay, up to 15 retransmissions

    init_buffer[0] = 0x01;
    rf24l01_write_register(ctx, RF_CH, init_buffer, 1); // 2.401 GHz

    init_buffer[0] = 0x07;
    rf24l01_write_register(ctx, RF_SETUP, init_buffer, 1); // 1Mbps, -0dB

    // set unique address of device
    u8 *ptr = (uint8_t*)U_ID_PTR;
    init_buffer[0] = *(ptr++);
    init_buffer[1] = *(ptr++);
    init_buffer[2] = *(ptr++);
    init_buffer[3] = *(ptr++);
    init_buffer[4] = *(ptr++);
//    printf("Device address: %02X %02X %02X %02X %02X\n\r", init_buffer[0], init_buffer[1],
//            init_buffer[2], init_buffer[3], init_buffer[4]);
    rf24l01_write_register(ctx, RX_ADDR_P0, init_buffer, 5);
    rf24l01_write_register(ctx, TX_ADDR, init_buffer, 5); // must be the same if EN_AA is enabled
    rf24l01_read_register(ctx, RX_ADDR_P0, init_buffer, 5);

    init_buffer[0] = 5;
    rf24l01_write_register(ctx, RX_PW_P0, init_buffer, 1); // set payload size to 5 bytes

    init_buffer[0] = 0x0E;
    if (role == RF24L01_ROLE_RECEIVER) {
        init_buffer[0] |= 0x01;
    }
    rf24l01_write_register(ctx, CONFIG, init_buffer, 1); // enable crc, use 2 bytes, power up

    // Wait for initialization
    delay_us(ctx.timer, 150);

    return ctx;
}

u8 rf24l01_status(rf24l01_context ctx) {
    return rf24l01_send_command_single(ctx, NOP);
}

// Wrapper is used, because if we have more than one device on SPI bus, it's
// required to control its NSS bit
void rf24l01_spi_send_wrapper(rf24l01_context ctx, u8 *tx, u8 *rx, u8 bytes) {

    rf24l01_select_device(ctx);
    spi_send(ctx.spi, tx, rx, bytes);
    rf24l01_deselect_device(ctx);
}

void rf24l01_write_register(rf24l01_context ctx, u8 addr, u8 *tx_buffer, u8 bytes) {
    tx[0] = W_REGISTER | addr;
    memcpy(tx+1, tx_buffer, bytes);


    rf24l01_spi_send_wrapper(ctx, tx, rx, bytes + 1);
}

// Reads `bytes` bytes from `addr` to `rx_buffer`. Returns Status register
u8 rf24l01_read_register(rf24l01_context ctx, u8 addr, u8 *rx_buffer, u8 bytes) {
    memset(tx, 0, bytes + 1);
    tx[0] = R_REGISTER | addr;
    rf24l01_spi_send_wrapper(ctx, tx, rx_buffer, bytes + 1);
    return rx_buffer[0];
}

u8 rf24l01_send_command_single(rf24l01_context ctx, u8 command) {
    tx[0] = command;
    rf24l01_spi_send_wrapper(ctx, tx, rx, 1);
    return rx[0];
}

u8 rf24l01_send_command_multiple(rf24l01_context ctx, u8 command, u8 *data, u8 bytes) {
    tx[0] = command;
    memcpy(tx+1, data, bytes);
    rf24l01_spi_send_wrapper(ctx, tx, rx, bytes + 1);
    return rx[0];
}

void rf24l01_send_payload(rf24l01_context ctx, u8 *data, u8 size) {
    rf24l01_send_command_single(ctx, FLUSH_TX);
    rf24l01_send_command_multiple(ctx, W_TX_PAYLOAD, data, size);

    delay_us(ctx.timer, 1000);
    rf24l01_ce(ctx, 1);
    delay_us(ctx.timer, 20);
    rf24l01_ce(ctx, 0);
    delay_us(ctx.timer, 1000);
    rf24l01_reset(ctx);
}

void rf24l01_reset(rf24l01_context ctx) {
    tx[0] = 0x70;
    rf24l01_write_register(ctx, STATUS, tx, 1);
}

u8 rf24l01_receive_payload(rf24l01_context ctx) {

    rf24l01_read_register(ctx, R_RX_PAYLOAD, rx, 5);
    rf24l01_reset(ctx);

    return rx[1];
}
