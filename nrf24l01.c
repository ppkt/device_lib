#include "nrf24l01.h"

uint8_t tx[33];
uint8_t rx[33];

// Writes 1 on all NSS pins
void nrf24l01_deselect_device(nrf24l01_context *ctx) {
    if (ctx->device_id == 0) {
        GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_SET);
    } else if (ctx->device_id == 1) {
        GPIO_WriteBit(GPIOA, GPIO_Pin_0, Bit_SET);
    } else if (ctx->device_id == 2) {
        GPIO_WriteBit(GPIOB, GPIO_Pin_2, Bit_SET);
    }
}

// Select device to SPI transmissions by sending 0 to its NSS
void nrf24l01_select_device(nrf24l01_context *ctx) {
    nrf24l01_deselect_device(ctx);

    if (ctx->device_id == 0) {
        GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_RESET);
    } else if (ctx->device_id == 1) {
        GPIO_WriteBit(GPIOA, GPIO_Pin_0, Bit_RESET);
    } else if (ctx->device_id == 2) {
        GPIO_WriteBit(GPIOB, GPIO_Pin_2, Bit_RESET);
    }
}

void nrf24l01_ce(nrf24l01_context *ctx, bool new_state) {
    BitAction b = new_state ? Bit_SET : Bit_RESET;


    if (ctx->device_id == 0) {
        GPIO_WriteBit(GPIOA, GPIO_Pin_8, b);
    } else if (ctx->device_id == 1) {
        GPIO_WriteBit(GPIOA, GPIO_Pin_2, b);
    } else if (ctx->device_id == 2) {
        GPIO_WriteBit(GPIOB, GPIO_Pin_0, b);
    }
}

void nrf24l01_gpio_init(nrf24l01_context *ctx) {
    GPIO_InitTypeDef GPIO_InitStructure;

    // Configure NRF24 pins
    if (ctx->device_id == 0) {
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
        // CE
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
        GPIO_Init(GPIOA, &GPIO_InitStructure);
        GPIO_WriteBit(GPIOA, GPIO_Pin_8, Bit_RESET);

        // Soft NSS
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
        GPIO_Init(GPIOA, &GPIO_InitStructure);

//        // IRQ
//        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
//        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
//        GPIO_Init(GPIOA, &GPIO_InitStructure);

    } else if (ctx->device_id == 1) {
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
        // CE
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
        GPIO_Init(GPIOA, &GPIO_InitStructure);
        GPIO_WriteBit(GPIOA, GPIO_Pin_2, Bit_RESET);

        // Soft NSS
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
        GPIO_Init(GPIOA, &GPIO_InitStructure);
    } else if (ctx->device_id == 2) {
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
        // CE
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
        GPIO_Init(GPIOB, &GPIO_InitStructure);
        GPIO_WriteBit(GPIOB, GPIO_Pin_2, Bit_RESET);

        // Soft NSS
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
        GPIO_Init(GPIOB, &GPIO_InitStructure);
        GPIO_WriteBit(GPIOB, GPIO_Pin_0, Bit_RESET);

    }

    nrf24l01_deselect_device(ctx);

}

nrf24l01_context *
nrf24l01_init(uint8_t device_id, SPI_TypeDef *spi, TIM_TypeDef *timer, nrf24l01_role role,
              nrf24l01_config *config) {
    nrf24l01_context *ctx = malloc(sizeof(nrf24l01_context));
    ctx->spi = spi;
    ctx->timer = timer;
    ctx->device_id = device_id;
    ctx->role = role;

    // Initialize SPI
    spi_init(ctx->spi);

    // Prepare timer
    setup_delay_timer(ctx->timer);

    // Setup GPIO
    nrf24l01_gpio_init(ctx);

    // Turn off device
    uint8_t init_buffer[5] = {0x00};
    init_buffer[0] = 0x08;
    nrf24l01_write_register(ctx, CONFIG, init_buffer, 1);

    // Setup Auto-Ack on pipe 0
    init_buffer[0] = 0b00111111; // enable AA
//    init_buffer[0] = 0x00; // disable AA
    nrf24l01_write_register(ctx, EN_AA, init_buffer, 1);

    // Enable RX pipe 0 and 1 (default)
    init_buffer[0] = 0b11;
    nrf24l01_write_register(ctx, EN_RXADDR, init_buffer, 1);

    // Address Widths - 5 bytes RF address
    init_buffer[0] = 0b00000011;
    nrf24l01_write_register(ctx, SETUP_AW, init_buffer, 1);

    nrf24l01_setup_automatic_retransmission(ctx, 4000, 15);

    // Set channel to 2 - 2.402 GHz
    uint8_t channel = 0x02;
    if (config) channel = config->channel;
    init_buffer[0] = (uint8_t) (0x7F & channel);
    nrf24l01_write_register(ctx, RF_CH, init_buffer, 1);

    if (config) {
        nrf24l01_read_register(ctx, RF_SETUP, init_buffer, 1);
        nrf24l01_rf_setup.raw = init_buffer[1];

        nrf24l01_rf_setup.cont_wave = 0;
        nrf24l01_rf_setup._ = 0;
        nrf24l01_rf_setup.pll_lock = 0;

        nrf24l01_rf_setup.rf_dr_high = check_bit(config->speed, 0);
        nrf24l01_rf_setup.rf_dr_low = check_bit(config->speed, 1);

        nrf24l01_rf_setup.rf_pwr = 0b11;

        init_buffer[0] = nrf24l01_rf_setup.raw;
        nrf24l01_write_register(ctx, RF_SETUP, init_buffer, 1);
    } else {
        // 2Mbps, -18dB
        init_buffer[0] = 0b00001111;
        nrf24l01_write_register(ctx, RF_SETUP, init_buffer, 1);
    }


    if (config) {
        if (config->dynamic_payload) {
            // Enable dynamic payload
            nrf24l01_enable_dynamic_payload(ctx, 0);
            // Activate dynamic payload feature
            nrf24l01_activate_dynamic_payload(ctx);
        } else {
            // Set fixed payload and disable dynamic payload
            init_buffer[0] = config->payload_length;
            // Set payload size to 5 bytes on pipe 0 and 1
            nrf24l01_write_register(ctx, RX_PW_P0, init_buffer, 1);
            nrf24l01_write_register(ctx, RX_PW_P1, init_buffer, 1);

            nrf24l01_disable_dynamic_payload(ctx);
        }
    } else {
        // Enable dynamic payload
        nrf24l01_enable_dynamic_payload(ctx, 0);
        // Activate dynamic payload feature
        nrf24l01_activate_dynamic_payload(ctx);
    }


    // Set payload size to 5 bytes on pipe 1
//    nrf24l01_write_register(ctx, RX_PW_P1, init_buffer, 1);

    // Clean TX / RX queues
    nrf24l01_send_command_single(ctx, FLUSH_TX);
    nrf24l01_send_command_single(ctx, FLUSH_RX);

    // Clean interrupts
    init_buffer[0] = 0b01110000;
    nrf24l01_write_register(ctx, STATUS, init_buffer, 1);

    init_buffer[0] = 0x0E;
    if (role == NRF24L01_ROLE_RECEIVER) {
        // Set PRX bit
        init_buffer[0] |= 0x01;
    }

    // Enable CRC, use 2 bytes to encode CRC, power up
    nrf24l01_write_register(ctx, CONFIG, init_buffer, 1);

    // Wait for startup
    delay_us(ctx->timer, 1500);

    // Device in Standby-I mode
    return ctx;
}

// Return content of STATUS register
nrf24l01_status_reg nrf24l01_status(nrf24l01_context *ctx) {
    uint8_t data = nrf24l01_send_command_single(ctx, NOP);
    nrf24l01_status_reg s = { .data = data };
    return s;
}

// If device is a receiver, listen
// TODO: Maybe add switching to listening by toggling PTX / PRX bit?
void nrf24l01_listen(nrf24l01_context *ctx, bool status) {
    if (ctx->role == NRF24L01_ROLE_TRANSMITTER) {
        return;
    }

    nrf24l01_ce(ctx, status);

    if (status) {
        // Wait for RX Mode
        delay_us(ctx->timer, 130);
    }
}

// Wrapper is used, because if we have more than one device on SPI bus, it's
// required to control its NSS bit
void nrf24l01_spi_send_wrapper(nrf24l01_context *ctx, uint8_t *tx, uint8_t *rx,
                               uint8_t bytes) {
    nrf24l01_select_device(ctx);
    delay_us(ctx->timer, 10);
    spi_send(ctx->spi, tx, rx, bytes);
    delay_us(ctx->timer, 10);
    nrf24l01_deselect_device(ctx);
}

void nrf24l01_write_register(nrf24l01_context *ctx, uint8_t addr,
                             uint8_t *tx_buffer, uint8_t bytes) {
    tx[0] = W_REGISTER | addr;
    memmove(tx+1, tx_buffer, bytes);


    nrf24l01_spi_send_wrapper(ctx, tx, rx, bytes + 1);
}

// Reads `bytes` bytes from `addr` to `rx_buffer`. Returns Status register
uint8_t nrf24l01_read_register(nrf24l01_context *ctx, uint8_t addr,
                               uint8_t *rx_buffer, uint8_t bytes) {
    memset(tx, NOP, bytes + 1);
    tx[0] = R_REGISTER | addr;
    nrf24l01_spi_send_wrapper(ctx, tx, rx_buffer, bytes + 1);
    return rx_buffer[0];
}

uint8_t nrf24l01_send_command_single(nrf24l01_context *ctx, uint8_t command) {
    tx[0] = command;
    nrf24l01_spi_send_wrapper(ctx, tx, rx, 1);
    return rx[0];
}

uint8_t nrf24l01_send_command_multiple(nrf24l01_context *ctx, uint8_t command,
                                       uint8_t *data, uint8_t bytes) {
    tx[0] = command;
    memcpy(tx + 1, data, bytes);
    nrf24l01_spi_send_wrapper(ctx, tx, rx, bytes + 1);
    memcpy(data, rx + 1, bytes);
    return rx[0];
}

void nrf24l01_send_payload(nrf24l01_context *ctx, uint8_t *data, uint8_t size) {
    nrf24l01_send_command_single(ctx, FLUSH_TX);
    nrf24l01_send_command_multiple(ctx, W_TX_PAYLOAD, data, size);

    delay_us(ctx->timer, 1000);
    nrf24l01_ce(ctx, 1);
    delay_us(ctx->timer, 50);
    nrf24l01_ce(ctx, 0);
    delay_us(ctx->timer, 1000);
}

void nrf24l01_reset(nrf24l01_context *ctx) {
    tx[1] = 0x70;
    nrf24l01_write_register(ctx, STATUS, tx+1, 1);
}

// Receive packet with known size
uint8_t* nrf24l01_receive_payload_static(nrf24l01_context *ctx, uint8_t length) {

    nrf24l01_send_command_multiple(ctx, R_RX_PAYLOAD, rx, length);
    uint8_t *ret = malloc(sizeof(uint8_t) * length);
    memcpy(ret, rx, length);
    return ret;
}

// Receive packed with dynamic length (setup Dynamic Payload correctly!)
uint8_t* nrf24l01_receive_payload_dynamic(nrf24l01_context *ctx,
                                          uint8_t *length) {
    // check number of bytes
    static uint8_t buffer[5];
    nrf24l01_send_command_multiple(ctx, R_RX_PL_WID, buffer, 1);
    *length = buffer[0];

    if (*length > 32) {
        // Invalid packet arrived
        *length = 0;
        nrf24l01_send_command_single(ctx, FLUSH_RX);
    }

    // in buffer[0] there is a number of bytes of dynamic payload
    nrf24l01_send_command_multiple(ctx, R_RX_PAYLOAD, tx, *length);

    uint8_t *ret = malloc(sizeof(uint8_t) * (*length));
    memcpy(ret, tx, *length);
    return ret;
}

// Important note:
// DO NOT set first byte of address to 0x00, 0x55, 0xAA, 0xFF !!!
// Note: for convinience, only 5 bytes addresses are supported
void nrf24l01_set_tx_address(nrf24l01_context *ctx, uint8_t new_address[5]) {
    nrf24l01_write_register(ctx, TX_ADDR, new_address, 5);
}


void nrf24l01_set_rx_address(nrf24l01_context *ctx,
                             uint8_t pipe, uint8_t new_address[5]) {

    if (pipe > 5) {
        pipe = 5;
    }

    // Read register and enable pipe
    uint8_t buffer[2];
    nrf24l01_read_register(ctx, EN_RXADDR, buffer, 1);

    buffer[0] = buffer[1] | 1 << pipe;
    // Enable RX address on selected pipe
    nrf24l01_write_register(ctx, EN_RXADDR, buffer, 1);

    // Set RX address on correct pipe
    switch (pipe) {
        case 0:
            nrf24l01_write_register(ctx, RX_ADDR_P0, new_address, 5);
            break;
        case 1:
            nrf24l01_write_register(ctx, RX_ADDR_P1, new_address, 5);
            break;
        case 2:
            nrf24l01_write_register(ctx, RX_ADDR_P2, new_address, 1);
            break;
        case 3:
            nrf24l01_write_register(ctx, RX_ADDR_P3, new_address, 1);
            break;
        case 4:
            nrf24l01_write_register(ctx, RX_ADDR_P4, new_address, 1);
            break;
        case 5:
            nrf24l01_write_register(ctx, RX_ADDR_P5, new_address, 1);
            break;
    }
}

// Setup automatic retransmission
// delay - wait before retransmission - value in uS - 250 <= x <= 4000
// count - number of retransmissions - 0 <= x <= 15
void nrf24l01_setup_automatic_retransmission(nrf24l01_context *ctx,
                                             uint16_t delay, uint8_t count) {
    if (delay > 4000) {
        delay = 4000;
    } else if (delay < 250) {
        delay = 250;
    }
    if (count > 15) {
        count = 15;
    }
    delay = delay / 250 - 1;

    uint8_t buffer[1] = {delay << 4 | count};

    nrf24l01_write_register(ctx, SETUP_RETR, buffer, 1);

}

void nrf24l01_activate_dynamic_payload(nrf24l01_context *ctx) {
    uint8_t buffer[2] = {0x73};
//    nrf24l01_send_command_multiple(ctx, ACTIVATE, buffer, 1);

    // Read FEATURE register
    nrf24l01_read_register(ctx, FEATURE, buffer, 1);
    nrf24l01_feature.raw = buffer[1];
    nrf24l01_feature.en_dpl = true;

    // Toggle EN_DPL bit and set new value
    buffer[0] = nrf24l01_feature.raw;
    nrf24l01_write_register(ctx, FEATURE, buffer, 1);
}

void nrf24l01_enable_dynamic_payload(nrf24l01_context *ctx, uint8_t pipe) {
    if (pipe > 5)
        pipe = 5;

    // Read register and enable pipe
    uint8_t buffer[2];
    nrf24l01_read_register(ctx, DYNPD, buffer, 1);

    buffer[0] = (uint8_t) (buffer[1] | 1 << pipe);
    nrf24l01_write_register(ctx, DYNPD, buffer, 1);
}

void nrf24l01_disable_dynamic_payload(nrf24l01_context *ctx) {
    uint8_t buffer[2] = {0x00, 0x00};

    // Read FEATURE buffer, disable Dynamic Payload bit and write new value
    nrf24l01_read_register(ctx, FEATURE, buffer, 1);
    nrf24l01_feature.raw = buffer[1];
    nrf24l01_feature.en_dpl = 0;
    buffer[0] = nrf24l01_feature.raw;
    nrf24l01_write_register(ctx, FEATURE, buffer, 1);

    // Disable dynamic payload on all pipes
    nrf24l01_write_register(ctx, DYNPD, buffer, 1);
}


void print_field(char* name, uint8_t reg, uint8_t pos, uint8_t length,
                 uint8_t def) {

    uint8_t length_mask = (uint8_t) (0xff >> (8 - length));

    if (length > 0) {
        usart1_printf("\t%-11.11s\t%i\t%i\r\n",
                      name, (reg >> pos) & length_mask, def);
    } else {
        usart1_printf("\t%-11.11s\t- \t- \r\n", name);
    }

}

void nrf24l01_print_register_map(nrf24l01_context *ctx) {
    nrf24l01_read_register(ctx, CONFIG, rx, 1);
    usart_printf(USART1, "[CONFIG] 0x%02x\r\n", rx[1]);
    print_field("[RESERVED]",   rx[1], 7, 1, 0);
    print_field("MASK_RX_DR",   rx[1], 6, 1, 0);
    print_field("MASK_RX_DS",   rx[1], 5, 1, 0);
    print_field("MASK_MAX_RT",  rx[1], 4, 1, 0);
    print_field("EN_CRC",       rx[1], 3, 1, 1);
    print_field("CRCO",         rx[1], 2, 1, 0);
    print_field("PWR_UP",       rx[1], 1, 1, 0);
    print_field("PRIM_RX",      rx[1], 0, 1, 0);

    nrf24l01_read_register(ctx, EN_AA, rx, 1);
    usart1_printf("[EN_AA] 0x%02x\r\n", rx[1]);
    print_field("[RESERVED]",   rx[1], 6, 2, 0);
    print_field("ENAA_P5",      rx[1], 5, 1, 1);
    print_field("ENAA_P4",      rx[1], 4, 1, 1);
    print_field("ENAA_P3",      rx[1], 3, 1, 1);
    print_field("ENAA_P2",      rx[1], 2, 1, 1);
    print_field("ENAA_P1",      rx[1], 1, 1, 1);
    print_field("ENAA_P0",      rx[1], 0, 1, 1);

    nrf24l01_read_register(ctx, EN_RXADDR, rx, 1);
    usart1_printf("[EN_RXADDR] 0x%02x\r\n", rx[1]);
    print_field("[RESERVED]",   rx[1], 6, 2, 0);
    print_field("ERX_P5",       rx[1], 5, 1, 0);
    print_field("ERX_P4",       rx[1], 4, 1, 0);
    print_field("ERX_P3",       rx[1], 3, 1, 0);
    print_field("ERX_P2",       rx[1], 2, 1, 0);
    print_field("ERX_P1",       rx[1], 1, 1, 1);
    print_field("ERX_P0",       rx[1], 0, 1, 1);

    nrf24l01_read_register(ctx, SETUP_AW, rx, 1);
    usart1_printf("[SETUP_AW] 0x%02x\r\n", rx[1]);
    print_field("[RESERVED]",   rx[1], 2, 5, 0);
    print_field("AW",           rx[1], 0, 2, 0b11);

    nrf24l01_read_register(ctx, SETUP_RETR, rx, 1);
    usart1_printf("[SETUP_RETR] 0x%02x\r\n", rx[1]);
    print_field("ARD",  rx[1], 3, 4, 0);
    print_field("ARC",  rx[1], 0, 4, 3);

    nrf24l01_read_register(ctx, RF_CH, rx, 1);
    usart1_printf("[SETUP_RETR] 0x%02x\r\n", rx[1]);
    print_field("[RESERVED]",   rx[1], 7, 1, 0);
    print_field("RF_CH",        rx[1], 0, 7, 0b10);

    nrf24l01_read_register(ctx, RF_SETUP, rx, 1);
    usart1_printf("[RF_SETUP] 0x%02x\r\n", rx[1]);
    print_field("CONT_WAVE",    rx[1], 7, 1, 0);
    print_field("[RESERVED]",   rx[1], 6, 1, 0);
    print_field("RF_DR_LOW",    rx[1], 5, 1, 0);
    print_field("PLL_LOCK",     rx[1], 4, 1, 0);
    print_field("RF_DR_HIGH",   rx[1], 3, 1, 1);
    print_field("RF_PWR",       rx[1], 1, 2, 0b11);
    print_field("[OBSOLETE]",   rx[1], 0, 0, 0);

    nrf24l01_read_register(ctx, STATUS, rx, 1);
    usart1_printf("[STATUS] 0x%02x\r\n", rx[1]);
    print_field("[RESERVED]",   rx[1], 7, 1, 0);
    print_field("RX_DR",        rx[1], 6, 1, 0);
    print_field("TX_DS",        rx[1], 5, 1, 0);
    print_field("MAX_RT",       rx[1], 4, 1, 0);
    print_field("RX_P_NO",      rx[1], 1, 3, 0b111);
    print_field("TX_FULL",      rx[1], 0, 1, 0);

    nrf24l01_read_register(ctx, OBSERVE_TX, rx, 1);
    usart1_printf("[OBSERVE_TX] 0x%02x\r\n", rx[1]);
    print_field("PLOS_CNT",     rx[1], 3, 4, 0);
    print_field("ARC_CNT",      rx[1], 0, 4, 0);

    nrf24l01_read_register(ctx, RPD, rx, 1);
    usart1_printf("[RPD] 0x%02x\r\n", rx[1]);
    print_field("RPD",          rx[1], 0, 1, 0);

    nrf24l01_read_register(ctx, RX_ADDR_P0, rx, 5);
    usart1_printf("[RX_ADDR_P0] 0x%02x%02x%02x%02x%02x\t%c%c%c%c%c\r\n",
                 rx[1], rx[2], rx[3], rx[4], rx[5],
                 rx[1], rx[2], rx[3], rx[4], rx[5]);
    nrf24l01_read_register(ctx, RX_ADDR_P1, rx, 5);
    usart1_printf("[RX_ADDR_P1] 0x%02x%02x%02x%02x%02x\t%c%c%c%c%c\r\n",
                 rx[1], rx[2], rx[3], rx[4], rx[5],
                 rx[1], rx[2], rx[3], rx[4], rx[5]);
    nrf24l01_read_register(ctx, RX_ADDR_P2, rx, 1);
    usart1_printf("[RX_ADDR_P2]           %02x\t    %c\r\n",
                 rx[1], rx[1]);
    nrf24l01_read_register(ctx, RX_ADDR_P3, rx, 1);
    usart1_printf("[RX_ADDR_P3]           %02x\t    %c\r\n",
                 rx[1], rx[1]);
    nrf24l01_read_register(ctx, RX_ADDR_P4, rx, 1);
    usart1_printf("[RX_ADDR_P4]           %02x\t    %c\r\n",
                 rx[1], rx[1]);
    nrf24l01_read_register(ctx, RX_ADDR_P5, rx, 1);
    usart1_printf("[RX_ADDR_P5]           %02x\t    %c\r\n",
                 rx[1], rx[1]);

    nrf24l01_read_register(ctx, TX_ADDR, rx, 5);
    usart1_printf("[TX_ADDR]    0x%02x%02x%02x%02x%02x\t%c%c%c%c%c\r\n",
                 rx[1], rx[2], rx[3], rx[4], rx[5],
                 rx[1], rx[2], rx[3], rx[4], rx[5]);

    nrf24l01_read_register(ctx, RX_PW_P0, rx, 1);
    usart1_printf("[RX_PW_P0] 0x%02x\r\n", rx[1]);
    print_field("[RESERVED]",   rx[1], 6, 2, 0);
    print_field("RX_PW_P0",     rx[1], 0, 5, 0);

    nrf24l01_read_register(ctx, RX_PW_P1, rx, 1);
    usart1_printf("[RX_PW_P1] 0x%02x\r\n", rx[1]);
    print_field("[RESERVED]",   rx[1], 6, 2, 0);
    print_field("RX_PW_P1",     rx[1], 0, 5, 0);

    nrf24l01_read_register(ctx, RX_PW_P2, rx, 1);
    usart1_printf("[RX_PW_P2] 0x%02x\r\n", rx[1]);
    print_field("[RESERVED]",   rx[1], 6, 2, 0);
    print_field("RX_PW_P2",     rx[1], 0, 5, 0);

    nrf24l01_read_register(ctx, RX_PW_P3, rx, 1);
    usart1_printf("[RX_PW_P3] 0x%02x\r\n", rx[1]);
    print_field("[RESERVED]",   rx[1], 6, 2, 0);
    print_field("RX_PW_P3",     rx[1], 0, 5, 0);

    nrf24l01_read_register(ctx, RX_PW_P4, rx, 1);
    usart1_printf("[RX_PW_P4] 0x%02x\r\n", rx[1]);
    print_field("[RESERVED]",   rx[1], 6, 2, 0);
    print_field("RX_PW_P4",     rx[1], 0, 5, 0);

    nrf24l01_read_register(ctx, RX_PW_P5, rx, 1);
    usart1_printf("[RX_PW_P5] 0x%02x\r\n", rx[1]);
    print_field("[RESERVED]",   rx[1], 6, 2, 0);
    print_field("RX_PW_P5",     rx[1], 0, 5, 0);

    nrf24l01_read_register(ctx, FIFO_STATUS, rx, 1);
    usart1_printf("[FIFO_STATUS] 0x%02x\r\n", rx[1]);
    print_field("[RESERVED]",   rx[1], 7, 1, 0);
    print_field("TX_REUSE",     rx[1], 6, 1, 0);
    print_field("TX_FULL",      rx[1], 5, 1, 0);
    print_field("TX_EMPTY",     rx[1], 4, 1, 1);
    print_field("[RESERVED]",   rx[1], 2, 2, 0);
    print_field("RX_FULL",      rx[1], 1, 1, 0);
    print_field("RX_EMPTY",     rx[1], 0, 1, 1);

    nrf24l01_read_register(ctx, DYNPD, rx, 1);
    usart1_printf("[DYNPD] 0x%02x\r\n", rx[1]);
    print_field("[RESERVED]",   rx[1], 6, 1, 0);
    print_field("DPL_P5",       rx[1], 5, 1, 0);
    print_field("DPL_P4",       rx[1], 4, 1, 0);
    print_field("DPL_P3",       rx[1], 3, 1, 0);
    print_field("DPL_P2",       rx[1], 2, 1, 0);
    print_field("DPL_P1",       rx[1], 1, 1, 0);
    print_field("DPL_P0",       rx[1], 0, 1, 0);

    nrf24l01_read_register(ctx, FEATURE, rx, 1);
    usart1_printf("[FEATURE] 0x%02x\r\n", rx[1]);
    print_field("[RESERVED]",   rx[1], 3, 5, 0);
    print_field("EN_DPL",       rx[1], 2, 1, 0);
    print_field("EN_ACK_PAY",   rx[1], 1, 1, 0);
    print_field("EN_DYN_ACK",   rx[1], 0, 1, 0);
}
