#ifndef __RF24L01_H__
#define __RF24L01_H__

#include "stdbool.h"
#include "stdio.h"
#include "string.h"

#include "common_lib/spi.h"
#include "common_lib/utils.h"

// Commands
#define R_REGISTER  0b00000000
#define W_REGISTER  0b00100000
#define NOP         0b11111111

// Registers
#define CONFIG      0x00
#define EN_AA       0x01
#define EN_RXADDR   0x02
#define SETUP_AW    0x03
#define SETUP_RETR  0x04
#define RF_CH       0x05
#define RF_SETUP    0x06
#define STATUS      0x07
#define OBSERVE_TX  0x08
#define RPD         0x09

// Register below - 5 bytes length
#define TX_ADDR     0x10
#define RX_ADDR_P0  0x0A
#define RX_ADDR_P1  0x0B

#define RX_ADDR_P2  0x0C
#define RX_ADDR_P3  0x0D
#define RX_ADDR_P4  0x0E
#define RX_ADDR_P5  0x0F
#define RX_PW_P0    0x11
#define RX_PW_P1    0x12
#define RX_PW_P2    0x13
#define RX_PW_P3    0x14
#define RX_PW_P4    0x15
#define RX_PW_P5    0x16
#define FIFO_STATUS 0x17

void rf24l01_init(bool receiver);
u8 rf24l01_status(void);
void rf24l01_ce(bool new_state);
void rf24l01_write_register(u8 addr, u8 *tx_buffer, u8 bytes);
u8 rf24l01_read_register(u8 addr, u8 *rx_buffer, u8 bytes);

#endif // __RF24L01_H__
