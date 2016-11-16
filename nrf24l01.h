#ifndef __RF24L01_H__
#define __RF24L01_H__

#include "stdbool.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"

#include "common_lib/spi.h"
#include "common_lib/utils.h"

// Commands
#define R_REGISTER      0x00
#define W_REGISTER      0x20
#define ACTIVATE        0x50
#define R_RX_PL_WID     0x60
#define R_RX_PAYLOAD    0x61
#define W_TX_PAYLOAD    0xA0
#define FLUSH_TX        0xE1
#define FLUSH_RX        0xE2
#define NOP             0xFF

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
#define DYNPD       0x1C
#define FEATURE     0x1D


typedef enum {
    NRF24L01_ROLE_TRANSMITTER = 0,
    NRF24L01_ROLE_RECEIVER
} nrf24l01_role;

typedef struct {
    SPI_TypeDef *spi;
    TIM_TypeDef *timer;
    nrf24l01_role role;
    uint8_t device_id;
} nrf24l01_context;

nrf24l01_context* nrf24l01_init(uint8_t device_id, SPI_TypeDef *spi,
                                TIM_TypeDef *timer, nrf24l01_role role);
uint8_t nrf24l01_status(nrf24l01_context *ctx);
void nrf24l01_reset(nrf24l01_context *ctx);

void nrf24l01_flush_tx(nrf24l01_context *ctx);
void nrf24l01_flush_rx(nrf24l01_context *ctx);

void nrf24l01_write_register(nrf24l01_context *ctx, uint8_t addr,
                             uint8_t *tx_buffer, uint8_t bytes);
uint8_t nrf24l01_read_register(nrf24l01_context *ctx, uint8_t addr,
                               uint8_t *rx_buffer, uint8_t bytes);

uint8_t nrf24l01_send_command_single(nrf24l01_context *ctx, uint8_t command);
uint8_t nrf24l01_send_command_multiple(nrf24l01_context *ctx, uint8_t command,
                                       uint8_t *data, uint8_t bytes);

void nrf24l01_send_payload(nrf24l01_context *ctx, uint8_t *data, uint8_t bytes);
uint8_t nrf24l01_receive_payload(nrf24l01_context *ctx, uint8_t bytes);

void nrf24l01_listen(nrf24l01_context *ctx, bool status);

void nrf24l01_setup_automatic_retransmission(nrf24l01_context *ctx,
                                             uint16_t delay, uint8_t count);

void nrf24l01_set_tx_address(nrf24l01_context *ctx, uint8_t new_address[5]);
void nrf24l01_set_rx_address(nrf24l01_context *ctx,
                             uint8_t pipe, uint8_t new_address[5]);

void nrf24l01_activate_dynamic_payload(nrf24l01_context *ctx);
void nrf24l01_enable_dynamic_payload(nrf24l01_context *ctx, uint8_t pipe);
char *nrf24l01_receive_payload_dynamic(nrf24l01_context *ctx);
// Low level API
//void nrf24l01_ce(nrf24l01_context ctx, bool new_state);

#endif // __RF24L01_H__
