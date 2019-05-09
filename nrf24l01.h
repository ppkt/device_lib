#pragma once

#include "stdbool.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"

#include "common_lib/spi.h"
#include "common_lib/utils.h"
#include "common_lib/usart.h"

// Commands
#define R_REGISTER 0x00u
#define W_REGISTER 0x20u
#define ACTIVATE 0x50
#define R_RX_PL_WID 0x60u
#define R_RX_PAYLOAD 0x61u
#define W_TX_PAYLOAD 0xA0u
#define FLUSH_TX 0xE1u
#define FLUSH_RX 0xE2u
#define NOP 0xFFu

// Registers
#define CONFIG 0x00u
#define EN_AA 0x01u
#define EN_RXADDR 0x02u
#define SETUP_AW 0x03u
#define SETUP_RETR 0x04u
#define RF_CH 0x05u
#define RF_SETUP 0x06u
#define STATUS 0x07u
#define OBSERVE_TX 0x08u
#define RPD 0x09u

// Register below - 5 bytes length
#define TX_ADDR 0x10u
#define RX_ADDR_P0 0x0Au
#define RX_ADDR_P1 0x0Bu

#define RX_ADDR_P2 0x0Cu
#define RX_ADDR_P3 0x0Du
#define RX_ADDR_P4 0x0Eu
#define RX_ADDR_P5 0x0Fu
#define RX_PW_P0 0x11u
#define RX_PW_P1 0x12u
#define RX_PW_P2 0x13u
#define RX_PW_P3 0x14u
#define RX_PW_P4 0x15u
#define RX_PW_P5 0x16u

#define FIFO_STATUS 0x17u
#define DYNPD 0x1Cu
#define FEATURE 0x1Du

typedef enum {
  NRF24L01_ROLE_TRANSMITTER = 0,
  NRF24L01_ROLE_RECEIVER
} nrf24l01_role;

typedef enum {
  NRF24L01_SPEED_1MBPS = 0b00,
  NRF24L01_SPEED_2MBPS = 0b01,
  NRF24L01_SPEED_250KBPS = 0b10,
} nrf24l01_speed;

typedef struct {
  spi_device dev;
  pin ce;
  uint32_t timer;
  nrf24l01_role role;
  uint8_t *tx;
  uint8_t *rx;
} nrf24l01_context;

typedef union {
  uint8_t data;

  struct __attribute__((packed)) {
    uint8_t tx_full : 1;
    uint8_t rx_p_no : 3;
    uint8_t max_rt : 1;
    uint8_t tx_ds : 1;
    uint8_t rx_dr : 1;
    uint8_t _reserved : 1;
  } reg;
} nrf24l01_status_reg;

typedef struct {
  uint8_t channel;
  uint8_t address[5];

  // Enable dynamic payload or set payload length
  uint8_t dynamic_payload : 1;
  uint8_t payload_length;

  // valid values are described in register RF_DR_HIGH in datasheet
  nrf24l01_speed speed : 2;
} nrf24l01_config;

nrf24l01_context *nrf24l01_init(uint32_t spi, const pin *spi_ce,
                                const pin *spi_nss, uint32_t timer,
                                nrf24l01_role role,
                                const nrf24l01_config *config);
nrf24l01_status_reg nrf24l01_status(const nrf24l01_context *ctx);
void nrf24l01_reset(const nrf24l01_context *ctx);

void nrf24l01_flush_tx(const nrf24l01_context *ctx);
void nrf24l01_flush_rx(const nrf24l01_context *ctx);

void nrf24l01_write_register(const nrf24l01_context *ctx, uint8_t addr,
                             const uint8_t *tx_buffer, uint8_t bytes);
uint8_t nrf24l01_read_register(const nrf24l01_context *ctx, uint8_t addr,
                               uint8_t *rx_buffer, uint8_t bytes);

uint8_t nrf24l01_send_command_single(const nrf24l01_context *ctx,
                                     uint8_t command);
uint8_t nrf24l01_send_command_multiple(const nrf24l01_context *ctx,
                                       uint8_t command, uint8_t *data,
                                       uint8_t bytes);

void nrf24l01_send_payload(const nrf24l01_context *ctx, uint8_t *data,
                           uint8_t bytes);
uint8_t *nrf24l01_receive_payload_static(const nrf24l01_context *ctx,
                                         uint8_t bytes);

void nrf24l01_listen(const nrf24l01_context *ctx, bool status);

void nrf24l01_setup_automatic_retransmission(const nrf24l01_context *ctx,
                                             uint16_t delay, uint8_t count);

void nrf24l01_set_tx_address(const nrf24l01_context *ctx,
                             const uint8_t *new_address);
void nrf24l01_set_rx_address(const nrf24l01_context *ctx, uint8_t pipe,
                             const uint8_t *new_address);

void nrf24l01_activate_dynamic_payload(const nrf24l01_context *ctx);
void nrf24l01_enable_dynamic_payload(const nrf24l01_context *ctx, uint8_t pipe);

void nrf24l01_disable_dynamic_payload(const nrf24l01_context *ctx);

uint8_t *nrf24l01_receive_payload_dynamic(const nrf24l01_context *ctx,
                                          uint8_t *length);
void nrf24l01_print_register_map(const nrf24l01_context *ctx);

// Low level API
// void nrf24l01_ce(nrf24l01_context ctx, bool new_state);
