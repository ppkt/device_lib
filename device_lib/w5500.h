#pragma once

#include <common_lib/spi.h>
#include <common_lib/usart.h>

#define W5500_RWB_READ 0
#define W5500_RWB_WRITE 1

#define W5500_BSB_COMMON_REG 0x00
#define W5500_BSB_SOCKET0_REG 0x01
#define W5500_BSB_SOCKET0_TX_BUF 0x02
#define W5500_BSB_SOCKET0_RX_BUF 0x03

// Offsets (Common Mode)
// Mode Register
#define W5500_MR 0x0000
// Gateway IP Address Register
#define W5500_GAR 0x0001
// Subnet Mask Register
#define W5500_SUBR 0x0005
// Source Hardware Address Register
#define W5500_SHAR 0x0009
// Source IP Address Register
#define W5500_SIPR 0x000F

#define W5500_PHYCFGR 0x002E
#define W5500_VERSIONR 0x0039

// Offsets (Socket Mode)
// Mode Register
#define W5500_SN_MR 0x0000
// Configuration Register
#define W5500_SN_CR 0x0001
// Interrupt Register
#define W5500_SN_IR 0x0002
// Status Register
#define W5500_SN_SR 0x0003
// Source Port Register
#define W5500_SN_PORT 0x0004
// Destination Hardware Address Register
#define W5500_SN_DHAR 0x0006
// Destination IP Address Register
#define W5500_SN_DIPR 0x000C
// Destination Port Register
#define W5500_SN_DPORT 0x0010

// Socket TX Free size
#define W5500_SN_TX_FSR 0x0020
// TX Read Pointer Register
#define W5500_SN_TX_RD 0x0022
// TX Write Pointer Register
#define W5500_SN_TX_WR 0x0024
// RX Received Size Register
#define W5500_SN_RX_RSR 0x0026
// RX Read Pointer Register
#define W5500_SN_RX_RD 0x0028

#define W5500_MR_RST (1u << 7u)

typedef struct {
  spi_device spi_device;
  uint8_t source_ip[4];
  uint8_t hw_addr[6];
} w5500_device;

typedef union {
  struct __attribute__((packed)) {
    uint8_t om : 2;
    uint8_t rwb : 1;
    uint8_t bsb : 5;
  };
  uint8_t raw;
} w5500_control_reg;

typedef union {
  struct __attribute__((packed)) {
    uint8_t lnk : 1;
    uint8_t spd : 1;
    uint8_t dpx : 1;
    uint8_t opmdc : 3;
    uint8_t opmd : 1;
    uint8_t rst : 1;
  };
  uint8_t raw;
} w5500_phycfgr;

#define W5500_PROTOCOL_CLOSED 0x00
#define W5500_PROTOCOL_TCP 0x01
#define W5500_PROTOCOL_UDP 0x02
#define W5500_PROTOCOL_MACRAW 0x04
typedef union {
  struct __attribute__((packed)) {
    uint8_t protocol : 4;
    uint8_t ucastb_mip6b : 1;
    uint8_t nd_mc_mmb : 1;
    uint8_t bcastb : 1;
    uint8_t multi_mfen : 1;
  };
  uint8_t raw;
} w5500_socket_mode;

#define W5500_COMMAND_OPEN 0x01
#define W5500_COMMAND_LISTEN 0x02
#define W5500_COMMAND_CONNECT 0x04
#define W5500_COMMAND_DISCON 0x08
#define W5500_COMMAND_CLOSE 0x10
#define W5500_COMMAND_SEND 0x20
#define W5500_COMMAND_SEND_MAC 0x21
#define W5500_COMMAND_SEND_KEEP 0x22
#define W5500_COMMAND_RECV 0x40

#define W5500_CON_INTERRUPT 1u << 0u
#define W5500_DISCON_INTERRUPT 1u << 1u
#define W5500_RECV_INTERRUPT 1u << 2u
#define W5500_TIMEOUT_INTERRUPT 1u << 3u
#define W5500_SEND_OK_INTERRUPT 1u << 4u
#define W5500_ALL_INTERRUPTS                                                   \
  (W5500_CON_INTERRUPT | W5500_DISCON_INTERRUPT | W5500_RECV_INTERRUPT |       \
   W5500_TIMEOUT_INTERRUPT | W5500_SEND_OK_INTERRUPT)

#define W5500_STATUS_SOCK_CLOSED 0x00
#define W5500_STATUS_SOCK_INIT 0x13
#define W5500_STATUS_SOCK_LISTEN 0x14
#define W5500_STATUS_SOCK_ESTABLISHED 0x17
#define W5500_STATUS_SOCK_CLOSE_WAIT 0x1C
#define W5500_STATUS_SOCK_UDP 0x22
#define W5500_STATUS_SOCK_MACRAW 0x42

#define W5500_STATUS_SOCK_SYNSENT 0x15
#define W5500_STATUS_SOCK_SYNRECV 0x16
#define W5500_STATUS_SOCK_FIN_WAIT 0X18
#define W5500_STATUS_SOCK_CLOSING 0X1A
#define W5500_STATUS_SOCK_TIME_WAIT 0X1B
#define W5500_STATUS_SOCK_LAST_ACK

typedef struct {
  uint8_t src_ip[4];
  uint16_t src_port;
  uint16_t length;
  uint8_t *payload;
} w5500_packet;

error_t w5500_init(const spi_device *spi_device, w5500_device *device);

error_t w5500_set_gateway_address(const w5500_device *device,
                                  const uint8_t ip_addr[4]);
error_t w5500_set_subnet_mask(const w5500_device *device,
                              const uint8_t mask[4]);
error_t w5500_set_hw_address(w5500_device *device, const uint8_t hw_addr[6]);
error_t w5500_set_src_ip_address(w5500_device *device,
                                 const uint8_t ip_addr[4]);
error_t w5500_set_socket_tx_buffer_size(const w5500_device *device, uint8_t sn,
                                        uint8_t size_kb);

error_t w5500_get_socket_interrupts(const w5500_device *device, uint8_t sn,
                                    uint8_t *socket_interrupts);

void w5500_print_socket_interrupts(uint8_t socket_interrupts);
error_t w5500_clear_socket_interrupts(const w5500_device *device, uint8_t sn,
                                      uint8_t socket_interrupts);

error_t w5500_send_udp_data(w5500_device *device, uint8_t sn,
                            const uint8_t src_ip[4], uint16_t src_port,
                            const uint8_t dst_ip[4], uint16_t dst_port,
                            const uint8_t *data, uint16_t len);
error_t w5500_get_socket_rx_data_size(const w5500_device *device, uint8_t sn,
                                      uint16_t *len);
error_t w5500_socket_recv(const w5500_device *device, uint8_t sn,
                          w5500_packet *pkt);
