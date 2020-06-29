#include "w5500.h"

void w5500_gpio_init(const spi_device *device) {
  rcc_periph_clock_enable(port_to_rcc(device->nss.port));

#if STM32F1
  gpio_set_mode(device->nss.port, GPIO_MODE_OUTPUT_50_MHZ,
                GPIO_CNF_OUTPUT_PUSHPULL, device->nss.gpio);
#else
#error TODO
#endif
  spi_cs_deselect(device);
}

#define ARRAY_TO_16BIT_VALUE(array) ((uint16_t)(array[0] << 8u) | array[1])

#define VALUE_16BIT_TO_ARRAY(value, array)                                     \
  do {                                                                         \
    array[0] = (uint8_t)(value >> 8u) & 0xFFu;                                 \
    array[1] = value & 0xFFu;                                                  \
  } while (0)

error_t w5500_read(const w5500_device *device, uint8_t reg, uint16_t offset,
                   uint8_t *data, uint16_t length) {
  w5500_control_reg control_reg = {
      .rwb = W5500_RWB_READ,
      .bsb = reg,
      .om = 0x00,
  };

  //  debug_printf(" Reading %d bytes from offset 0x%.4x\r\n", length, offset);
  spi_cs_select(&device->spi_device);
  spi_xfer(device->spi_device.spi, offset >> 8u);
  spi_xfer(device->spi_device.spi, offset & 0xFFu);
  spi_xfer(device->spi_device.spi, control_reg.raw);

  for (uint16_t i = 0; i < length; ++i) {
    data[i] = spi_xfer(device->spi_device.spi, 0xFF);
  }
  spi_cs_deselect(&device->spi_device);

  return E_SUCCESS;
}

error_t w5500_write(const w5500_device *device, uint8_t reg, uint16_t offset,
                    const uint8_t *data, uint16_t length) {
  w5500_control_reg control_reg = {
      .rwb = W5500_RWB_WRITE,
      .bsb = reg,
      .om = 0x00,
  };
  //  debug_printf("Writing %d bytes to offset 0x%.4x\r\n", length, offset);
  spi_cs_select(&device->spi_device);
  spi_xfer(device->spi_device.spi, offset >> 8u);
  spi_xfer(device->spi_device.spi, offset & 0xFFu);
  spi_xfer(device->spi_device.spi, control_reg.raw);

  for (uint16_t i = 0; i < length; ++i) {
    spi_xfer(device->spi_device.spi, data[i]);
  }
  spi_cs_deselect(&device->spi_device);
  return E_SUCCESS;
}

void w5500_print(const uint8_t buffer[], const uint8_t len) {
  for (uint8_t i = 0; i < len; ++i) {
    print_variable_hex(buffer[i]);
  }
}

error_t w5500_soft_reset(const w5500_device *device) {
  check_error(w5500_write(device, W5500_BSB_COMMON_REG, W5500_MR,
                          (uint8_t[]){W5500_MR_RST}, 1));
  return E_SUCCESS;
}

error_t w5500_link_status(const w5500_device *device, w5500_phycfgr *reg) {
  uint8_t buffer[1] = {
      0,
  };
  check_error(w5500_read(device, W5500_BSB_COMMON_REG, W5500_PHYCFGR, buffer,
                         sizeof_a(buffer)));
  reg->raw = buffer[0];

  return E_SUCCESS;
}

bool w5500_check_presence(const w5500_device *device) {
  uint8_t buffer[1] = {
      0,
  };
  check_error(w5500_read(device, W5500_BSB_COMMON_REG, W5500_VERSIONR, buffer,
                         sizeof_a(buffer)));
  return buffer[0] == 0x04;
}

error_t w5500_init(const spi_device *spi_device, w5500_device *device) {
  device->spi_device = *spi_device;
  spi_init(device->spi_device.spi);
  w5500_gpio_init(&device->spi_device);

  if (!w5500_check_presence(device)) {
    return E_NOT_FOUND;
  }

  check_error(w5500_soft_reset(device));

  w5500_phycfgr phy_reg = {.raw = 0x00};

  uint32_t cnt = 0;
  do {
    delay_ms(1);
    check_error(w5500_link_status(device, &phy_reg));
    if (++cnt == 0xFFFF) {
      debug_print("Timed out waiting for link up");
      return E_TIMEOUT;
    }
  } while (phy_reg.lnk != 1);
  print_variable_int(cnt);

  if (phy_reg.lnk) {
    debug_print("Link is up\r\n");
  } else {
    debug_print("Link is DOWN!\r\n");
  }

  if (phy_reg.spd) {
    debug_print("100 Mbps\r\n");
  } else {
    debug_print("10 Mbps\r\n");
  }

  if (phy_reg.dpx) {
    debug_print("Full duplex\r\n");
  } else {
    debug_print("Half duplex\r\n");
  }

  cm3_assert(phy_reg.lnk);

  return E_SUCCESS;
}

error_t w5500_get_socket_status(const w5500_device *device, uint8_t sn,
                                uint8_t *status) {
  uint8_t buffer[1] = {
      0,
  };
  check_error(w5500_read(device, W5500_BSB_SOCKET0_REG + sn * 4, W5500_SN_SR,
                         buffer, sizeof_a(buffer)));
  *status = buffer[0];
  return E_SUCCESS;
}

error_t w5500_get_socket_protocol(const w5500_device *device, uint8_t sn,
                                  uint8_t *protocol) {
  uint8_t buffer[1];
  check_error(w5500_read(device, W5500_BSB_SOCKET0_REG + sn * 4, W5500_SN_MR,
                         buffer, sizeof_a(buffer)));
  w5500_socket_mode mode = {.raw = buffer[0]};
  *protocol = mode.protocol;
  return E_SUCCESS;
}

error_t w5500_print_socket_status(const w5500_device *device, uint8_t sn,
                                  uint8_t *status) {
  check_error(w5500_get_socket_status(device, sn, status));

  debug_printf("Socket %d status: \r\n", sn);
  switch (*status) {
  case W5500_STATUS_SOCK_CLOSED:
    debug_print("SOCK_CLOSED\r\n");
    break;
  case W5500_STATUS_SOCK_INIT:
    debug_print("SOCK_INIT\r\n");
    break;
  case W5500_STATUS_SOCK_UDP:
    debug_print("SOCK_UDP\r\n");
    break;
  default:
    debug_printf("TODO: NOT IMPLEMENTED: %02x\r\n", *status);
  }
  return E_SUCCESS;
}

error_t w5500_set_gateway_address(const w5500_device *device,
                                  const uint8_t *ip_addr) {
  debug_printf("Setting gateway to: %d.%d.%d.%d\r\n", ip_addr[0], ip_addr[1],
               ip_addr[2], ip_addr[3]);
  check_error(w5500_write(device, W5500_BSB_COMMON_REG, W5500_GAR, ip_addr, 4));
  return E_SUCCESS;
}

error_t w5500_set_subnet_mask(const w5500_device *device, const uint8_t *mask) {
  debug_printf("Setting subnet mask to: %d.%d.%d.%d\r\n", mask[0], mask[1],
               mask[2], mask[3]);
  check_error(w5500_write(device, W5500_BSB_COMMON_REG, W5500_SUBR, mask, 4));
  return E_SUCCESS;
}

error_t w5500_set_hw_address(w5500_device *device, const uint8_t *hw_addr) {
  debug_printf("Setting HW Address to: %02x:%02x:%02x:%02x:%02x:%02x\r\n",
               hw_addr[0], hw_addr[1], hw_addr[2], hw_addr[3], hw_addr[4],
               hw_addr[5]);
  memcpy(device->hw_addr, hw_addr, 6);
  check_error(w5500_write(device, W5500_BSB_COMMON_REG, W5500_SHAR,
                          device->hw_addr, 6));
  return E_SUCCESS;
}

error_t w5500_set_socket_protocol(const w5500_device *device, uint8_t sn,
                                  uint8_t protocol) {
  w5500_socket_mode socket_mode = {.raw = 0x00};
  socket_mode.protocol = protocol;
  check_error(w5500_write(device, W5500_BSB_SOCKET0_REG + sn * 4, W5500_SN_MR,
                          (uint8_t[]){socket_mode.raw}, 1));
  return E_SUCCESS;
}

error_t w5500_socket_send_command(const w5500_device *device, uint8_t sn,
                                  uint8_t command) {
  check_error(w5500_write(device, W5500_BSB_SOCKET0_REG + sn * 4, W5500_SN_CR,
                          (uint8_t[]){command}, 1));
  return E_SUCCESS;
}

error_t w5500_set_src_ip_address(w5500_device *device, const uint8_t *ip_addr) {
  // close all sockets (sorry!)
  for (uint8_t i = 0; i < 8; ++i) {
    //    w5500_socket_send_command(device, i, W5500_COMMAND_CLOSE);
  }

  debug_printf("Setting IP Address to: %d.%d.%d.%d\r\n", ip_addr[0], ip_addr[1],
               ip_addr[2], ip_addr[3]);
  memcpy(device->source_ip, ip_addr, 4 * sizeof(uint8_t));
  check_error(w5500_write(device, W5500_BSB_COMMON_REG, W5500_SIPR,
                          device->source_ip, 4));

  return E_SUCCESS;
}

error_t w5500_get_socket_interrupts(const w5500_device *device, uint8_t sn,
                                    uint8_t *interrupts) {
  uint8_t buffer[1] = {
      0,
  };
  check_error(w5500_read(device, W5500_BSB_SOCKET0_REG + sn * 4, W5500_SN_IR,
                         buffer, sizeof_a(buffer)));
  *interrupts = buffer[0];
  return E_SUCCESS;
}

error_t w5500_clear_socket_interrupts(const w5500_device *device, uint8_t sn,
                                      uint8_t socket_interrupts) {
  check_error(w5500_write(device, W5500_BSB_SOCKET0_REG + sn * 4, W5500_SN_IR,
                          (uint8_t[]){socket_interrupts}, 1));
  return E_SUCCESS;
}

void w5500_print_socket_interrupts(uint8_t socket_interrupts) {

  if (socket_interrupts == 0) {
    // Shortcut if no interrupts
    return;
  }

  if (socket_interrupts & W5500_CON_INTERRUPT)
    debug_print("W5500_CON_INTERRUPT\r\n");
  if (socket_interrupts & W5500_DISCON_INTERRUPT)
    debug_print("W5500_DISCON_INTERRUPT\r\n");
  if (socket_interrupts & W5500_RECV_INTERRUPT)
    debug_print("W5500_RECV_INTERRUPT\r\n");
  if (socket_interrupts & W5500_TIMEOUT_INTERRUPT)
    debug_print("W5500_TIMEOUT_INTERRUPT\r\n");
  if (socket_interrupts & W5500_SEND_OK_INTERRUPT)
    debug_print("W5500_SEND_OK_INTERRUPT\r\n");
}

error_t w5500_set_source_port(const w5500_device *device, uint8_t sn,
                              uint16_t port) {
  uint8_t buffer[2] = {
      port >> 8u,
      port & 0xFFu,
  };
  check_error(w5500_write(device, W5500_BSB_SOCKET0_REG + sn * 4, W5500_SN_PORT,
                          buffer, 2));
  return E_SUCCESS;
}

error_t w5500_set_destination_port(const w5500_device *device, uint8_t sn,
                                   uint16_t port) {
  uint8_t buffer[2] = {
      port >> 8u,
      port & 0xFFu,
  };
  check_error(w5500_write(device, W5500_BSB_SOCKET0_REG + sn * 4,
                          W5500_SN_DPORT, buffer, 2));
  return E_SUCCESS;
}

error_t w5500_set_destination_ip(const w5500_device *device, uint8_t sn,
                                 const uint8_t ip_addr[4]) {
  check_error(w5500_write(device, W5500_BSB_SOCKET0_REG + sn * 4, W5500_SN_DIPR,
                          ip_addr, 4));
  return E_SUCCESS;
}

error_t w5500_set_socket_tx_buffer_size(const w5500_device *device, uint8_t sn,
                                        uint8_t size_kb) {
  switch (size_kb) {
  case 0:
  case 1:
  case 2:
  case 4:
  case 8:
  case 16:
    break;
  default:
    return E_VALUE_INVALID;
  }

  check_error(w5500_write(device, W5500_BSB_SOCKET0_REG + sn * 4, W5500_SN_DIPR,
                          (uint8_t[]){size_kb}, 1));
  return E_SUCCESS;
}

error_t w5500_write_socket_tx_data(const w5500_device *device, uint8_t sn,
                                   const uint8_t *data, uint16_t len) {
  uint8_t ptr[2];
  uint16_t free_size;
  uint32_t cnt = 0;

  do {
    // Check buffer free size
    check_error(w5500_read(device, W5500_BSB_SOCKET0_REG + sn * 4,
                           W5500_SN_TX_FSR, ptr, 2));
    free_size = ARRAY_TO_16BIT_VALUE(ptr);
    if (++cnt == 0xFFFF) {
      debug_print("Timeout waiting for buffer");
      return E_TIMEOUT;
    }
  } while (free_size < len);

  // Get pointer to the beginning of TX buffer
  check_error(w5500_read(device, W5500_BSB_SOCKET0_REG + sn * 4, W5500_SN_TX_WR,
                         ptr, 2));
  uint16_t ptr_int = ARRAY_TO_16BIT_VALUE(ptr);
  print_variable_int(ptr_int);

  // Write data to buffer
  check_error(w5500_write(device, W5500_BSB_SOCKET0_TX_BUF + sn * 4, ptr_int,
                          data, len));
  ptr_int += len;

  // Write new pointer to TX buffer indicating how much data to send
  VALUE_16BIT_TO_ARRAY(ptr_int, ptr);
  check_error(w5500_write(device, W5500_BSB_SOCKET0_REG + sn * 4,
                          W5500_SN_TX_WR, ptr, 2));
  return E_SUCCESS;
}

error_t w5500_send_udp_packet(const w5500_device *device, uint8_t sn,
                              const uint8_t *data, uint8_t len) {
  uint8_t ptr[2] = {
      0,
  };
  uint8_t status;
  uint8_t socket_interrupts;

  // Clear interrupts
  check_error(w5500_get_socket_interrupts(device, sn, &socket_interrupts));
  w5500_print_socket_interrupts(socket_interrupts);
  check_error(w5500_clear_socket_interrupts(device, sn, W5500_ALL_INTERRUPTS));

  // Set socket mode to UDP
  w5500_socket_mode socket_mode = {.raw = 0x00};
  socket_mode.protocol = W5500_PROTOCOL_UDP;
  check_error(w5500_write(device, W5500_BSB_SOCKET0_REG + sn * 4, W5500_SN_MR,
                          (uint8_t[]){socket_mode.raw}, 1));

  // Read back to ensure that socket mode is set correctly
  check_error(
      w5500_read(device, W5500_BSB_SOCKET0_REG + sn * 4, W5500_SN_MR, ptr, 1));
  cm3_assert(ptr[0] == socket_mode.raw);

  // Set source port
  check_error(w5500_set_source_port(device, sn, 6666 + sn));

  // Open port
  check_error(w5500_socket_send_command(device, sn, W5500_COMMAND_OPEN));

  // Wait until socket is ready
  uint32_t cnt = 0;
  do {
    check_error(w5500_get_socket_status(device, sn, &status));
    if (++cnt == 0xFFFF) {
      debug_print("Socket not ready");
      return E_TIMEOUT;
    }
  } while ((status != W5500_STATUS_SOCK_UDP));

  // Set destination IP address and port
  check_error(
      w5500_set_destination_ip(device, sn, (uint8_t[]){192, 168, 0, 10}));
  check_error(w5500_set_destination_port(device, sn, 6666));

  check_error(w5500_write_socket_tx_data(device, sn, data, len));

  // Order sending packet
  check_error(w5500_socket_send_command(device, sn, W5500_COMMAND_SEND));

  // Wait for packet sending
  cnt = 0;
  do {
    w5500_get_socket_interrupts(device, sn, &socket_interrupts);
    if (++cnt == 0xFFFF) {
      debug_print("Send timeout");
      return E_TIMEOUT;
    }
  } while ((socket_interrupts & W5500_SEND_OK_INTERRUPT) == 0);

  // Clear only send_ok interrupt
  w5500_clear_socket_interrupts(device, sn, W5500_SEND_OK_INTERRUPT);

  return E_SUCCESS;
}

error_t w5500_send_udp_data(w5500_device *device, uint8_t sn,
                            const uint8_t *src_ip, uint16_t src_port,
                            const uint8_t *dst_ip, uint16_t dst_port,
                            const uint8_t *data, uint16_t len) {
  // Check whether provided socket is already opened as UDP
  uint8_t status, socket_interrupts;
  bool initialize_required = false;

  if (memcmp(device->source_ip, src_ip, 4 * sizeof(uint8_t)) != 0) {
    check_error(w5500_set_src_ip_address(device, src_ip));
  }

  check_error(w5500_get_socket_status(device, sn, &status));

  if (status != W5500_STATUS_SOCK_UDP) {
    initialize_required = true;
    w5500_set_socket_protocol(device, sn, W5500_PROTOCOL_UDP);
  }

  // Set source port
  check_error(w5500_set_source_port(device, sn, src_port));

  if (initialize_required) {
    // Open port
    check_error(w5500_socket_send_command(device, sn, W5500_COMMAND_OPEN));

    // Wait until socket is ready
    uint32_t cnt = 0;
    do {
      check_error(w5500_get_socket_status(device, sn, &status));
      if (++cnt == 0xFFFF) {
        debug_print("Socket not ready");
        return E_TIMEOUT;
      }
    } while ((status != W5500_STATUS_SOCK_UDP));
  }

  // Set destination IP address and port
  check_error(w5500_set_destination_ip(device, sn, dst_ip));
  check_error(w5500_set_destination_port(device, sn, dst_port));

  // Write data to socket TX buffer
  check_error(w5500_write_socket_tx_data(device, sn, data, len));

  // Order sending packet
  check_error(w5500_socket_send_command(device, sn, W5500_COMMAND_SEND));

  // Wait for packet sending
  uint32_t cnt = 0;
  do {
    w5500_get_socket_interrupts(device, sn, &socket_interrupts);
    if (++cnt == 0xFFFF) {
      debug_print("Send timeout");
      return E_TIMEOUT;
    }
  } while ((socket_interrupts & W5500_SEND_OK_INTERRUPT) == 0);

  // Clear only send_ok interrupt
  w5500_clear_socket_interrupts(device, sn, W5500_SEND_OK_INTERRUPT);

  debug_printf("Sent %d bytes from %d.%d.%d.%d:%d to %d.%d.%d.%d:%d\r\n", len,
               src_ip[0], src_ip[1], src_ip[2], src_ip[3], src_port, dst_ip[0],
               dst_ip[1], dst_ip[2], dst_ip[3], dst_port);

  return E_SUCCESS;
}

error_t w5500_get_socket_rx_data_size(const w5500_device *device, uint8_t sn,
                                       uint16_t *len) {
  uint8_t ptr[2];
  // Get amount of data in RX register
  check_error(w5500_read(device, W5500_BSB_SOCKET0_REG + sn * 4,
                         W5500_SN_RX_RSR, ptr, 2));
  *len = ARRAY_TO_16BIT_VALUE(ptr);
  return E_SUCCESS;
}

error_t w5500_read_socket_rx_data(const w5500_device *device, uint8_t sn,
                                  uint8_t *data, uint16_t len) {
  uint8_t ptr[2];
  // Get pointer to beginning of data in RX buffer
  check_error(w5500_read(device, W5500_BSB_SOCKET0_REG + sn * 4, W5500_SN_RX_RD,
                         ptr, 2));
  uint16_t ptr_val = ARRAY_TO_16BIT_VALUE(ptr);

  // Start reading from place indicated by pointer
  check_error(w5500_read(device, W5500_BSB_SOCKET0_RX_BUF + sn * 4, ptr_val,
                         data, len));
  ptr_val += len;

  // Update RX buffer pointer
  VALUE_16BIT_TO_ARRAY(ptr_val, ptr);
  check_error(w5500_write(device, W5500_BSB_SOCKET0_REG + sn * 4,
                          W5500_SN_RX_RD, ptr, 2));

  // Order RECV command to indicate that read was successful
  check_error(w5500_socket_send_command(device, sn, W5500_COMMAND_RECV));

  return E_SUCCESS;
}

error_t w5500_socket_recv(const w5500_device *device, uint8_t sn,
                          w5500_packet *pkt) {
  uint16_t len = 0;
  uint8_t buf[8];

  // Get amount of data in RX register
  check_error(w5500_get_socket_rx_data_size(device, sn, &len));
  print_variable_int(len);

  // Check protocol
  uint8_t protocol;
  check_error(w5500_get_socket_protocol(device, sn, &protocol));
  cm3_assert(protocol == W5500_PROTOCOL_UDP);

  // Receive packet header
  check_error(w5500_read_socket_rx_data(device, sn, buf, 8));

  memcpy(pkt->src_ip, buf, 4 * sizeof(uint8_t));
  uint8_t *buf_ptr = buf + 4;
  pkt->src_port = ARRAY_TO_16BIT_VALUE(buf_ptr);
  buf_ptr += 2;
  pkt->length = ARRAY_TO_16BIT_VALUE(buf_ptr);
  debug_printf("%d bytes from %d.%d.%d.%d:%d\r\n", pkt->length, pkt->src_ip[0],
               pkt->src_ip[1], pkt->src_ip[2], pkt->src_ip[3], pkt->src_port);

  pkt->payload = malloc(pkt->length * sizeof(uint8_t));
  if (pkt->payload == NULL) {
    return E_NULL_PTR;
  }

  check_error(w5500_read_socket_rx_data(device, sn, pkt->payload, pkt->length));
  return E_SUCCESS;
}
