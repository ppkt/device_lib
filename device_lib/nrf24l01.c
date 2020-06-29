#include "nrf24l01.h"

static void nrf24l01_nss(const nrf24l01_context *ctx, bool new_state);
static void nrf24l01_ce(const nrf24l01_context *ctx, bool new_state);
static void nrf24l01_spi_send_wrapper(const nrf24l01_context *ctx,
                                      const uint8_t *_tx, uint8_t *_rx,
                                      uint8_t bytes);
static void nrf24l01_gpio_init(const nrf24l01_context *ctx);
static void print_field(char *name, uint8_t reg, uint8_t pos, uint8_t length,
                        uint8_t def);

union {
  uint8_t raw;
  struct __attribute__((packed)) {
    uint8_t obsolete : 1;
    uint8_t rf_pwr : 2;
    uint8_t rf_dr_high : 1;
    uint8_t pll_lock : 1;
    uint8_t rf_dr_low : 1;
    uint8_t : 1;
    uint8_t cont_wave : 1;
  };
} nrf24l01_rf_setup;

union {
  uint8_t raw;
  struct __attribute__((packed)) {
    uint8_t en_dyn_ack : 1; // dynamic acknowledgement
    uint8_t en_ack_pay : 1; // payload with ack
    uint8_t en_dpl : 1;     // dynamic payload length
    uint8_t : 5;
  };
} nrf24l01_feature;

void nrf24l01_nss(const nrf24l01_context *ctx, bool new_state) {
  void (*fun_ptr)(uint32_t, uint16_t);
  fun_ptr = new_state ? gpio_set : gpio_clear;

  fun_ptr(ctx->dev.nss.port, ctx->dev.nss.gpio);
}

void nrf24l01_ce(const nrf24l01_context *ctx, bool new_state) {
  void (*fun_ptr)(uint32_t, uint16_t);
  fun_ptr = new_state ? gpio_set : gpio_clear;

  fun_ptr(ctx->ce.port, ctx->ce.gpio);
}

void nrf24l01_gpio_init(const nrf24l01_context *ctx) {
  if (ctx->ce.port == GPIOA || ctx->dev.nss.port == GPIOA) {
    rcc_periph_clock_enable(RCC_GPIOA);
  }
  if (ctx->ce.port == GPIOB || ctx->dev.nss.port == GPIOB) {
    rcc_periph_clock_enable(RCC_GPIOB);
  }

#ifdef STM32F0
  gpio_mode_setup(ctx->ce.port, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, ctx->ce.gpio);
  gpio_mode_setup(ctx->nss.port, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,
                  ctx->nss.gpio);

#elif STM32F1
  gpio_set_mode(ctx->ce.port, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL,
                ctx->ce.gpio);
  gpio_set_mode(ctx->dev.nss.port, GPIO_MODE_OUTPUT_50_MHZ,
                GPIO_CNF_OUTPUT_PUSHPULL, ctx->dev.nss.gpio);
#endif
  gpio_clear(ctx->ce.port, ctx->ce.gpio);
  gpio_set(ctx->dev.nss.port, ctx->dev.nss.gpio);
}

nrf24l01_context *nrf24l01_init(uint32_t spi, const pin *spi_ce,
                                const pin *spi_nss, uint32_t timer,
                                nrf24l01_role role,
                                const nrf24l01_config *config) {
  nrf24l01_context *ctx = malloc(sizeof(nrf24l01_context));
  ctx->dev.spi = spi;
  ctx->timer = timer;
  ctx->role = role;
  ctx->ce = (pin){
      .gpio = spi_ce->gpio,
      .port = spi_ce->port,
  };
  ctx->dev.nss = (pin){
      .gpio = spi_nss->gpio,
      .port = spi_nss->port,
  };

  ctx->tx = malloc(sizeof(uint8_t) * 33);
  ctx->rx = malloc(sizeof(uint8_t) * 33);

  // Initialize SPI
  spi_init(ctx->dev.spi);

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
  // init_buffer[0] = 0x00;       // disable AA
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
  if (config) {
    channel = config->channel;
  }
  init_buffer[0] = (uint8_t)(0x7Fu & channel);
  nrf24l01_write_register(ctx, RF_CH, init_buffer, 1);

  if (config) {
    nrf24l01_read_register(ctx, RF_SETUP, init_buffer, 1);
    nrf24l01_rf_setup.raw = init_buffer[1];

    nrf24l01_rf_setup.cont_wave = 0;
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
    init_buffer[0] |= 0x01u;
  }

  // Enable CRC, use 2 bytes to encode CRC, power up
  nrf24l01_write_register(ctx, CONFIG, init_buffer, 1);

  // Wait for startup
  delay_us(ctx->timer, 1500);

  // Device in Standby-I mode
  return ctx;
}

// Return content of STATUS register
nrf24l01_status_reg nrf24l01_status(const nrf24l01_context *ctx) {
  uint8_t data = nrf24l01_send_command_single(ctx, NOP);
  nrf24l01_status_reg s = {.data = data};
  return s;
}

// If device is a receiver, listen
// TODO: Maybe add switching to listening by toggling PTX / PRX bit?
void nrf24l01_listen(const nrf24l01_context *ctx, bool status) {
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
void nrf24l01_spi_send_wrapper(const nrf24l01_context *ctx, const uint8_t *_tx,
                               uint8_t *_rx, uint8_t bytes) {
  nrf24l01_nss(ctx, false);
  delay_us(ctx->timer, 10);
  spi_send_recv(ctx->dev.spi, _tx, _rx, bytes);
  delay_us(ctx->timer, 10);
  nrf24l01_nss(ctx, true);
}

void nrf24l01_write_register(const nrf24l01_context *ctx, uint8_t addr,
                             const uint8_t *tx_buffer, uint8_t bytes) {
  ctx->tx[0] = W_REGISTER | addr;
  memmove(ctx->tx + 1, tx_buffer, bytes);

  nrf24l01_spi_send_wrapper(ctx, ctx->tx, ctx->rx, bytes + 1);
}

// Reads `bytes` bytes from `addr` to `ctx->rx_buffer`. Returns Status register
uint8_t nrf24l01_read_register(const nrf24l01_context *ctx, uint8_t addr,
                               uint8_t *rx_buffer, uint8_t bytes) {
  memset(ctx->tx, NOP, bytes + 1);
  ctx->tx[0] = R_REGISTER | addr;
  nrf24l01_spi_send_wrapper(ctx, ctx->tx, rx_buffer, bytes + 1);
  return rx_buffer[0];
}

uint8_t nrf24l01_send_command_single(const nrf24l01_context *ctx,
                                     uint8_t command) {
  ctx->tx[0] = command;
  nrf24l01_spi_send_wrapper(ctx, ctx->tx, ctx->rx, 1);
  return ctx->rx[0];
}

uint8_t nrf24l01_send_command_multiple(const nrf24l01_context *ctx,
                                       uint8_t command, uint8_t *data,
                                       uint8_t bytes) {
  ctx->tx[0] = command;
  memcpy(ctx->tx + 1, data, bytes);
  nrf24l01_spi_send_wrapper(ctx, ctx->tx, ctx->rx, bytes + 1);
  memcpy(data, ctx->rx + 1, bytes);
  return ctx->rx[0];
}

void nrf24l01_send_payload(const nrf24l01_context *ctx, uint8_t *data,
                           uint8_t bytes) {
  nrf24l01_send_command_single(ctx, FLUSH_TX);
  nrf24l01_send_command_multiple(ctx, W_TX_PAYLOAD, data, bytes);

  delay_us(ctx->timer, 1000);
  nrf24l01_ce(ctx, 1);
  delay_us(ctx->timer, 50);
  nrf24l01_ce(ctx, 0);
  delay_us(ctx->timer, 1000);
}

void nrf24l01_reset(const nrf24l01_context *ctx) {
  ctx->tx[1] = 0x70;
  nrf24l01_write_register(ctx, STATUS, ctx->tx + 1, 1);
}

// Receive packet with known size
uint8_t *nrf24l01_receive_payload_static(const nrf24l01_context *ctx,
                                         uint8_t bytes) {

  nrf24l01_send_command_multiple(ctx, R_RX_PAYLOAD, ctx->rx, bytes);
  uint8_t *ret = malloc(sizeof(uint8_t) * bytes);
  memcpy(ret, ctx->rx, bytes);
  return ret;
}

// Receive packed with dynamic length (setup Dynamic Payload correctly!)
uint8_t *nrf24l01_receive_payload_dynamic(const nrf24l01_context *ctx,
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
  nrf24l01_send_command_multiple(ctx, R_RX_PAYLOAD, ctx->tx, *length);

  uint8_t *ret = malloc(sizeof(uint8_t) * (*length));
  memcpy(ret, ctx->tx, *length);
  return ret;
}

// Important note:
// DO NOT set first byte of address to 0x00, 0x55, 0xAA, 0xFF !!!
// Note: for convenience, only 5 bytes addresses are supported
void nrf24l01_set_tx_address(const nrf24l01_context *ctx,
                             const uint8_t *new_address) {
  nrf24l01_write_register(ctx, TX_ADDR, new_address, 5);
}

void nrf24l01_set_rx_address(const nrf24l01_context *ctx, uint8_t pipe,
                             const uint8_t *new_address) {

  if (pipe > 5) {
    pipe = 5;
  }

  // Read register and enable pipe
  uint8_t buffer[2];
  nrf24l01_read_register(ctx, EN_RXADDR, buffer, 1);

  buffer[0] = buffer[1] | 1u << pipe;
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
  default:
    break;
  }
}

// Setup automatic retransmission
// delay - wait before retransmission - value in uS - 250 <= x <= 4000
// count - number of retransmissions - 0 <= x <= 15
void nrf24l01_setup_automatic_retransmission(const nrf24l01_context *ctx,
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

  uint8_t buffer[1] = {(uint8_t)(delay << 4u) | count};

  nrf24l01_write_register(ctx, SETUP_RETR, buffer, 1);
}

void nrf24l01_activate_dynamic_payload(const nrf24l01_context *ctx) {
  uint8_t buffer[2] = {0x73};
  //  nrf24l01_send_command_multiple(ctx, ACTIVATE, buffer, 1);

  // Read FEATURE register
  nrf24l01_read_register(ctx, FEATURE, buffer, 1);
  nrf24l01_feature.raw = buffer[1];
  nrf24l01_feature.en_dpl = true;

  // Toggle EN_DPL bit and set new value
  buffer[0] = nrf24l01_feature.raw;
  nrf24l01_write_register(ctx, FEATURE, buffer, 1);
}

void nrf24l01_enable_dynamic_payload(const nrf24l01_context *ctx,
                                     uint8_t pipe) {
  if (pipe > 5)
    pipe = 5;

  // Read register and enable pipe
  uint8_t buffer[2];
  nrf24l01_read_register(ctx, DYNPD, buffer, 1);

  buffer[0] = (uint8_t)(buffer[1] | 1u << pipe);
  nrf24l01_write_register(ctx, DYNPD, buffer, 1);
}

void nrf24l01_disable_dynamic_payload(const nrf24l01_context *ctx) {
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

void print_field(char *name, uint8_t reg, uint8_t pos, uint8_t length,
                 uint8_t def) {

  uint8_t length_mask = (uint8_t)(0xffu >> (8u - length));

  if (length > 0) {
    usart1_printf("\t%-11.11s\t%i\t%i\r\n", name, (reg >> pos) & length_mask,
                  def);
  } else {
    usart1_printf("\t%-11.11s\t- \t- \r\n", name);
  }
}

void nrf24l01_print_register_map(const nrf24l01_context *ctx) {
  nrf24l01_read_register(ctx, CONFIG, ctx->rx, 1);
  usart_printf(USART1, "[CONFIG] 0x%02x\r\n", ctx->rx[1]);
  print_field("[RESERVED]", ctx->rx[1], 7, 1, 0);
  print_field("MASK_RX_DR", ctx->rx[1], 6, 1, 0);
  print_field("MASK_RX_DS", ctx->rx[1], 5, 1, 0);
  print_field("MASK_MAX_RT", ctx->rx[1], 4, 1, 0);
  print_field("EN_CRC", ctx->rx[1], 3, 1, 1);
  print_field("CRCO", ctx->rx[1], 2, 1, 0);
  print_field("PWR_UP", ctx->rx[1], 1, 1, 0);
  print_field("PRIM_RX", ctx->rx[1], 0, 1, 0);

  nrf24l01_read_register(ctx, EN_AA, ctx->rx, 1);
  usart1_printf("[EN_AA] 0x%02x\r\n", ctx->rx[1]);
  print_field("[RESERVED]", ctx->rx[1], 6, 2, 0);
  print_field("ENAA_P5", ctx->rx[1], 5, 1, 1);
  print_field("ENAA_P4", ctx->rx[1], 4, 1, 1);
  print_field("ENAA_P3", ctx->rx[1], 3, 1, 1);
  print_field("ENAA_P2", ctx->rx[1], 2, 1, 1);
  print_field("ENAA_P1", ctx->rx[1], 1, 1, 1);
  print_field("ENAA_P0", ctx->rx[1], 0, 1, 1);

  nrf24l01_read_register(ctx, EN_RXADDR, ctx->rx, 1);
  usart1_printf("[EN_RXADDR] 0x%02x\r\n", ctx->rx[1]);
  print_field("[RESERVED]", ctx->rx[1], 6, 2, 0);
  print_field("ERX_P5", ctx->rx[1], 5, 1, 0);
  print_field("ERX_P4", ctx->rx[1], 4, 1, 0);
  print_field("ERX_P3", ctx->rx[1], 3, 1, 0);
  print_field("ERX_P2", ctx->rx[1], 2, 1, 0);
  print_field("ERX_P1", ctx->rx[1], 1, 1, 1);
  print_field("ERX_P0", ctx->rx[1], 0, 1, 1);

  nrf24l01_read_register(ctx, SETUP_AW, ctx->rx, 1);
  usart1_printf("[SETUP_AW] 0x%02x\r\n", ctx->rx[1]);
  print_field("[RESERVED]", ctx->rx[1], 2, 5, 0);
  print_field("AW", ctx->rx[1], 0, 2, 0b11);

  nrf24l01_read_register(ctx, SETUP_RETR, ctx->rx, 1);
  usart1_printf("[SETUP_RETR] 0x%02x\r\n", ctx->rx[1]);
  print_field("ARD", ctx->rx[1], 3, 4, 0);
  print_field("ARC", ctx->rx[1], 0, 4, 3);

  nrf24l01_read_register(ctx, RF_CH, ctx->rx, 1);
  usart1_printf("[SETUP_RETR] 0x%02x\r\n", ctx->rx[1]);
  print_field("[RESERVED]", ctx->rx[1], 7, 1, 0);
  print_field("RF_CH", ctx->rx[1], 0, 7, 0b10);

  nrf24l01_read_register(ctx, RF_SETUP, ctx->rx, 1);
  usart1_printf("[RF_SETUP] 0x%02x\r\n", ctx->rx[1]);
  print_field("CONT_WAVE", ctx->rx[1], 7, 1, 0);
  print_field("[RESERVED]", ctx->rx[1], 6, 1, 0);
  print_field("RF_DR_LOW", ctx->rx[1], 5, 1, 0);
  print_field("PLL_LOCK", ctx->rx[1], 4, 1, 0);
  print_field("RF_DR_HIGH", ctx->rx[1], 3, 1, 1);
  print_field("RF_PWR", ctx->rx[1], 1, 2, 0b11);
  print_field("[OBSOLETE]", ctx->rx[1], 0, 0, 0);

  nrf24l01_read_register(ctx, STATUS, ctx->rx, 1);
  usart1_printf("[STATUS] 0x%02x\r\n", ctx->rx[1]);
  print_field("[RESERVED]", ctx->rx[1], 7, 1, 0);
  print_field("RX_DR", ctx->rx[1], 6, 1, 0);
  print_field("TX_DS", ctx->rx[1], 5, 1, 0);
  print_field("MAX_RT", ctx->rx[1], 4, 1, 0);
  print_field("RX_P_NO", ctx->rx[1], 1, 3, 0b111);
  print_field("TX_FULL", ctx->rx[1], 0, 1, 0);

  nrf24l01_read_register(ctx, OBSERVE_TX, ctx->rx, 1);
  usart1_printf("[OBSERVE_TX] 0x%02x\r\n", ctx->rx[1]);
  print_field("PLOS_CNT", ctx->rx[1], 3, 4, 0);
  print_field("ARC_CNT", ctx->rx[1], 0, 4, 0);

  nrf24l01_read_register(ctx, RPD, ctx->rx, 1);
  usart1_printf("[RPD] 0x%02x\r\n", ctx->rx[1]);
  print_field("RPD", ctx->rx[1], 0, 1, 0);

  nrf24l01_read_register(ctx, RX_ADDR_P0, ctx->rx, 5);
  usart1_printf("[RX_ADDR_P0] 0x%02x%02x%02x%02x%02x\t%c%c%c%c%c\r\n",
                ctx->rx[1], ctx->rx[2], ctx->rx[3], ctx->rx[4], ctx->rx[5],
                ctx->rx[1], ctx->rx[2], ctx->rx[3], ctx->rx[4], ctx->rx[5]);
  nrf24l01_read_register(ctx, RX_ADDR_P1, ctx->rx, 5);
  usart1_printf("[RX_ADDR_P1] 0x%02x%02x%02x%02x%02x\t%c%c%c%c%c\r\n",
                ctx->rx[1], ctx->rx[2], ctx->rx[3], ctx->rx[4], ctx->rx[5],
                ctx->rx[1], ctx->rx[2], ctx->rx[3], ctx->rx[4], ctx->rx[5]);
  nrf24l01_read_register(ctx, RX_ADDR_P2, ctx->rx, 1);
  usart1_printf("[RX_ADDR_P2]           %02x\t    %c\r\n", ctx->rx[1],
                ctx->rx[1]);
  nrf24l01_read_register(ctx, RX_ADDR_P3, ctx->rx, 1);
  usart1_printf("[RX_ADDR_P3]           %02x\t    %c\r\n", ctx->rx[1],
                ctx->rx[1]);
  nrf24l01_read_register(ctx, RX_ADDR_P4, ctx->rx, 1);
  usart1_printf("[RX_ADDR_P4]           %02x\t    %c\r\n", ctx->rx[1],
                ctx->rx[1]);
  nrf24l01_read_register(ctx, RX_ADDR_P5, ctx->rx, 1);
  usart1_printf("[RX_ADDR_P5]           %02x\t    %c\r\n", ctx->rx[1],
                ctx->rx[1]);

  nrf24l01_read_register(ctx, TX_ADDR, ctx->rx, 5);
  usart1_printf("[TX_ADDR]    0x%02x%02x%02x%02x%02x\t%c%c%c%c%c\r\n",
                ctx->rx[1], ctx->rx[2], ctx->rx[3], ctx->rx[4], ctx->rx[5],
                ctx->rx[1], ctx->rx[2], ctx->rx[3], ctx->rx[4], ctx->rx[5]);

  nrf24l01_read_register(ctx, RX_PW_P0, ctx->rx, 1);
  usart1_printf("[RX_PW_P0] 0x%02x\r\n", ctx->rx[1]);
  print_field("[RESERVED]", ctx->rx[1], 6, 2, 0);
  print_field("RX_PW_P0", ctx->rx[1], 0, 5, 0);

  nrf24l01_read_register(ctx, RX_PW_P1, ctx->rx, 1);
  usart1_printf("[RX_PW_P1] 0x%02x\r\n", ctx->rx[1]);
  print_field("[RESERVED]", ctx->rx[1], 6, 2, 0);
  print_field("RX_PW_P1", ctx->rx[1], 0, 5, 0);

  nrf24l01_read_register(ctx, RX_PW_P2, ctx->rx, 1);
  usart1_printf("[RX_PW_P2] 0x%02x\r\n", ctx->rx[1]);
  print_field("[RESERVED]", ctx->rx[1], 6, 2, 0);
  print_field("RX_PW_P2", ctx->rx[1], 0, 5, 0);

  nrf24l01_read_register(ctx, RX_PW_P3, ctx->rx, 1);
  usart1_printf("[RX_PW_P3] 0x%02x\r\n", ctx->rx[1]);
  print_field("[RESERVED]", ctx->rx[1], 6, 2, 0);
  print_field("RX_PW_P3", ctx->rx[1], 0, 5, 0);

  nrf24l01_read_register(ctx, RX_PW_P4, ctx->rx, 1);
  usart1_printf("[RX_PW_P4] 0x%02x\r\n", ctx->rx[1]);
  print_field("[RESERVED]", ctx->rx[1], 6, 2, 0);
  print_field("RX_PW_P4", ctx->rx[1], 0, 5, 0);

  nrf24l01_read_register(ctx, RX_PW_P5, ctx->rx, 1);
  usart1_printf("[RX_PW_P5] 0x%02x\r\n", ctx->rx[1]);
  print_field("[RESERVED]", ctx->rx[1], 6, 2, 0);
  print_field("RX_PW_P5", ctx->rx[1], 0, 5, 0);

  nrf24l01_read_register(ctx, FIFO_STATUS, ctx->rx, 1);
  usart1_printf("[FIFO_STATUS] 0x%02x\r\n", ctx->rx[1]);
  print_field("[RESERVED]", ctx->rx[1], 7, 1, 0);
  print_field("TX_REUSE", ctx->rx[1], 6, 1, 0);
  print_field("TX_FULL", ctx->rx[1], 5, 1, 0);
  print_field("TX_EMPTY", ctx->rx[1], 4, 1, 1);
  print_field("[RESERVED]", ctx->rx[1], 2, 2, 0);
  print_field("RX_FULL", ctx->rx[1], 1, 1, 0);
  print_field("RX_EMPTY", ctx->rx[1], 0, 1, 1);

  nrf24l01_read_register(ctx, DYNPD, ctx->rx, 1);
  usart1_printf("[DYNPD] 0x%02x\r\n", ctx->rx[1]);
  print_field("[RESERVED]", ctx->rx[1], 6, 1, 0);
  print_field("DPL_P5", ctx->rx[1], 5, 1, 0);
  print_field("DPL_P4", ctx->rx[1], 4, 1, 0);
  print_field("DPL_P3", ctx->rx[1], 3, 1, 0);
  print_field("DPL_P2", ctx->rx[1], 2, 1, 0);
  print_field("DPL_P1", ctx->rx[1], 1, 1, 0);
  print_field("DPL_P0", ctx->rx[1], 0, 1, 0);

  nrf24l01_read_register(ctx, FEATURE, ctx->rx, 1);
  usart1_printf("[FEATURE] 0x%02x\r\n", ctx->rx[1]);
  print_field("[RESERVED]", ctx->rx[1], 3, 5, 0);
  print_field("EN_DPL", ctx->rx[1], 2, 1, 0);
  print_field("EN_ACK_PAY", ctx->rx[1], 1, 1, 0);
  print_field("EN_DYN_ACK", ctx->rx[1], 0, 1, 0);
}
