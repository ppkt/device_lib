#include "si4703.h"

char* pty_code[] = {
  "NO_PROGRAMME",
  "NEWS",
  "CURRRENT_AFFAIRS",
  "INFORMATION",
  "SPORT",
  "EDUCATION",
  "DRAMA",
  "CULTURE",
  "SCIENCE",
  "VARIED",
  "POPULAR_MUSIC",
  "ROCK_MUSIC",
  "EASY_LISTENING",
  "LIGHT_CLASSICAL",
  "SERIOUS_CLASSICAL",
  "OTHER_MUSIC",
  "WEATHER",
  "FINANCE",
  "CHILDRENS_PROGRAMMES",
  "SOCIAL_AFFAIRS",
  "RELIGION",
  "PHONE_IN",
  "TRAVEL",
  "LEISURE",
  "JAZZ_MUSIC",
  "COUNTRY_MUSIC",
  "NATIONAL_MUSIC",
  "OLDIES_MUSIC",
  "FOLK_MUSIC",
  "DOCUMENTARY",
  "ALARM_TEST",
  "ALARM",
};

error_t si4703_read_memory(si4703_device *device) {
  uint8_t tmp_buffer[32];
  i2c_transfer7(device->i2c.i2c, device->i2c.address, NULL, 0, tmp_buffer, 32);

  for (uint8_t i = 0; i < 16; ++i) {
    // rx buffer starts at 0x0A register and wraps at 0x0F
    device->memory[(i + 0x0A) % 0x10] =
        (uint16_t)(tmp_buffer[i * 2] << 8u) | tmp_buffer[i * 2 + 1];
  }

  return E_SUCCESS;
}

error_t si4703_write_memory(si4703_device *device, uint8_t register_id) {
  // writing starts at 0x02 and ends at register_id (including)
  // registers are 2 bytes wide
  uint8_t registers_to_write = register_id - 0x02 + 1;
  uint8_t *tx = malloc(registers_to_write * 2);

  for (uint8_t i = 0; i < registers_to_write; ++i) {
    tx[i * 2] = device->memory[i + 0x02] >> 8u;
    tx[i * 2 + 1] = device->memory[i + 0x02] & 0xFFu;

    debug_printf("memory[0x%02x] = 0x%04x", i + 0x02, device->memory[i + 0x02]);
  }

  i2c_transfer7(device->i2c.i2c, device->i2c.address, tx,
                registers_to_write * 2, NULL, 0);
  free(tx);
  return E_SUCCESS;
}

error_t si4703_set_channel(si4703_device *device, uint16_t frequency) {
  uint16_t channel = frequency - 875;

  // set channel and turn on bit for start tunning in
  device->memory[SI4703_CHANNEL_REG] = SI4703_CHANNEL_TUNE | channel;
  check_error(si4703_write_memory(device, SI4703_CHANNEL_REG));

  // wait for tuning to complete, time value from datasheet
  delay_ms(60);
  uint16_t cnt = 0;
  do {
    check_error(si4703_read_memory(device));
    if (++cnt == 0xFF) {
      debug_print("Timeout reached during tuning");
      return E_TIMEOUT;
    }
  } while ((device->memory[SI4703_STATUSRSSI_REG] & SI4703_STATUSRSSI_STC) ==
           0);

  // print details
  debug_print("Tuning completed");
  if (device->memory[SI4703_STATUSRSSI_REG] & SI4703_STATUSRSSI_ST) {
    debug_print("Stereo");
  } else {
    debug_print("Mono");
  }
  debug_printf("RSSI: %d", device->memory[SI4703_STATUSRSSI_REG] & 0xFFu);

  uint16_t current_channel = device->memory[SI4703_READCHAN_REG] & 0x03FFu;
  if (current_channel)
    current_channel += 875;

  debug_printf("Current channel: %d", current_channel);

  // clear tune bit
  device->memory[SI4703_CHANNEL_REG] &= ~SI4703_CHANNEL_TUNE;
  check_error(si4703_write_memory(device, SI4703_CHANNEL_REG));

  // wait until device will clear STC bit
  cnt = 0;
  do {
    check_error(si4703_read_memory(device));
    if (++cnt == 0xFF) {
      debug_print("Timeout reached during tuning");
      return E_TIMEOUT;
    }
  } while ((device->memory[SI4703_STATUSRSSI_REG] & SI4703_STATUSRSSI_STC));

  return E_SUCCESS;
}

static void si4703_gpio_setup(const pin *rst, const pin *sen) {
  // Enable clocks
  rcc_periph_clock_enable(port_to_rcc(rst->port));
  rcc_periph_clock_enable(port_to_rcc(sen->port));

  // Set pins to 'output push-pull'
  gpio_set_mode(rst->port, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL,
                rst->gpio);
  gpio_set_mode(sen->port, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL,
                sen->gpio);
  gpio_clear(rst->port, rst->gpio);
  gpio_clear(sen->port, sen->gpio);
}

error_t si4703_init(si4703_device *device, uint32_t i2c, const pin *rst,
                    const pin *sen, uint32_t timer) {
  device->i2c.i2c = i2c;
  device->i2c.address = 0x10;

  setup_delay_timer(timer);
  si4703_gpio_setup(rst, sen);

  // initialize device in 2-wire operation
  delay_us(timer, 100);
  gpio_set(sen->port, sen->gpio);
  delay_us(timer, 100);
  gpio_set(rst->port, rst->gpio);

  if (!i2c_check_presence(device->i2c.i2c, device->i2c.address)) {
    return E_NOT_FOUND;
  }

  si4703_chip_id chip_id_reg;
  si4703_device_id device_id_reg;

  check_error(si4703_read_memory(device));

  // Enable the oscillator
  print_variable_hex(device->memory[SI4703_TEST1_REG]);
  device->memory[SI4703_TEST1_REG] |= SI4703_TEST1_XOSCEN;
  print_variable_hex(device->memory[SI4703_TEST1_REG]);

  check_error(si4703_write_memory(device, SI4703_TEST1_REG));

  // Wait for oscillator to power up
  delay_ms(500);

  // Enable the device
  check_error(si4703_read_memory(device));
  device->memory[SI4703_POWERCFG_REG] =
      //      SI4703_POWERCFG_DSMUTE |
      SI4703_POWERCFG_DMUTE | SI4703_POWERCFG_ENABLE;
  print_variable_hex(device->memory[SI4703_POWERCFG_REG]);

  check_error(si4703_write_memory(device, SI4703_POWERCFG_REG));

  // Wait for device to power up (according to datasheet)
  delay_ms(110);

  check_error(si4703_read_memory(device));

  device_id_reg.raw = device->memory[SI4703_DEVICEID_REG];
  if (device_id_reg.pn != 0x01 || device_id_reg.mfgid != 0x242) {
    return E_NOT_FOUND;
  }

  chip_id_reg.raw = device->memory[SI4703_CHIPID_REG];
  print_variable_hex(chip_id_reg.dev);
  if (chip_id_reg.dev == 0 || chip_id_reg.dev == 1) {
    debug_print("Si4702");
  } else if (chip_id_reg.dev == 0b1000 || chip_id_reg.dev == 0b1001) {
    debug_print("Si4703");
  }
  debug_printf("Revision %d", chip_id_reg.rev);
  if (chip_id_reg.dev & 1u) {
    debug_printf("Firmware version: %d", chip_id_reg.firmware);
  } else {
    debug_print("Device DISABLED");
  }
  debug_printf("Firmware version: %d", chip_id_reg.firmware);

  // Set De-emphasis to 50 us (Europe)
  device->memory[SI4703_SYSCONFIG1_REG] |= SI4703_SYSCONFIG1_DE;

  // Setup the device
  si4703_sysconfig2_reg sysconfig_2 = {
      .raw = device->memory[SI4703_SYSCONFIG2_REG]};
  sysconfig_2.band = 0x00;  // Europe
  sysconfig_2.space = 0x01; // Europe
  sysconfig_2.volume = 0x0001;
  device->memory[SI4703_SYSCONFIG2_REG] = sysconfig_2.raw;

  check_error(si4703_write_memory(device, SI4703_SYSCONFIG2_REG));

  return E_SUCCESS;
}
