#include "hd44780-i2c.h"

/*
 * Useful commands:
 * 0x1E - Scroll display one character right (every line): [  Dupa] -> [a  Dup]
 * 0x18 - Scroll display one character left (every line)
 * 0x10 - Move cursor one character left
 * 0x14 - Move cursor one character right
 * 0x02 - Go cursor home
 * 0x0E - Underlined cursor
 * 0x0F - Blinked block cursor
 * 0x0C - Invisible cursor
 * 0x08 - Hide display
 * 0x0C - Show display with invisible cursor
 * 0x01 - Clear display
 */

static uint8_t hd44780_data[4] = {0x00, 0x00, 0x00, 0x00};

static error_t hd44780_send(const hd44780_device *device, uint8_t cmd,
                            bool set_rs) {
  if (!device) {
    return E_NULL_PTR;
  }

  uint8_t rs_ = 0;
  if (set_rs)
    rs_ = HD44780_RS;

  hd44780_data[0] = (cmd & 0xF0u) | HD44780_BACKLIGHT | HD44780_EN | rs_;
  hd44780_data[1] = (cmd & 0xF0u) | HD44780_BACKLIGHT;
  hd44780_data[2] = (cmd & 0x0Fu) << 4u | HD44780_BACKLIGHT | HD44780_EN | rs_;
  hd44780_data[3] = (cmd & 0x0Fu) << 4u | HD44780_BACKLIGHT;
  return i2c_master_write(device->device.i2c, device->device.address,
                          hd44780_data, 4);
}

static error_t hd44780_char(const hd44780_device *device, uint8_t cmd) {
  if (!device) {
    return E_NULL_PTR;
  }

  check_error(hd44780_send(device, cmd, true));

  delay_us(device->timer, 200);
  return E_SUCCESS;
}

static error_t hd44780_cmd(const hd44780_device *device, uint8_t cmd) {
  if (!device) {
    return E_NULL_PTR;
  }

  check_error(hd44780_send(device, cmd, false));

  delay_us(device->timer, 5000);
  return E_SUCCESS;
}

error_t hd44780_print(const hd44780_device *device, const char *string) {
  if (!device || !string) {
    return E_NULL_PTR;
  }

  const char *ptr = string;
  while (*(ptr) != 0) {
    check_error(hd44780_char(device, *ptr++));
  }
  return E_SUCCESS;
}

error_t hd44780_backlight(const hd44780_device *device, bool new_value) {
  if (!device) {
    return E_NULL_PTR;
  }

  uint8_t backlight = new_value << 3u;
  check_error(i2c_master_transaction_write_read(
      device->device.i2c, device->device.address, NULL, 0, hd44780_data, 1));

  hd44780_data[0] |= backlight;
  check_error(i2c_master_transaction_write_read(
      device->device.i2c, device->device.address, hd44780_data, 1, NULL, 0));
  return E_SUCCESS;
}

error_t hd44780_go_to_line(const hd44780_device *device, uint8_t line) {
  if (!device) {
    return E_NULL_PTR;
  }

  switch (line) {
  case 0:
    check_error(hd44780_cmd(device, 0x80));
    break;
  case 1:
    check_error(hd44780_cmd(device, 0xC0));
    break;
  case 2:
    check_error(hd44780_cmd(device, 0x94));
    break;
  case 3:
    check_error(hd44780_cmd(device, 0xD4));
    break;
  default:
    hacf();
  }
  return E_SUCCESS;
}

error_t hd44780_go_to(const hd44780_device *device, uint8_t row, uint8_t col) {
  if (!device) {
    return E_NULL_PTR;
  }

  check_error(hd44780_go_to_line(device, row));

  uint8_t i = 0;
  while (i++ < col) {
    check_error(hd44780_cmd(device, HD44780_MOVE_CURSOR_RIGHT));
  }
  return E_SUCCESS;
}

error_t hd44780_cgram_write(const hd44780_device *device, uint8_t pos,
                            uint8_t data_[8]) {
  if (!device) {
    return E_NULL_PTR;
  }

  if (pos > 7) {
    return E_VALUE_INVALID;
  }

  pos = 64 + pos * 8;
  check_error(hd44780_cmd(device, pos));

  uint8_t i;
  for (i = 0; i < 8; ++i) {
    check_error(hd44780_send(device, data_[i], true));
  }

  return E_SUCCESS;
}

error_t hd44780_init(hd44780_device *device, uint32_t i2c, uint8_t address,
                     uint32_t timer) {
  if (!device) {
    return E_NULL_PTR;
  }

  check_error(i2c_check_arguments(i2c, address));

  device->device.i2c = i2c;
  device->device.address = address;
  device->timer = timer;

  // Setup clock
  setup_delay_timer(timer);

  hd44780_data[0] = 0x00u | HD44780_BACKLIGHT;

  // Reset all
  check_error(i2c_master_write(device->device.i2c, device->device.address,
                               hd44780_data, 1));

  check_error(hd44780_cmd(device, 0x03));
  delay_us(device->timer, 5000);

  check_error(hd44780_cmd(device, 0x03));
  delay_us(device->timer, 100);

  check_error(hd44780_cmd(device, HD44780_MOVE_TO_HOME));
  delay_us(device->timer, 200);

  // 4 bit mode
  check_error(hd44780_cmd(device, 0x28));

  // set direction of cursor to right
  check_error(hd44780_cmd(device, 0x06));

  // clear display, go to 0x0
  check_error(hd44780_cmd(device, HD44780_DISPLAY_ERASE));

  // turn on display, set invisiblecursor
  check_error(hd44780_cmd(device, HD44780_DISPLAY_SHOW));

  // clear display, go to 0x0
  check_error(hd44780_cmd(device, 0x01));

  return E_SUCCESS;
}
