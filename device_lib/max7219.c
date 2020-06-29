#include <common_lib/utils.h>
#include "max7219.h"

// TODO(ppkt): Maybe move this to spi_device structure?
static uint8_t data[8];

/**
 * Send one byte of data to selected address
 */
static void
max7219_send(const spi_device *device, uint8_t address, uint8_t value) {
    uint8_t tx[] = {address, value};
    spi_send_recv(device->spi, tx, NULL, sizeof(tx) / sizeof(tx[0]));
}

spi_device *
max7219_init(uint32_t spi) {
    spi_device *device = malloc(sizeof(spi_device));
    device->spi = spi;

    max7219_reset(device);

    // Enable all channels
    max7219_send(device, REG_SCAN_LIMIT, 7);

    max7219_clear_display(device);

    return device;
}

void
max7219_reset(const spi_device* device) {
    // Shutdown
    max7219_send(device, REG_SHUTDOWN, 0x00);

    // Normal operation
    max7219_send(device, REG_SHUTDOWN, 0x01);
}

void
max7219_self_test(const spi_device* device) {
    // Turn on all LEDs
    max7219_send(device, REG_TEST, 0x01);

    delay_ms(500);

    // Return to normal operation
    max7219_send(device, REG_TEST, 0x00);
}

void
max7219_clear_display(const spi_device* device) {
    for (uint8_t row = 0; row < 8; ++row) {
        data[row] = 0x00;
        max7219_send(device, REG_CHANNEL0 + row, data[row]);
    }
}

void
max7219_turn_all(const spi_device* device) {
    for (uint8_t row = 0; row < 8; ++row) {
        data[row] = 0xFF;
        max7219_send(device, REG_CHANNEL0 + row, data[row]);
    }
}

void
max7219_set_brightness(const spi_device* device, uint8_t new_value) {
    if (new_value > 0x0F) {
        new_value = 0x0F;
    }
    max7219_send(device, REG_INTENSITY, new_value);
}

/**
 * Set new value for single LED
 */
static void
max7219_new_value(const spi_device* device, uint8_t row, uint8_t column, bool value) {
    if (row >= 8 || column >= 8) return;

    // columns ale numbered from right, so we have invert them
    column = (uint8_t) (7 - column);

    if (value)
        data[row] |= 1 << column;
    else
        data[row] &= ~(1 << column);

    max7219_send(device, REG_CHANNEL0 + row, data[row]);
}

void
max7219_set_led(const spi_device* device, uint8_t row, uint8_t column) {
    max7219_new_value(device, row, column, true);
}

void
max7219_reset_led(const spi_device* device, uint8_t row, uint8_t column) {
    max7219_new_value(device, row, column, false);
}

void
max7219_set_data(const spi_device* device, uint8_t new_data[8]) {
    for (uint8_t row = 0; row < 8; ++row) {
        data[row] = new_data[row];
        max7219_send(device, REG_CHANNEL0 + row, data[row]);
    }
}
