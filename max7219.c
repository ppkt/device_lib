#include "max7219.h"

static uint8_t data[8];

// Set pin as soft NSS
void max7219_gpio_configuration(void) {
    // Configure SPI1 NSS pin
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

// Send one byte of data to selected address
void max7219_send(uint8_t address, uint8_t value) {
    static uint8_t tx[2];
    static uint8_t rx[2];

    tx[0] = address;
    tx[1] = value;
    GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_RESET);
    spi_send(SPI1, tx, rx, 2);
    GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_SET);
}

// Initialize module and peripherals
void max7219_init(void) {
    spi_init(SPI1);
    max7219_gpio_configuration();

    max7219_reset();

    // Enable all channels
    max7219_send(REG_SCAN_LIMIT, 7);

    max7219_clear_display();
}

// Perform reset operation
void max7219_reset(void) {
    // Shutdown
    max7219_send(REG_SHUTDOWN, 0x00);

    // Normal operation
    max7219_send(REG_SHUTDOWN, 0x01);
}

// Turn on briefly all LEDs on board
void max7219_self_test(void) {
    // Turn on all LEDs
    max7219_send(REG_TEST, 0x01);

    uint32_t i;
    for (i = 0; i < 0x200000; ++i);

    // Return to normal operation
    max7219_send(REG_TEST, 0x00);
}

// Turn off all LEDs
void max7219_clear_display(void) {
    static uint8_t row;
    for (row = 0; row < 8; ++row) {
        data[row] = 0x00;
        max7219_send(REG_CHANNEL0 + row, data[row]);
    }
}

// Turn on all LEDs
void max7219_turn_all(void) {
    static uint8_t row;
    for (row = 0; row < 8; ++row) {
        data[row] = 0xFF;
        max7219_send(REG_CHANNEL0 + row, data[row]);
    }
}

// Set new intensity of LEDs
void max7219_set_brightness(uint8_t new_value) {
    if (new_value > 0x0F) {
        new_value = 0x0F;
    }
    max7219_send(REG_INTENSITY, new_value);
}

// Set new value for single LED
static void max7219_new_value(uint8_t row, uint8_t column, bool value) {
    if (row >= 8 || column >= 8) return;

    // columns ale numbered from right, so we have invert them
    column = 7 - column;

    if (value)
        data[row] |= 1 << column;
    else
        data[row] &= ~(1 << column);

    max7219_send(REG_CHANNEL0 + row, data[row]);
}

// Turn on single LED
void max7219_set_led(uint8_t row, uint8_t column) {
    max7219_new_value(row, column, true);
}

// Turn off single LED
void max7219_reset_led(uint8_t row, uint8_t column) {
    max7219_new_value(row, column, false);
}

// Draw whole 8x8 segment
void max7219_set_data(uint8_t new_data[8]) {
    static uint8_t row;
    for (row = 0; row < 8; ++row) {
        data[row] = new_data[row];
        max7219_send(REG_CHANNEL0 + row, data[row]);
    }
}
