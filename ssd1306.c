#include "ssd1306.h"

static uint8_t tx[33];
//static uint8_t rx[2];
static uint8_t _buffer [64*128/8] = {0, };
void ssd1306_command(i2c_device *device, uint8_t command);

void ssd1306_command(i2c_device *device, uint8_t command) {
    tx[0] = 0x00;  // command
    tx[1] = command;

    bool error = i2c_master_write(device->i2c, device->address, tx, 2);
    if (error)
        hacf();

}

// Perform initialization
// Taken from: https://github.com/adafruit/Adafruit_SSD1306/
i2c_device *ssd1306_init(uint32_t i2c) {
//    I2C_LowLevel_Init(I2Cx);

    i2c_device *dev = malloc(sizeof(i2c_device));
    dev->i2c = i2c;
    dev->address = SSD1306_ADDRESS;

    tx[0] = 0x00;
    tx[1] = SSD1306_DISPLAY_OFF;
    tx[2] = SSD1306_SET_DISPLAY_CLOCK_DIVIDE;
    tx[3] = 0x80;
    tx[4] = SSD1306_SET_MULTIPLEX_RATIO;
    tx[5] = 64 - 1;
    bool error = i2c_master_write(dev->i2c, dev->address , tx, 6);
    if (error)
        hacf();

    tx[1] = SSD1306_SET_DISPLAY_OFFSET;
    tx[2] = 0x00;
    tx[3] = SSD1306_SET_START_LINE | 0x00;
    tx[4] = SSD1306_CHARGE_PUMP;
    tx[5] = 0x14;

    error = i2c_master_write(dev->i2c, dev->address , tx, 6);
    if (error)
        hacf();

    tx[1] = 0x20;
    tx[2] = 0x00;
    tx[3] = 0xA0 | 0x1;
    tx[4] = 0xC8;

    error = i2c_master_write(dev->i2c, dev->address , tx, 5);
    if (error)
        hacf();

    tx[1] = 0xDA;
    tx[2] = 0x12;
    tx[3] = 0x81;
    tx[4] = 0xCF;

    error = i2c_master_write(dev->i2c, dev->address , tx, 5);
    if (error)
        hacf();

    tx[1] = 0xD9;
    tx[2] = 0xF1;
    tx[3] = 0xDB;
    tx[4] = 0x40;
    tx[5] = 0xA4;
    tx[6] = 0xA6;
    tx[7] = 0x2E;
    tx[8] = 0xAF;
    tx[8] = 0xA4;

    error = i2c_master_write(dev->i2c, dev->address , tx, 10);
    if (error)
        hacf();

    return dev;
}


// print buffer
void ssd1306_print_buffer(i2c_device *dev) {
    tx[0] = 0x00;
    tx[1] = 0x22;
    tx[2] = 0x00;
    tx[3] = 0xFF;
    tx[4] = 0x21;
    tx[5] = 0x00;
    tx[6] = 128 - 1;

    bool error = i2c_master_write(dev->i2c, dev->address , tx, 7);
    if (error)
        hacf();

    uint16_t bytes_sent = 0;
    tx[0] = 0x40;
    while (bytes_sent < sizeof(_buffer)) {
        memcpy(&tx[1], &_buffer[0 + bytes_sent], 32);
        error = i2c_master_write(dev->i2c, dev->address , tx, sizeof(tx));
        if (error)
            hacf();
        bytes_sent += 32;
    }
}

void ssd1306_draw_pixel(uint16_t x, uint16_t y) {
    _buffer[x + (y/8) * 128] |= (1 << (y & 7));
}

void ssd1306_clear_pixel(uint16_t x, uint16_t y) {
    _buffer[x + (y/8) * 128] &= ~(1 << (y & 7));
}

// print checkers on display
void ssd1306_demo_checker(i2c_device *dev) {
    // fill buffer
    for (uint16_t i = 0; i < (64 * (128/8)); ++i){
        if (i % 2 == 0)
            _buffer[i] = 0x55;
        else
            _buffer[i] = 0xAA;
    }

    ssd1306_print_buffer(dev);
}
