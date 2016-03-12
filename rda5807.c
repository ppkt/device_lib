#include "rda5807.h"

static u8 tx[] = {0x02,
        0xF4, 0x01, // 0x02h
        0x00, 0x00,
        0x00, 0x00,
        0x88, 0x0F, //0x05h
        0x00, 0x00,
        0x7C, 0x12};

static u8 rx[8];

void rda5807_init(TIM_TypeDef *timer) {
    tx[0] = 0x00;
    I2C_Master_BufferWrite(I2C1, tx, 1, DMA, RDA5970_ADDRESS << 1);
    I2C_Master_BufferRead(I2C1, rx, 2, DMA, RDA5970_ADDRESS << 1);

    if (rx[0] == RDA5970_ID) {
//        printf("Chip is present\n\r");
    }

    // Reset radio
    tx[0] = 0x02;
    tx[1] = 0x00;
    tx[2] = 0x03;
    I2C_Master_BufferWrite(I2C1, tx, 3, DMA, RDA5970_ADDRESS << 1);
    delay_ms(timer, 500);

    // Enable
    tx[1] = 0xC0;
    tx[2] = 0x0D;
    I2C_Master_BufferWrite(I2C1, tx, 3, DMA, RDA5970_ADDRESS << 1);
    delay_ms(timer, 500);

//    printf("Configuration done\r\n");
}

void rda5807_set_frequency(u16 new_frequency) {
    u16 freq = new_frequency;
    u16 freqB = freq - 870;
    u8 freqH = freqB >> 2;
    u8 freqL = (freqB & 3) << 6; // Shift channel selection for matching register 0x03

    tx[0] = 0x03;
    tx[1] = freqH;
    tx[2] = freqL + 0x10;
    I2C_Master_BufferWrite(I2C1, tx, 3, DMA, RDA5970_ADDRESS << 1);
}

bool rda5807_toggle_mute() {
    // Read current data
    tx[0] = 0x02;
    I2C_Master_BufferWrite(I2C1, tx, 1, DMA, RDA5970_ADDRESS << 1);
    I2C_Master_BufferRead(I2C1, tx+1, 2, DMA, RDA5970_ADDRESS << 1);

    // Toggle mute bit
    tx[1] = tx[1] ^ 1 << 6;
    I2C_Master_BufferWrite(I2C1, tx, 3, DMA, RDA5970_ADDRESS << 1);

    // Return new mute value
    return !(tx[1] & 1 << 6);
}

void rda5807_set_mute(bool value) {
    value = !value;

    // Read current data
    tx[0] = 0x02;
    I2C_Master_BufferWrite(I2C1, tx, 1, DMA, RDA5970_ADDRESS << 1);
    I2C_Master_BufferRead(I2C1, tx+1, 2, DMA, RDA5970_ADDRESS << 1);

    // Set mute bit
    tx[1] ^= (-value ^ tx[1]) & (1 << 6);
    I2C_Master_BufferWrite(I2C1, tx, 3, DMA, RDA5970_ADDRESS << 1);
}

bool rda5807_toggle_bass_boost() {
    // Read current data
    tx[0] = 0x02;
    I2C_Master_BufferWrite(I2C1, tx, 1, DMA, RDA5970_ADDRESS << 1);
    I2C_Master_BufferRead(I2C1, tx+1, 2, DMA, RDA5970_ADDRESS << 1);

    // Toggle bass bit
    tx[1] = tx[1] ^ 1 << 4;
    I2C_Master_BufferWrite(I2C1, tx, 3, DMA, RDA5970_ADDRESS << 1);

    // Return new bass value
    return (tx[1] & 1 << 4);

}

void rda5807_set_bass_boost(bool value) {
    // Read current data
    tx[0] = 0x02;
    I2C_Master_BufferWrite(I2C1, tx, 1, DMA, RDA5970_ADDRESS << 1);
    I2C_Master_BufferRead(I2C1, tx+1, 2, DMA, RDA5970_ADDRESS << 1);

    // Set mute bit
    tx[1] ^= (-value ^ tx[1]) & (1 << 4);
    I2C_Master_BufferWrite(I2C1, tx, 3, DMA, RDA5970_ADDRESS << 1);
}

void rda5807_set_volume(u8 new_volume) {
    // Read current data
    tx[0] = 0x05;
    I2C_Master_BufferWrite(I2C1, tx, 1, DMA, RDA5970_ADDRESS << 1);
    I2C_Master_BufferRead(I2C1, tx+1, 2, DMA, RDA5970_ADDRESS << 1);

    tx[2] = (tx[2] & 0xF0) | new_volume;

    I2C_Master_BufferWrite(I2C1, tx, 3, DMA, RDA5970_ADDRESS << 1);
}
