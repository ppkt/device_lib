#include "rda5807.h"

static u8 tx[] = {0x02,
        0xF4, 0x01, // 0x02h
        0x00, 0x00,
        0x00, 0x00,
        0x88, 0x0F, //0x05h
        0x00, 0x00,
        0x7C, 0x12};

static u8 rx[64];

void rda5807_init() {
    tx[0] = 0x00;
    I2C_Master_BufferWrite(I2C1, tx, 1, DMA, RDA5970_ADDRESS << 1);
    I2C_Master_BufferRead(I2C1, rx, 2, DMA, RDA5970_ADDRESS << 1);

    if (rx[0] == RDA5970_ID) {
        printf("Chip is present\n\r");
    }

    // Reset radio
    tx[0] = 0x02;
    tx[1] = 0x00;
    tx[2] = 0x03;
    I2C_Master_BufferWrite(I2C1, tx, 3, DMA, RDA5970_ADDRESS << 1);
    delay(500);

    // Enable
    tx[1] = 0xC0;
    tx[2] = 0x0D;
    I2C_Master_BufferWrite(I2C1, tx, 3, DMA, RDA5970_ADDRESS << 1);
    delay(500);

    delay(100);
    printf("Configuration done\r\n");
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
//    delay(500);
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

void rda5807_set_volume(u8 new_volume) {
    printf("%d\r\n", new_volume);

    // Read current data
    tx[0] = 0x05;
    I2C_Master_BufferWrite(I2C1, tx, 1, DMA, RDA5970_ADDRESS << 1);
    I2C_Master_BufferRead(I2C1, tx+1, 2, DMA, RDA5970_ADDRESS << 1);

    printf("%X\r\n", tx[2]);
    tx[2] = (tx[2] & 0xF0) | new_volume;
    printf("%X\r\n", tx[2]);

    I2C_Master_BufferWrite(I2C1, tx, 3, DMA, RDA5970_ADDRESS << 1);
}

/* Attempt to use RDS, doesn't work
u16 b1, b2, b3, b4;

u16 old_pi = 0, pi = 0;
u8 group_type_code, b0, tp, pty, individual;
u8 segment = 0, segment_old = 0;
void rda5807_print_rds(void) {
    // Get RDS data
    tx[0] = 0x0C;
    I2C_Master_BufferWrite(I2C1, tx, 1, DMA, RDA5970_ADDRESS << 1);
    I2C_Master_BufferRead(I2C1, rx, 8, DMA, RDA5970_ADDRESS << 1);

    b1 = rx[0] << 8 | rx[1];
    b2 = rx[2] << 8 | rx[3];
    b3 = rx[4] << 8 | rx[5];
    b4 = rx[6] << 8 | rx[7];

    pi = b1;
    if (old_pi != pi) {
        printf("PI: %4x\r\n", pi);
        old_pi = pi;
    }

    group_type_code = b2 >> 12;
    b0 = b2 >> 11 & 1;
    tp = b2 >> 10 & 1;
    pty = b2 >> 5 & 0b11111;
    individual = b2 & 0b11111;

    static char PS[9];
    PS[8] = 0;


    if (group_type_code == 0 && b0 == 0) {
        // A0
        segment = rx[4] & 0b11;
        if (segment_old == segment)
            return;
        segment_old = segment;
        if (segment == 3) {
            printf("%d\r\n%s\r\n", segment, PS);
            memset(PS, ' ', 8);
        }
//        segment = (segment + 3) % 4;

        if (rx[6] < 32 || rx[6] > 126)
            rx[6] = ' ';
        if (rx[7] < 32 || rx[7] > 126)
                    rx[7] = ' ';

        PS[segment * 2] = rx[6];
        PS[segment * 2 + 1] = rx[7];
//        printf("%d\r\n%s\r\n", segment, PS);
    }

//    printf(" A: %2x b0: %d TPC: %d, PTY: %2d, R: %3x ", group_type_code, b0, tp, pty, individual);
//    printf(" %4x %4x %4x %4x\r\n", b1, b2, b3, b4);

//    if (rx[0] <= 0x0F)
//        printf("%d", 0);
//    printf("%X", rx[0]);
//
//    if (rx[1] <= 0x0F)
//        printf("%d", 0);
//    printf("%X ", rx[1]);
//
//    if (rx[2] <= 0x0F)
//        printf("%d", 0);
//    printf("%X", rx[2]);
//    if (rx[3] <= 0x0F)
//        printf("%d", 0);
//    printf("%X ", rx[3]);
//
//    printf("%X", rx[4]);
//    printf("%X", rx[5]);
//    printf("%X", rx[6]);
//    printf("%X\r\n", rx[7]);
}
*/
