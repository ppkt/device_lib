#include "rda5807_tea.h"

static u8 tx[] = {0x20, 0xD0, 0x00, 0x00, 0x00, 0x00, 0x40, 0x90, 0x88};
//u8 rx[64];

void rda5807_tea_set_frequency(unsigned int new_frequency) {
    unsigned int frequencyB=4*(new_frequency*100000+225000)/32768; //calculating PLL word

    u8 frequencyH=frequencyB>>8;

    u8 frequencyL=frequencyB&0XFF;

    tx[0] = frequencyH;
    tx[1] = frequencyL;
    tx[2] = 0xB0;
    tx[3] = 0x10;
    tx[4] = 0x00;
    I2C_Master_BufferWrite(I2C1, tx, 5, Polling, RDA5970_TEA_ADDRESS << 1);
}

void rda5807_tea_init() {
}

//bool rda5807_check_presence() {
//    //	I2C_Master_BufferWrite(I2C1, tx, 1, DMA, RDA5970_ADDRESS << 1);
//    //	I2C_Master_BufferRead(I2C1, rx, 64, DMA, RDA5970_ADDRESS << 1);
//
//    if (rx[6] == RDA5970_ID) {
//        printf("R\r\n");
//        return true;
//    }
//    return false;
//}

bool rda5807_tea_mute() {
    tx[0] = tx[0] ^ 1 << 7;

    I2C_Master_BufferWrite(I2C1, tx, 5, Polling, RDA5970_TEA_ADDRESS << 1);

    return tx[0] & 1 << 7;
}

