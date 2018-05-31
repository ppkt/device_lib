#include <stdlib.h>

#include "common_lib/i2c_dma.h"
#include "common_lib/utils.h"

#include "mpr121.h"

uint8_t mpr121_setup(mpr121_device_t **dev,
                     I2C_TypeDef* i2c,
                     TIM_TypeDef* timer,
                     uint8_t address)
{
    *dev = (mpr121_device_t*) malloc(sizeof(mpr121_device_t));
    (*dev)->i2c = i2c;
    (*dev)->timer = timer;
    (*dev)->address = address;

    return 0;
}

uint8_t mpr121_init(mpr121_device_t *dev,
                    uint8_t touch_threshold,
                    uint8_t release_threshold)
{

    static uint8_t tx[10] = {0, }, rx[10] = {0,};
    Status s = 0;
    UNUSED(s);

    // Init I2C bus
    I2C_LowLevel_Init(dev->i2c);

    // Perform reset
    s = i2c_master_buffer_write_byte(dev->i2c, dev->address,
            MPR121_SRST_REG, 0x63);
    // If device is not responding / present - following assert will fail
    assert_param(s == Success);
    if (s != Success)
        return -1;

    // Check presence
    tx[0] = MPR121_AFE_CONF1_REG;
    while (rx[0] != 0x10 || rx[1] != 0x24) {
        // Wait for reset
        i2c_master_transaction_write_read(dev->i2c, dev->address, tx, 1, rx, 2,
                                          DMA);

        delay_ms(dev->timer, 100);
    }

    // Section A - Controls filtering when data is > baseline.
    i2c_master_buffer_write_byte(dev->i2c, dev->address, MPR121_MHD_R, 0x01);
    i2c_master_buffer_write_byte(dev->i2c, dev->address, MPR121_NHD_R, 0x01);
    i2c_master_buffer_write_byte(dev->i2c, dev->address, MPR121_NCL_R, 0x00);
    i2c_master_buffer_write_byte(dev->i2c, dev->address, MPR121_FDL_R, 0x00);

    // Section B - Controls filtering when data is < baseline.
    i2c_master_buffer_write_byte(dev->i2c, dev->address, MPR121_MHD_F, 0x01);
    i2c_master_buffer_write_byte(dev->i2c, dev->address, MPR121_NHD_F, 0x01);
    i2c_master_buffer_write_byte(dev->i2c, dev->address, MPR121_NCL_F, 0xFF);
    i2c_master_buffer_write_byte(dev->i2c, dev->address, MPR121_FDL_F, 0x02);

    // Section C - Sets touch and release thresholds for each electrode
    for (uint8_t i = 0; i < 12; ++i) {
        i2c_master_buffer_write_byte(dev->i2c, dev->address,
                                     MPR121_ELE0_T + i * 2, touch_threshold);
        i2c_master_buffer_write_byte(dev->i2c, dev->address,
                                     MPR121_ELE0_R + i * 2, release_threshold);
    }

    // Section D
    // Set the Filter Configuration
    // Set ESI2
    i2c_master_buffer_write_byte(dev->i2c, dev->address,
                                 MPR121_AFE_CONF2_REG, 0x04);

    // Section E
    // Electrode Configuration
    // Set ELE_CFG to 0x00 to return to standby mode
    i2c_master_buffer_write_byte(dev->i2c, dev->address,
                                 MPR121_ERC_REG, 0x0C);

    return 0;
}

uint16_t mpr121_read_buttons(mpr121_device_t* device)
{
    static uint8_t tx[1], rx[2];
    tx[0] = 0x00;
    i2c_master_transaction_write_read(device->i2c, dev->address, tx, 1, rx, 2,
                                      DMA);

    return rx[1] << 8 | rx[0];
}
