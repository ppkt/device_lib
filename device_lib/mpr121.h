#pragma once

#define MPR121_DEFAULT_ADDRESS 0x5A

// Register addresses
#define MPR121_MHD_R 0x2B
#define MPR121_NHD_R 0x2C
#define MPR121_NCL_R 0x2D
#define MPR121_FDL_R 0x2E
#define MPR121_MHD_F 0x2F
#define MPR121_NHD_F 0x30
#define MPR121_NCL_F 0x31
#define MPR121_FDL_F 0x32
#define MPR121_ELE0_T 0x41
#define MPR121_ELE0_R 0x42
#define MPR121_ELE1_T 0x43
#define MPR121_ELE1_R 0x44
#define MPR121_ELE2_T 0x45
#define MPR121_ELE2_R 0x46
#define MPR121_ELE3_T 0x47
#define MPR121_ELE3_R 0x48
#define MPR121_ELE4_T 0x49
#define MPR121_ELE4_R 0x4A
#define MPR121_ELE5_T 0x4B
#define MPR121_ELE5_R 0x4C
#define MPR121_ELE6_T 0x4D
#define MPR121_ELE6_R 0x4E
#define MPR121_ELE7_T 0x4F
#define MPR121_ELE7_R 0x50
#define MPR121_ELE8_T 0x51
#define MPR121_ELE8_R 0x52
#define MPR121_ELE9_T 0x53
#define MPR121_ELE9_R 0x54
#define MPR121_ELE10_T 0x55
#define MPR121_ELE10_R 0x56
#define MPR121_ELE11_T 0x57
#define MPR121_ELE11_R 0x58

#define MPR121_AFE_CONF1_REG 0x5C
#define MPR121_AFE_CONF2_REG 0x5D
#define MPR121_ERC_REG 0x5E

#define MPR121_AUTO_CONFIG0 0x7B

#define MPR121_SRST_REG 0x80

struct mpr121_device {
    I2C_TypeDef* i2c;
    TIM_TypeDef* timer;
    uint8_t address;
};
typedef struct mpr121_device mpr121_device_t;


uint8_t mpr121_setup(mpr121_device_t **dev,
                     I2C_TypeDef* i2c,
                     TIM_TypeDef* timer,
                     uint8_t address);
uint8_t mpr121_init(mpr121_device_t* device,
                    uint8_t touch_threshold,
                    uint8_t release_threshold);
uint16_t mpr121_read_buttons(mpr121_device_t* device);
