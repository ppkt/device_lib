#include "pca9685.h"

void pca9685_init() {
    I2C_LowLevel_Init(I2C1);
}
