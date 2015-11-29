#include "bmp180.h"

/* Buffer of data to be received by I2C */
static uint8_t rx[22];
/* Buffer of data to be transmitted by I2C */
static uint8_t tx[2] = {0xAA};

// delay timer
static TIM_TypeDef *timer;
// i2c peripheral in use
I2C_TypeDef *i2c_bus;

// Get delay (in ms), depending on sampling mode
u8 bmp180_get_delay(BMP180_Mode mode) {
    switch (mode) {
        case BMP180_MODE_ULTRA_LOW_POWER:
            return 5;
        case BMP180_MODE_STANDARD:
            return 8;
        case BMP180_MODE_HIGH_RESOLUTION:
            return 14;
        case BMP180_MODE_ULTRA_HIGH_RESOLUTION:
            return 30;
        default:
            while(1) {}
    }
}

void bmp180_init(I2C_TypeDef *i2c, TIM_TypeDef *t) {
    i2c_bus = i2c;
    I2C_LowLevel_Init(i2c_bus);
    timer = t;

}

// 0. Check presence of BMP180 in I2C bus
bool bmp180_check_presence() {
    tx[0] = 0xD0; // Address of device
    I2C_Master_BufferWrite(i2c_bus, tx, 1, Polling, BMP180_ADDRESS << 1);
    I2C_Master_BufferRead(i2c_bus, rx, 1, Polling, BMP180_ADDRESS << 1);

    if (rx[0] == BMP180_CHIP_ID) {
        return true;
    } else {
        return false;
    }
}

// 1. Fetch calibration data
void bmp180_get_calibration_data(CalibrationData *c) {
    tx[0] = 0xAA; // Begin of calibration data, 22 bytes length
    I2C_Master_BufferWrite(i2c_bus, tx, 1, Polling, BMP180_ADDRESS << 1);
    I2C_Master_BufferRead(i2c_bus, rx, 22, Polling, BMP180_ADDRESS << 1);

    c->AC1 = rx[0] << 8 | rx[1];
    c->AC2 = rx[2] << 8 | rx[3];
    c->AC3 = rx[4] << 8 | rx[5];
    c->AC4 = rx[6] << 8 | rx[7];
    c->AC5 = rx[8] << 8 | rx[9];
    c->AC6 = rx[10] << 8 | rx[11];
    c->B1  = rx[12] << 8 | rx[13];
    c->B2  = rx[14] << 8 | rx[15];
    c->MB  = rx[16] << 8 | rx[17];
    c->MC  = rx[18] << 8 | rx[19];
    c->MD  = rx[20] << 8 | rx[21];
}

void bmp180_sanity_check1(CalibrationData *c) {
    // Bosch datasheet
    c->AC1 = 408;
    c->AC2 = -72;
    c->AC3 = -14383;
    c->AC4 = 32741;
    c->AC5 = 32757;
    c->AC6 = 23153;
    c->B1 = 6190;
    c->B2 = 4;
    c->MB = -32768;
    c->MC = -8711;
    c->MD = 2868;

    c->UT = 27898;

    c->UP = 23843;

    c->oss = 0;

    bmp180_calculate_true_temperature(c);
    bmp180_calculate_true_pressure(c);
    // Expected values: T=15.0 C, P=699.54 hPa
}

void bmp180_sanity_check2(CalibrationData *c) {
    // http://wmrx00.sourceforge.net/Arduino/BMP085-Calcs.pdf
    c->AC1 = 7911;
    c->AC2 = -934;
    c->AC3 = -14306;
    c->AC4 = 31567;
    c->AC5 = 25671;
    c->AC6 = 18974;
    c->B1 = 5498;
    c->B2 = 46;
    c->MB = -32768;
    c->MC = -11075;
    c->MD = 2432;

    c->UT = 27116;

    c->UP = 38959;

    c->oss = 0;

    bmp180_calculate_true_temperature(c);
    bmp180_calculate_true_pressure(c);
    // Expected values: T=23.7764 C, P=980.0456 hPa
}

// 2. Get raw value of temperature
void bmp180_get_uncompensated_temperature(CalibrationData* data) {
    tx[0] = 0xF4; // Register to write
    tx[1] = 0x2E; // Value to write (measure temperature)
    I2C_Master_BufferWrite(i2c_bus, tx, 2, Polling, BMP180_ADDRESS << 1);

    delay_ms(timer, 5);

    // Read two bytes
    tx[0] = 0xF6; // Register to read (temperature)
    I2C_Master_BufferWrite(i2c_bus, tx, 1, Polling, BMP180_ADDRESS << 1);
    I2C_Master_BufferRead(i2c_bus, rx, 2, Polling, BMP180_ADDRESS << 1);
    data->UT = rx[0] << 8 | rx[1];
}

// 3. Get raw value of pressure
void bmp180_get_uncompensated_pressure(CalibrationData* data) {
    tx[0] = 0xF4; // Register to write
    tx[1] = 0x34 | (data->oss << 6); // Value to write (measure pressure)
    I2C_Master_BufferWrite(i2c_bus, tx, 2, Polling, BMP180_ADDRESS << 1);

    // Wait for reading
    delay_ms(timer, bmp180_get_delay(data->oss));

    // Read two bytes
    tx[0] = 0xF6; // Register to read (pressure)
    I2C_Master_BufferWrite(i2c_bus, tx, 1, Polling, BMP180_ADDRESS << 1);
    I2C_Master_BufferRead(i2c_bus, rx, 3, Polling, BMP180_ADDRESS << 1);
    data->UP = (rx[0] << 16 | rx[1] << 8 | rx[2]) >> (8 - data->oss);
}

// 4. Using calibration data, get real temperature
void bmp180_calculate_true_temperature(CalibrationData* data) {
    data->X1 = ((data->UT - data->AC6) * data->AC5) >> 15;
    data->X2 = ((data-> MC) << 11) / (data->X1 + data->MD);
    data->B5 = data->X1 + data->X2;
    data->T = (data->B5 + 8) >> 4;

//    printf("T = %u\n\r", (unsigned int)data->T);
}

// 5. Using calibration data and real temperature, get real pressure
void bmp180_calculate_true_pressure(CalibrationData *data) {
    data->B6 = data->B5 - 4000;

    data->X1 = (data->B2 * (data->B6 * data->B6) >> 12) >> 11;
    data->X2 = (data->AC2 * data->B6) >> 11;
    data->X3 = data->X1 + data->X2;
    data->B3 = ((((long)data->AC1 * 4 + data->X3) << data->oss) + 2) >> 2;

    data->X1 = (data->AC3 * data->B6) >> 13;
    data->X2 = (data->B6 * data->B6) >> 12;
    data->X2 = (data->X2 * data->B1) >> 16;
    data->X3 = ((data->X1 + data->X2) + 2) >> 2;
    data->B4 = data->AC4 * (unsigned long)(data->X3 + 32768) >> 15;

    data->B7 = ((unsigned long)data->UP - data->B3) * (50000 >> data->oss);
    if (data->B7 < 0x80000000) {
        data->p = (data->B7 << 1) / data->B4;
    } else {
        data->p = (data->B7 / data->B4) << 1;
    }
    data->X1 = data->p >> 8;
    data->X1 *= data->X1;
    data->X1 = (data->X1 * 3038) >> 16;
    data->X2 = (data->p * -7357) >> 16;
    data->p = data->p + ((data->X1 + data->X2 + 3791) >> 4);

//    printf("p = %u\n\r", (unsigned int)data->p);
}

// 6. Convert real pressure to absolute altitude
void bmp180_get_absolute_altitude(CalibrationData *data) {
    // x^y -> exp(y * log(x))
    float b = expf(1.0/5.255 * logf(data->p / 101325.0));
    float f = 44330 * (1 - b);
//    printf("altitude = %d\n\r", (int)f);

    //	version above doesn't require more memory
    //	float F = 44330 * (1 - powf(data->p / 101325.0, 1.0/5.255));
    //	printf("altitude = %d\n\r", (int)F);
}
