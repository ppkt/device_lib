#include "ina219.h"

static uint8_t rx[3];

void ina219_initial_configuration(void) {
    // Check if device requires reset
    rx[0] = 0x00;
    I2C_Master_BufferWrite(I2C1, rx, 1, Polling, 0x40 << 1);
    I2C_Master_BufferRead(I2C1, rx, 2, Polling, 0x40 << 1);

    uint16_t configuration = rx[0] << 8 | rx[1];
    if (configuration != 0x399F) {
        printf("Restarting device\r\n");
        rx[0] = 0x00;
        rx[1] = 0x80;
        rx[2] = 0x00;
        I2C_Master_BufferWrite(I2C1, rx, 3, Polling, 0x40 << 1);

        do {
            // Wait until device is ready
            rx[0] = 0x00;
            I2C_Master_BufferWrite(I2C1, rx, 1, Polling, 0x40 << 1);
            I2C_Master_BufferRead(I2C1, rx, 2, Polling, 0x40 << 1);
            configuration = rx[0] << 8 | rx[1];
        } while (configuration != 0x399F);
    }

    bool brng = true; // Bus Voltage Range - 32V
    u8 pg = 0b11; // PGA gain and range - +8 / 320 mA
    // Default settings:
//    u8 badc = 0b0011; // Bus ADC Settings - 12bit samples / 532 us conversion time
//    u8 sadc = 0b0011; // Shunt ADC Settings - 12bit samples / 532 us conversion time

    u8 badc = 0b1111; // Bus ADC Settings - 128x12bit samples / 68.1 ms conversion time
    u8 sadc = 0b1111; // Shunt ADC Settings - 128x12bit samples / 68.1 ms conversion time

    u8 mode = 0b111; // Mode - Shunt and Bus, Continous

    configuration = brng << 13 | pg << 11 | badc << 7 | sadc << 3 | mode;
    printf("New configuration: %X\r\n", configuration);
    rx[0] = 0x00;
    rx[1] = configuration >> 8;
    rx[2] = 0x00FF & configuration;
    I2C_Master_BufferWrite(I2C1, rx, 3, Polling, 0x40 << 1);

}

void ina219_setup(void) {
    I2C_LowLevel_Init(I2C1);

    ina219_initial_configuration();
}

uint16_t ina219_perform_calibration(void) {
    // Write new values (calculated from INA219 datasheet) of calibration
    // register
    //
    // WARNING: All calculations (except measuring voltage on bus and shunt
    // resistor - 0.1 ohm in this case) are based on "PROGRAMMING THE INA219
    // POWER MEASUREMENT ENGINE" chapter of INA219 datasheet
    rx[0] = 0x05;
    rx[1] = 0x40;
    rx[2] = 0x00;

    I2C_Master_BufferWrite(I2C1, rx, 3, Polling, 0x40 << 1);

    return (rx[1] << 8) | rx[2];
}

uint16_t ina219_read_bus_voltage(void) {
    // Read and print bus voltage

    rx[0] = 0x02;
    I2C_Master_BufferWrite(I2C1, rx, 1, Polling, 0x40 << 1);
    I2C_Master_BufferRead(I2C1, rx, 2, Polling, 0x40 << 1);

    // TODO: Check and handle overflows
    uint16_t value = (rx[0] << 8 | rx[1]) >> 3;

    return value;
}

void ina219_print_bus_voltage(void) {
    int16_t voltage = ina219_read_bus_voltage();

    voltage *= 4; // 1 LSB = 4mV (value from datasheet)

    uint8_t v = voltage / 1000;
    printf("Bus voltage: %d.%03d V\r\n", v, abs(voltage - (v * 1000)));

}

int16_t ina219_read_shunt_voltage(void) {
    // Read drop of voltage across shunt

    rx[0] = 0x01;
    I2C_Master_BufferWrite(I2C1, rx, 1, Polling, 0x40 << 1);
    I2C_Master_BufferRead(I2C1, rx, 2, Polling, 0x40 << 1);

    int16_t value = (rx[0] << 8 | rx[1]);

    return value;
}

void ina219_print_shunt_voltage(void) {
    int16_t voltage = ina219_read_shunt_voltage();

    int8_t v = voltage / 1000; // 1 LSB = 10uV (value from datasheet)

    printf("Shunt voltage: %d.%03d mV\r\n", v, abs(voltage - (v * 1000)));
}

uint16_t ina219_read_current(void) {
    // Based on shunt voltage drop and value of calibration register
    // calculates current

    // Read calibration register
    rx[0] = 0x05;
    I2C_Master_BufferWrite(I2C1, rx, 1, Polling, 0x40 << 1);
    I2C_Master_BufferRead(I2C1, rx, 2, Polling, 0x40 << 1);

    uint16_t calibration_register = (rx[0] << 8 | rx[1]);

    int16_t shunt_voltage = ina219_read_shunt_voltage();

    uint32_t current = (shunt_voltage * calibration_register) / 4096;

    return current;
}

void ina219_print_current(void) {
    uint16_t current = ina219_read_current();

    float m_ampers = current * 0.025; // 1 LSB = 0.025 mA (calculated value)

    printf("Current: %d mA\r\n", (uint8_t)m_ampers);
}

uint16_t ina219_get_power(void) {
    // Read power register (current * drop voltage / 5000)

    rx[0] = 0x03;
    I2C_Master_BufferWrite(I2C1, rx, 1, Polling, 0x40 << 1);
    I2C_Master_BufferRead(I2C1, rx, 2, Polling, 0x40 << 1);
    uint16_t power = (rx[0] << 8 | rx[1]);

    return power;
}

void ina219_print_power(void) {
    uint16_t power = ina219_get_power();

    power /= 2; // 1 LSB = 0.5 mW (calculated value) = 20 * Current LSB

    printf("Power: %u mW\r\n", power);
}
