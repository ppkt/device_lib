#include "ina219.h"

bool ina219_read_register(ina219_device *device,
                          ina219_register address,
                          uint16_t *value);

bool ina219_write_register(ina219_device *device,
                           ina219_register address,
                           uint16_t new_value);

bool ina219_read_register(ina219_device *device,
                          ina219_register address,
                          uint16_t *value) {
    static uint8_t tx[1] = {0,}, rx[2];

    tx[0] = address;
    bool s = i2c_master_transaction_write_read(device->i2c, device->address,
                                                 tx, 1, rx, 2);
    *value = rx[0] << 8 | rx[1];
    return s;
}

bool ina219_write_register(ina219_device *device,
                              ina219_register address,
                              uint16_t new_value)
{
    static uint8_t tx[3] = {0, };

    tx[0] = address;
    tx[1] = (uint8_t) (new_value >> 8);
    tx[2] = (uint8_t) (new_value & 0xFF);
    bool s = i2c_master_write(device->i2c, device->address, tx, 3);

    return s;
}

bool ina219_setup(ina219_device **dev,
                  uint32_t i2c,
                  uint8_t address) {
    *dev = (ina219_device *) malloc(sizeof(ina219_device));
    (*dev)->i2c = i2c;
    (*dev)->address = address;

    return false;
}


bool ina219_init(ina219_device *device)
{
    ina219_config config;
    ina219_read_register(device, ina219_register_configuration, &config.raw);

    // Check if device requires reset
    if (config.raw != 0x399F) {
        usart1_print("Restarting device\r\n");
        config.rst = 1;
        ina219_write_register(device, ina219_register_configuration, config.raw);

        do {
            // Wait until device is ready
            ina219_read_register(device, ina219_register_configuration, &config.raw);
        } while (config.raw != 0x399F);
    }

    config = (ina219_config) {
        .brng = 0b1, // Bus Voltage Range - 32V
        .pga = 0b11, // PGA gain and range - +8 / 320 mA
        .badc = 0b1111, // Bus ADC Settings - 128x12bit samples / 68.1 ms conversion time
        .sadc = 0b1111, // Shunt ADC Settings - 128x12bit samples / 68.1 ms conversion time
        .mode = 0b111, // Mode - Shunt and Bus, Continuous
    };
    usart1_printf("New configuration: %X\r\n", config.raw);

    ina219_write_register(device, ina219_register_configuration, config.raw);

    return 0;
}

bool ina219_perform_calibration(ina219_device *device)
{
    return ina219_write_register(device, ina219_register_calibration,
                                 device->calibration_register);
}

void ina219_get_bus_voltage(ina219_device *device) {
    // Read bus voltage
    uint16_t val;
    ina219_read_register(device, ina219_register_bus_voltage, &val);

    // TODO: Check and handle overflows
    device->raw_bus_voltage = val >> 3;


    // 1 LSB = 4mV (value from datasheet)
    device->bus_voltage = device->raw_bus_voltage * 4 / 1000.0f;
}

void ina219_get_shunt_voltage(ina219_device *device) {
    // Read drop of voltage across shunt
    ina219_read_register(device, ina219_register_shunt_voltage,
                         (uint16_t *) &device->raw_shunt_voltage);

    device->shunt_voltage = device->raw_shunt_voltage / 1000.0f;
}

void ina219_get_current(ina219_device *device) {
    ina219_read_register(device, ina219_register_current,
                         (uint16_t *) &device->raw_current);

    // 1 LSB = 0.1 mA (calculated value)
    device->current = device->raw_current * 0.1f;
}

void ina219_get_power(ina219_device *device) {
    ina219_read_register(device, ina219_register_power,
                         (uint16_t *) &device->raw_power);

    // 1 LSB = 2 mW (calculated value) = 20 * Current LSB
    device->power = device->raw_power * 2;
}

void ina219_get_all_readings(ina219_device *device) {
    ina219_get_bus_voltage(device);
    ina219_get_shunt_voltage(device);
    ina219_get_current(device);
    ina219_get_power(device);
}

