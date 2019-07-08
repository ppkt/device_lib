#include "ina219.h"

static error_t ina219_read_register(const ina219_device *device,
                                    ina219_register address, uint16_t *value) {
  if (!device || !value) {
    return E_NULL_PTR;
  }

  uint8_t tx[1] = {address}, rx[2];

  error_t e = i2c_master_transaction_write_read(device->i2c, device->address,
                                                tx, 1, rx, 2);
  if (e != E_SUCCESS) {
    return e;
  }

  *value = (rx[0] << 8u) + rx[1];
  return e;
}

static error_t ina219_write_register(const ina219_device *device,
                                     ina219_register address,
                                     uint16_t new_value) {
  if (!device) {
    return E_NULL_PTR;
  }

  uint8_t tx[3] = {
      address,
      (uint8_t)(new_value >> 8u),
      (uint8_t)(new_value & 0xFFu),
  };

  error_t s = i2c_master_write(device->i2c, device->address, tx, 3);

  return s;
}

error_t ina219_init(ina219_device *device, uint32_t i2c, uint8_t address) {
  if (!device) {
    return E_NULL_PTR;
  }

  (*device).i2c = i2c;
  (*device).address = address;

  ina219_config config;
  error_t e;
  e = ina219_read_register(device, ina219_register_configuration, &config.raw);
  if (e != E_SUCCESS) {
    return e;
  }

  // Check if device requires reset
  if (config.raw != 0x399F) {
    usart1_print("Restarting device\r\n");
    config.rst = 1;
    e = ina219_write_register(device, ina219_register_configuration,
                              config.raw);
    if (e != E_SUCCESS) {
      return e;
    }

    do {
      // Wait until device is ready
      e = ina219_read_register(device, ina219_register_configuration,
                               &config.raw);
      if (e != E_SUCCESS) {
        return e;
      }
    } while (config.raw != 0x399F);
  }

  config = (ina219_config){
      .brng = 0b1,    // Bus Voltage Range - 32V
      .pga = 0b11,    // PGA gain and range - +8 / 320 mA
      .badc = 0b1111, // Bus ADC Settings - 128x12bit samples / 68.1 ms
                      // conversion time
      .sadc = 0b1111, // Shunt ADC Settings - 128x12bit samples / 68.1 ms
                      // conversion time
      .mode = 0b111,  // Mode - Shunt and Bus, Continuous
  };
  usart1_printf("New configuration: %X\r\n", config.raw);

  e = ina219_write_register(device, ina219_register_configuration, config.raw);

  return e;
}

error_t ina219_perform_calibration(ina219_device *device) {
  return ina219_write_register(device, ina219_register_calibration,
                               device->calibration_register);
}

error_t ina219_get_bus_voltage(ina219_device *device) {
  if (!device) {
    return E_NULL_PTR;
  }

  // Read bus voltage
  uint16_t val;
  error_t e = ina219_read_register(device, ina219_register_bus_voltage, &val);
  if (e != E_SUCCESS) {
    return e;
  }

  // TODO: Check and handle overflows
  device->raw_bus_voltage = val >> 3u;

  // 1 LSB = 4mV (value from datasheet)
  device->bus_voltage = device->raw_bus_voltage * 4 / 1000.0f;

  return e;
}

error_t ina219_get_shunt_voltage(ina219_device *device) {
  if (!device) {
    return E_NULL_PTR;
  }

  // Read drop of voltage across shunt
  error_t e = ina219_read_register(device, ina219_register_shunt_voltage,
                                   (uint16_t *)&device->raw_shunt_voltage);
  if (e != E_SUCCESS) {
    return e;
  }

  device->shunt_voltage = device->raw_shunt_voltage / 1000.0f;
  return e;
}

error_t ina219_get_current(ina219_device *device) {
  if (!device) {
    return E_NULL_PTR;
  }

  error_t e = ina219_read_register(device, ina219_register_current,
                                   (uint16_t *)&device->raw_current);
  if (e != E_SUCCESS) {
    return e;
  }

  // 1 LSB = 0.1 mA (calculated value)
  device->current = device->raw_current * 0.1f;
  return e;
}

error_t ina219_get_power(ina219_device *device) {
  if (!device) {
    return E_NULL_PTR;
  }

  error_t e = ina219_read_register(device, ina219_register_power,
                                   (uint16_t *)&device->raw_power);
  if (e != E_SUCCESS) {
    return e;
  }

  // 1 LSB = 2 mW (calculated value) = 20 * Current LSB
  device->power = device->raw_power * 2;
  return e;
}

error_t ina219_get_all_readings(ina219_device *device) {
  error_t e;
  e = ina219_get_bus_voltage(device);
  if (e != E_SUCCESS) {
    return e;
  }

  e = ina219_get_shunt_voltage(device);
  if (e != E_SUCCESS) {
    return e;
  }

  e = ina219_get_current(device);
  if (e != E_SUCCESS) {
    return e;
  }

  e = ina219_get_power(device);
  return e;
}
