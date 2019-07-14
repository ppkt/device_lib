#include "ina219.h"

static error_t ina219_read_register(const ina219_device *device,
                                    ina219_register address, uint16_t *value) {
  if (!device || !value) {
    return E_NULL_PTR;
  }

  uint8_t tx[1] = {address}, rx[2];
  check_error(i2c_master_transaction_write_read(device->i2c, device->address,
                                                tx, 1, rx, 2));

  *value = (rx[0] << 8u) + rx[1];
  return E_SUCCESS;
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
  check_error(i2c_master_write(device->i2c, device->address, tx, 3));

  return E_SUCCESS;
}

error_t ina219_init(ina219_device *device, uint32_t i2c, uint8_t address) {
  if (!device) {
    return E_NULL_PTR;
  }

  (*device).i2c = i2c;
  (*device).address = address;

  check_error(ina219_read_register(device, ina219_register_configuration,
                                   &device->config.raw));

  // Check if device requires reset
  if (device->config.raw != 0x399F) {
    usart1_print("Restarting device\r\n");
    device->config.rst = 1;
    check_error(ina219_write_register(device, ina219_register_configuration,
                                      device->config.raw));

    do {
      // Wait until device is ready
      check_error(ina219_read_register(device, ina219_register_configuration,
                                       &device->config.raw));
    } while (device->config.raw != 0x399F);
  }

  device->config = (ina219_config){
      // Mode - Shunt and Bus, Continuous
      .mode = INA219_MODE_SHUNT_AND_BUS_VOLTAGE_CONTINUOUS,

      // PGA gain and range - +8 / 320 mA
      .pga = INA219_GAIN_2,

      .brng = 0b0,    // Bus Voltage Range - 16V
      .badc = 0b1111, // Bus ADC Settings - 128x12bit samples / 68.1 ms
                      // conversion time
      .sadc = 0b1111, // Shunt ADC Settings - 128x12bit samples / 68.1 ms
                      // conversion time
  };

  return E_SUCCESS;
}

error_t ina219_write_config(ina219_device *device) {
  if (!device) {
    return E_NULL_PTR;
  }

  usart1_printf("New configuration: %X\r\n", device->config.raw);
  check_error(ina219_write_register(device, ina219_register_configuration,
                                    device->config.raw));

  return E_SUCCESS;
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
  check_error(ina219_read_register(device, ina219_register_bus_voltage, &val));

  // TODO: Check and handle overflows
  device->raw_bus_voltage = val >> 3u;

  // 1 LSB = 4mV (value from datasheet)
  device->bus_voltage = device->raw_bus_voltage * 4 / 1000.0f;

  return E_SUCCESS;
}

error_t ina219_get_shunt_voltage(ina219_device *device) {
  if (!device) {
    return E_NULL_PTR;
  }

  // Read drop of voltage across shunt
  check_error(ina219_read_register(device, ina219_register_shunt_voltage,
                                   (uint16_t *)&device->raw_shunt_voltage));

  device->shunt_voltage = device->raw_shunt_voltage / 1000.0f;
  return E_SUCCESS;
}

error_t ina219_get_current(ina219_device *device) {
  if (!device) {
    return E_NULL_PTR;
  }

  check_error(ina219_read_register(device, ina219_register_current,
                                   (uint16_t *)&device->raw_current));

  // 1 LSB = 0.1 mA (calculated value)
  device->current = device->raw_current / device->calibration.current_div;
  return E_SUCCESS;
}

error_t ina219_get_power(ina219_device *device) {
  if (!device) {
    return E_NULL_PTR;
  }

  check_error(ina219_read_register(device, ina219_register_power,
                                   (uint16_t *)&device->raw_power));

  // 1 LSB = 2 mW (calculated value) = 20 * Current LSB
  device->power = device->raw_power * device->calibration.power_multiplier;
  return E_SUCCESS;
}

error_t ina219_get_all_readings(ina219_device *device) {
  check_error(ina219_get_bus_voltage(device));
  check_error(ina219_get_shunt_voltage(device));
  check_error(ina219_get_current(device));
  check_error(ina219_get_power(device));
  return E_SUCCESS;
}

error_t ina219_calibration_32v_2a(ina219_device *device) {
  device->calibration_register = 10240;
  device->calibration.current_div = 25.f;
  device->calibration.power_multiplier = 0.8f;
  check_error(ina219_perform_calibration(device));

  device->config.brng = 1;  // 32V
  device->config.pga = INA219_GAIN_8;
  check_error(ina219_write_config(device));
  return E_SUCCESS;
}
error_t ina219_calibration_16v_400ma(ina219_device *device) {
  device->calibration_register = 8192;
  device->calibration.current_div = 20.f;
  device->calibration.power_multiplier = 1.f;
  check_error(ina219_perform_calibration(device));

  device->config.brng = 0;  // 16V
  device->config.pga = INA219_GAIN_1;
  check_error(ina219_write_config(device));
  return E_SUCCESS;
}
