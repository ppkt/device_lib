#include "ds18b20.h"

static uint8_t precision = 3;
static TIM_TypeDef *timer;

// Initialize library and return 1-Wire devices
ds18b20_devices ds18b20_init(GPIO_TypeDef *gpio, uint16_t port, TIM_TypeDef *t) {
    timer = t;
    one_wire_init(gpio, port, t);

    ds18b20_devices ret;
    ret.devices = one_wire_search_rom(&ret.size);
    return ret;
}

// Return precision
uint8_t ds18b20_get_precision(void) {
    return precision;
}

// Set new precision (0-3, 0-lowest, 3-highest)
void ds18b20_set_precision(uint8_t p) {
    precision = p;
    one_wire_reset_pulse();

    one_wire_write_byte(0xCC); // Skip ROM
    one_wire_write_byte(0x4E); // Write scratchpad

    one_wire_write_byte(0x4B);
    one_wire_write_byte(0x46);
    // set precision
    one_wire_write_byte((uint8_t) 0x1F | (precision << 5));
}

// Send command to device to convert temperature
// (use only if there is one device connected)
void ds18b20_convert_temperature_simple(void) {
    one_wire_reset_pulse();
    one_wire_write_byte(0xCC); // Skip ROM
    one_wire_write_byte(0x44); // Convert temperature
}

// Send command to device to convert temperature
void ds18b20_convert_temperature(one_wire_device device) {
    one_wire_reset_pulse();
    one_wire_match_rom(device); // Match ROM
    one_wire_write_byte(0x44); // Convert temperature
}

// Send command to convert temperature to every device in list
void ds18b20_convert_temperature_all(ds18b20_devices devices) {
    uint8_t i = 0;
    for (i = 0; i < devices.size; ++i) {
        ds18b20_convert_temperature(devices.devices[i]);
    }
}

// Read converted temperature from device
// (use only if there is one device connected)
float ds18b20_read_temperature_simple(void) {
    one_wire_reset_pulse();
    one_wire_write_byte(0xCC); // Skip ROM
    one_wire_write_byte(0xBE); // Read scratchpad

    return ds18b20_decode_temperature();
}

// Read converted temperature from device
float ds18b20_read_temperature(one_wire_device device) {
    one_wire_reset_pulse();
    one_wire_match_rom(device); // Match ROM
    one_wire_write_byte(0xBE); // Read scratchpad

    return ds18b20_decode_temperature();
}

// Read converted temperature from every device in list
float *ds18b20_read_temperature_all(ds18b20_devices devices) {
    uint8_t i = 0;
    float *temperatures;
    temperatures = malloc(devices.size * sizeof(float));

    for (i = 0; i < devices.size; ++i) {
        temperatures[i] = ds18b20_read_temperature(devices.devices[i]);
    }
    return temperatures;
}

// Decode temperature send from device. Use CRC to ensure that reading is
// correct. In case of CRC error a value -255 is returned.
float ds18b20_decode_temperature() {
    uint8_t crc = 0;
    uint8_t data[9];
    one_wire_reset_crc();

    for (uint8_t i = 0; i < 9; ++i) {
        data[i] = one_wire_read_byte();
        crc = one_wire_crc(data[i]);
    }

    uint8_t temp_msb = data[1]; // Sign byte + lsbit
    uint8_t temp_lsb = data[0]; // Temp data plus lsb

    if (crc != 0 || (!crc && !temp_msb && !temp_lsb)) {
        return -255.0f;
    }

//    char buffer[10];
//    sprintf(buffer, "%d.%d\r", (int)temp, rest);
//    usart2_print(buffer);

    return (int16_t) (temp_msb << 8 | temp_lsb) / 16.0f;
}

// Wait for temperature conversion depending on precision set
void ds18b20_wait_for_conversion(void) {
    if (precision == 0) {
        delay_ms(timer, 95);
    } else if (precision == 1) {
        delay_ms(timer, 190);
    } else if (precision == 2) {
        delay_ms(timer, 380);
    } else if (precision == 3) {
        delay_ms(timer, 750);
    }
}

// Read temperature from device. This function performs all operations in once:
// 1. Send command to device to convert temperature
// 2. Wait until conversion is done
// 3. Read and decode temperature
// (use only if there is one device connected)
float ds18b20_get_temperature_simple(void) {
    ds18b20_convert_temperature_simple();  // we have only one device
    ds18b20_wait_for_conversion();
    return ds18b20_read_temperature_simple();
}

// Read temperature from all devices in list.
// This function performs all operations in once:
// 1. Send command to devices to convert temperature
// 2. Wait until conversion is done
// 3. Read and decode temperature from every device
float *ds18b20_get_temperature_all(ds18b20_devices devices) {
    ds18b20_convert_temperature_all(devices);
    ds18b20_wait_for_conversion();
    return ds18b20_read_temperature_all(devices);
}
