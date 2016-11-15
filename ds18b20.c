#include "ds18b20.h"

static uint8_t precission = 3;
static one_wire_device *one_wire_list_of_devices;
static uint8_t one_wire_devices_list_size = 10;
static TIM_TypeDef *timer;

void ds18b20_init(GPIO_TypeDef *gpio, uint16_t port, TIM_TypeDef *t) {
    timer = t;
    one_wire_init(gpio, port, t);

    ds18b20_get_devices(true);
}

uint8_t ds18b20_get_precission(void) {
    return precission;
}

ds18b20_devices ds18b20_get_devices(bool scan) {
    if (scan) {
        one_wire_devices_list_size = 0;
        one_wire_list_of_devices = one_wire_search_rom(&one_wire_devices_list_size);
    }
    ds18b20_devices ret;
    ret.size = one_wire_devices_list_size;
    ret.devices = one_wire_list_of_devices;
    return ret;
}

void ds18b20_set_precission(uint8_t p) {
    precission = p;
    one_wire_reset_pulse();

    one_wire_write_byte(0xCC); // Skip ROM
    one_wire_write_byte(0x4E); // Write scratchpad

    one_wire_write_byte(0x4B);
    one_wire_write_byte(0x46);
    // set precission
    one_wire_write_byte(0x1F | (precission << 5));
}

void ds18b20_convert_temperature_simple(void) {
    one_wire_reset_pulse();
    one_wire_write_byte(0xCC); // Skip ROM
    one_wire_write_byte(0x44); // Convert temperature
}

void ds18b20_convert_temperature(one_wire_device device) {
    one_wire_reset_pulse();
    one_wire_match_rom(device); // Match ROM
    one_wire_write_byte(0x44); // Convert temperature
}

void ds18b20_convert_temperature_all(void) {
    uint8_t i = 0;
    for (i = 0; i < one_wire_devices_list_size; ++i) {
        ds18b20_convert_temperature(one_wire_list_of_devices[i]);
    }
}

simple_float ds18b20_read_temperature_simple(void) {
    one_wire_reset_pulse();
    one_wire_write_byte(0xCC); // Skip ROM
    one_wire_write_byte(0xBE); // Read scratchpad

    return ds18b20_decode_temperature();
}

simple_float ds18b20_read_temperature(one_wire_device device) {
    one_wire_reset_pulse();
    one_wire_match_rom(device); // Match ROM
    one_wire_write_byte(0xBE); // Read scratchpad

    return ds18b20_decode_temperature();
}

simple_float* ds18b20_read_temperature_all(void) {
    uint8_t i = 0;
    simple_float *temperatures;
    temperatures = malloc(one_wire_devices_list_size * sizeof(simple_float));

    for (i = 0; i < one_wire_devices_list_size; ++i) {
        temperatures[i] = ds18b20_read_temperature(one_wire_list_of_devices[i]);
    }
    return temperatures;
}

simple_float ds18b20_decode_temperature(void) {
    int i;
    uint8_t crc;
    uint8_t data[9];
    simple_float f;
    f.is_valid = false;
    one_wire_reset_crc();

    for (i = 0; i < 9; ++i) {
        data[i] = one_wire_read_byte();
        crc = one_wire_crc(data[i]);
    }

    uint8_t temp_msb = data[1]; // Sign byte + lsbit
    uint8_t temp_lsb = data[0]; // Temp data plus lsb

    if (crc != 0 || (!crc && !temp_msb && !temp_lsb)) {
        return f;
    }

    float temp = (int16_t)(temp_msb << 8 | temp_lsb) / 16.0;
    int rest = (temp - (int)temp) * 1000.0;

    f.integer = (uint8_t)temp;
    f.fractional = rest;
    f.is_valid = true;

//    char buffer[10];
//    sprintf(buffer, "%d.%d\r", (int)temp, rest);
//    usart2_print(buffer);
    return f;
}

void ds18b20_wait_for_conversion(void) {
    if (precission == 0) {
        delay_ms(timer, 95);
    } else if (precission == 1) {
        delay_ms(timer, 190);
    } else if (precission == 2) {
        delay_ms(timer, 380);
    } else if (precission == 3) {
        delay_ms(timer, 750);
    }
}


simple_float ds18b20_get_temperature_simple(void) {
    ds18b20_convert_temperature_simple();  // we have only one device
    ds18b20_wait_for_conversion();
    return ds18b20_read_temperature_simple();
}
