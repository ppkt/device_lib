#include "pcd8544.h"

TIM_TypeDef *pcd8544_timer;

u8 pcd8544_tx[1];
u8 pcd8544_rx[1];

u8 pcd8544_x_max = 84;
u8 pcd8544_y_max = 48;

bool pcd8544_disable_update = false;

void *pcd8544_memory;

void pcd8544_rcc_configuration() {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
}

void pcd8544_gpio_configuration() {
    GPIO_InitTypeDef GPIO_InitStructure;

    // Configure PCD8544 pins
    // RST -> PA3
    // D/C -> PA2
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_WriteBit(GPIOA, GPIO_Pin_3, Bit_RESET);
}

void pcd8544_RES() {
    // Reset pulse
    GPIO_WriteBit(GPIOA, GPIO_Pin_3, Bit_RESET);

    // Wait at least 100 ns
    delay_us(pcd8544_timer, 5);

    GPIO_WriteBit(GPIOA, GPIO_Pin_3, Bit_SET);

    delay_ms(pcd8544_timer, 1);
}

void pcd8544_send(bool DC, u8 value) {
    // DC - true - sending data, false - sending commands
    if (DC) {
        GPIO_WriteBit(GPIOA, GPIO_Pin_2, Bit_SET);
    } else {
        GPIO_WriteBit(GPIOA, GPIO_Pin_2, Bit_RESET);
    }

    pcd8544_tx[0] = value;
    spi_send(SPI1, pcd8544_tx, pcd8544_rx, 1);

}

void pcd8544_update() {
    if (pcd8544_disable_update) {
        return;
    }

    u16 idx;
    spi_nss(SPI1, Bit_RESET);
    pcd8544_send(0, 0x80);
    pcd8544_send(0, 0x40);

    u8 *ptr = pcd8544_memory;
    for (idx = 0; idx < (48*84) / 8; ++idx, ++ptr) {
        pcd8544_send(1, *ptr);
    }

    spi_nss(SPI1, Bit_SET);
}

void pcd8544_clear(bool value) {
    u16 idx;
    for (idx = 0; idx < (48*84)/8; ++idx) {
        ((u8*)pcd8544_memory)[idx] = 0x00;
    }

    pcd8544_update();
}

void pcd8544_set_pixel(uint8_t x, uint8_t y) {
    static uint8_t bank;
    static uint8_t index;

    bank = y / 8;
    index = y - (bank * 8);

    ((uint8_t*)pcd8544_memory)[x * (pcd8544_y_max/8) + bank] |= 1 << index;
}

void pcd8544_draw_pixel(uint8_t x, uint8_t y) {
    // draws a single pixel in memory and updates a screen
    // warning - this way of addressing works only if Vertical Addressing
    //           register is set to TRUE
    if (x >= pcd8544_x_max) {
        x = pcd8544_x_max - 1; // should return?
    }
    if (y >= pcd8544_y_max) {
        y = pcd8544_y_max - 1;
    }

    pcd8544_set_pixel(x, y);

    pcd8544_update();
}

void pcd8544_draw_line(uint8_t x1, uint8_t y1,
                       uint8_t x2, uint8_t y2) {
    // Line drawing with aliasing, from:
    // http://mst.mimuw.edu.pl/lecture.php?lecture=gk1&part=Ch2

    int8_t delta_x, delta_y, g, h, c;
    delta_x = x2 - x1;
    if (delta_x > 0)
        g = 1;
    else
        g = -1;

    delta_y = y2 - y1;
    if (delta_y > 0)
        h = 1;
    else
        h = -1;

    delta_x = abs(delta_x);
    delta_y = abs(delta_y);

    if (delta_x > delta_y) {
        c = -delta_x;
        while (x1 != x2) {
            pcd8544_draw_pixel(x1, y1);
            c += 2 * delta_y;
            if (c > 0) {
                y1 += h;
                c -= 2 * delta_x;
            }
            x1 += g;
        }
    } else {
        c = -delta_y;
        while (y1 != y2) {
            pcd8544_draw_pixel(x1, y1);
            c += 2 * delta_x;
            if (c > 0) {
                x1 += g;
                c -= 2 * delta_y;
            }
            y1 += h;
        }
    }
}

inline void pcd8544_set_autoupdate(bool value) {
    pcd8544_disable_update = !value;
}

void pcd8544_force_update(void) {
    bool old_value = pcd8544_disable_update;
    pcd8544_disable_update = false;
    pcd8544_update();
    pcd8544_disable_update = old_value;
}

void pcd8544_init(TIM_TypeDef *timer) {
    pcd8544_timer = timer;

    pcd8544_memory = malloc((48*84) / 8);

    pcd8544_rcc_configuration();
    pcd8544_gpio_configuration();

    spi_init(SPI1);

    pcd8544_RES();

    spi_nss(SPI1, Bit_RESET);
    // Initial configuration
//    pcd8544_send(0, 0x21);
//    pcd8544_send(0, 0x90);
//    pcd8544_send(0, 0x20);
//    pcd8544_send(0, 0x0D);

    bool vertical_addressing = true;

    pcd8544_send(0, 0x21 | vertical_addressing << 1);
    pcd8544_send(0, 0xB1);
    pcd8544_send(0, 0x04);
    pcd8544_send(0, 0x14);
    pcd8544_send(0, 0x0C);
    pcd8544_send(0, 0x20 | vertical_addressing << 1);
    pcd8544_send(0, 0x0C);

//    pcd8544_send(1, 0x1F);
//    pcd8544_send(1, 0x05);
//    pcd8544_send(1, 0x07);
//    pcd8544_send(1, 0x00);

//    pcd8544_send(1, 0xAA);
    spi_nss(SPI1, Bit_SET);

//    pcd8544_clear();

}
