#include "hx711.h"

static uint32_t zero = 0;
static TIM_TypeDef *timer;
float gram = 151962.0f;

void hx711_gpio_init() {
    /* Enable GPIO clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_WriteBit(GPIOA, GPIO_Pin_11, 0);


    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void hx711_init(TIM_TypeDef *t) {
    hx711_gpio_init();
    timer = t;

    // perform calibration
    zero = 0;

    // dummy read to set gain
    hx711_single_read();
    delay_ms(timer, 1000);

    zero = hx711_avg_read(16);
}

int32_t hx711_single_read(void) {
    uint8_t i;
    int32_t reading = 0;

    for (i = 0; i < 27; ++i) {
        GPIO_WriteBit(GPIOA, GPIO_Pin_11, 1);
        reading = reading << 1;
        u8 bit = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_12);
        if (bit)
            reading++;
        GPIO_WriteBit(GPIOA, GPIO_Pin_11, 0);
    }
    reading = reading << (32 - i);

    return reading - zero;
}

int32_t hx711_avg_read(uint8_t samples) {
    int64_t adc_data = 0;
    uint8_t i;

    for (i = 0; i < samples; ++i) {
        adc_data += hx711_single_read();
        delay_ms(timer, 100);
    }
    adc_data /= samples;
    return adc_data;
}

float hx711_read_gram(void) {
    int32_t read = hx711_avg_read(8);

    return (float)read / gram;
}
