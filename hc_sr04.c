#include "hc_sr04.h"

void sr04_init(void) {
    /* Enable GPIO clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    // Use PB9 - trigger
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // Use PB8 - echo
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void sr04_trigger(void) {
    GPIO_SetBits(GPIOB, GPIO_Pin_9);
    delay_us(TIM2, 12);
    GPIO_ResetBits(GPIOB, GPIO_Pin_9);
}

uint16_t sr04_get_delay(void) {
    while (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_8) == Bit_RESET){}
    TIM2->CNT = 0;
    while (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_8) == Bit_SET){}
    return TIM2->CNT;
}
