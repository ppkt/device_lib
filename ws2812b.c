#include "ws2812b.h"

#define PWM_TIMER   TIM3
//#define DMA_STREAM  DMA1_Stream2
#define DMA_TCIF    DMA_FLAG_TCIF2
#define DMA_CHANNEL DMA_Channel_5
#define DMA_SOURCE  TIM_DMA_Update

RgbColor HsvToRgb(HsvColor hsv)
{
    RgbColor rgb;
    unsigned char region, p, q, t;
    unsigned int h, s, v, remainder;

    if (hsv.s == 0)
    {
        rgb.r = hsv.v;
        rgb.g = hsv.v;
        rgb.b = hsv.v;
        return rgb;
    }

    // converting to 16 bit to prevent overflow
    h = hsv.h;
    s = hsv.s;
    v = hsv.v;

    region = h / 43;
    remainder = (h - (region * 43)) * 6;

    p = (v * (255 - s)) >> 8;
    q = (v * (255 - ((s * remainder) >> 8))) >> 8;
    t = (v * (255 - ((s * (255 - remainder)) >> 8))) >> 8;

    switch (region)
    {
        case 0:
            rgb.r = v;
            rgb.g = t;
            rgb.b = p;
            break;
        case 1:
            rgb.r = q;
            rgb.g = v;
            rgb.b = p;
            break;
        case 2:
            rgb.r = p;
            rgb.g = v;
            rgb.b = t;
            break;
        case 3:
            rgb.r = p;
            rgb.g = q;
            rgb.b = v;
            break;
        case 4:
            rgb.r = t;
            rgb.g = p;
            rgb.b = v;
            break;
        default:
            rgb.r = v;
            rgb.g = p;
            rgb.b = q;
            break;
    }

    return rgb;
}

HsvColor RgbToHsv(RgbColor rgb)
{
    HsvColor hsv;
    unsigned char rgbMin, rgbMax;

    rgbMin = rgb.r < rgb.g ? (rgb.r < rgb.b ? rgb.r : rgb.b) : (rgb.g < rgb.b ? rgb.g : rgb.b);
    rgbMax = rgb.r > rgb.g ? (rgb.r > rgb.b ? rgb.r : rgb.b) : (rgb.g > rgb.b ? rgb.g : rgb.b);

    hsv.v = rgbMax;
    if (hsv.v == 0)
    {
        hsv.h = 0;
        hsv.s = 0;
        return hsv;
    }

    hsv.s = 255 * ((long)(rgbMax - rgbMin)) / hsv.v;
    if (hsv.s == 0)
    {
        hsv.h = 0;
        return hsv;
    }

    if (rgbMax == rgb.r)
        hsv.h = 0 + 43 * (rgb.g - rgb.b) / (rgbMax - rgbMin);
    else if (rgbMax == rgb.g)
        hsv.h = 85 + 43 * (rgb.b - rgb.r) / (rgbMax - rgbMin);
    else
        hsv.h = 171 + 43 * (rgb.r - rgb.g) / (rgbMax - rgbMin);

    return hsv;
}

//#define TIM_PERIOD          51
//#define TIM_COMPARE_HIGH    33
//#define TIM_COMPARE_LOW     (TIM_PERIOD - TIM_COMPARE_HIGH) - 1

TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;
GPIO_InitTypeDef GPIO_InitStructure;
DMA_InitTypeDef DMA_InitStructure;
NVIC_InitTypeDef NVIC_InitStructure;

/* Buffer that holds one complete DMA transmission
 *
 * Ensure that this buffer is big enough to hold
 * all data bytes that need to be sent
 *
 * The buffer size can be calculated as follows:
 * number of LEDs * 24 bytes + 42 bytes (reset pulse)
 *
 * This leaves us with a maximum string length of
 * (2^16 bytes per DMA stream - 42 bytes)/24 bytes per LED = 2728 LEDs
 */
#define LED (10)
#define BUFFER_SIZE (LED * 24 + 42)
uint8_t LED_BYTE_Buffer[BUFFER_SIZE];

#define TIM1_CCR3_Address    0x40012C3C

uint16_t TimerPeriod = 0;
uint16_t TIM_COMPARE_LOW = 0;
uint16_t TIM_COMPARE_HIGH = 0;

bool transfer_in_progress = false;

DMA_InitTypeDef DMA_InitStructure;

void ws2812b_init(void)
{
    memset(LED_BYTE_Buffer, 0x00, BUFFER_SIZE);

    /* TIM1, GPIOA and GPIOB clock enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 | RCC_APB2Periph_GPIOA |
                           RCC_APB2Periph_GPIOB, ENABLE);
    /* DMA clock enable */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;

    /* GPIOA Configuration: Channel 3 as alternate function push-pull */
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//    GPIO_Init(GPIOA, &GPIO_InitStructure);

//    GPIO_ResetBits(GPIOA, GPIO_Pin_10);

    /* DMA1 Channel5 Config */
    DMA_DeInit(DMA1_Channel5);

    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)TIM1_CCR3_Address;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)LED_BYTE_Buffer;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_BufferSize = BUFFER_SIZE;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;

    DMA_Init(DMA1_Channel5, &DMA_InitStructure);
//    DMA_ITConfig(DMA1_Channel5, DMA_IT_TC, ENABLE);

//    NVIC_InitTypeDef NVIC_InitStructure;
//    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
//    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel5_IRQn;
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//    NVIC_Init(&NVIC_InitStructure);

    /* TIM1 DMA Transfer example -------------------------------------------------
    TIM1CLK = SystemCoreClock, Prescaler = 0, TIM1 counter clock = SystemCoreClock

    -----------------------------------------------------------------------------*/
    /* Compute the value to be set in ARR register to generate signal frequency at 800 Khz */
    TimerPeriod = (SystemCoreClock / 800000 ) - 1;

    uint16_t cycle = 1250; //us
    uint16_t zero_duty_cycle = (400/*us*/ * 100/*%*/) / cycle;
    uint16_t one_duty_cycle  = (800/*us*/ * 100/*%*/) / cycle;

    // Duty cycle 32%
    TIM_COMPARE_LOW = (uint16_t) (((uint32_t) zero_duty_cycle * (TimerPeriod - 1)) / 100);
    // 64 %
    TIM_COMPARE_HIGH = (uint16_t) (((uint32_t) one_duty_cycle * (TimerPeriod - 1)) / 100);

    /* TIM1 Peripheral Configuration --------------------------------------------*/
    /* Time Base configuration */
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period = TimerPeriod;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;

    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

    /* Channel 3 Configuration in PWM mode */
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;

    TIM_OC3Init(TIM1, &TIM_OCInitStructure);

    /* TIM1 Update DMA Request enable */
    TIM_DMACmd(TIM1, TIM_DMA_Update, ENABLE);

    // TIM1 counter enable
    TIM_Cmd(TIM1, ENABLE);

    /* DMA1 Channel5 enable */
    DMA_Cmd(DMA1_Channel5, ENABLE);

    /* Main Output Enable */
    TIM_CtrlPWMOutputs(TIM1, ENABLE);

}

void WS2812_send(u8 r, u8 g, u8 b)
{
    u8 i, l;
    uint16_t memaddr = 0;

//    memset(LED_BYTE_Buffer, 0, BUFFER_SIZE);

    while(memaddr < 5) {
        LED_BYTE_Buffer[memaddr] = 0;
        memaddr++;
    }

//    GPIO_SetBits(GPIOA, GPIO_Pin_9);
    // fill transmit buffer with correct compare values to achieve
    // correct pulse widths according to color values
    for (l = 0; l < LED; ++l) {
        // Green
        for (i = 0; i < 8; ++i) {
            if ( (g << i) & 0x80 ) {  // data sent MSB first, j = 0 is MSB j = 7 is LSB
                LED_BYTE_Buffer[memaddr] = TIM_COMPARE_HIGH;    // compare value for logical 1
            } else {
                LED_BYTE_Buffer[memaddr] = TIM_COMPARE_LOW;     // compare value for logical 0
            }
            memaddr++;
        }

        // Red
        for (i = 0; i < 8; ++i) {
            if ( (r << i) & 0x80 ) {
                LED_BYTE_Buffer[memaddr] = TIM_COMPARE_HIGH;
            } else {
                LED_BYTE_Buffer[memaddr] = TIM_COMPARE_LOW;
            }
            memaddr++;
        }

        // Blue
        for (i = 0; i < 8; ++i) {
          if ( (b << i) & 0x80 ) {
              LED_BYTE_Buffer[memaddr] = TIM_COMPARE_HIGH;
          } else {
              LED_BYTE_Buffer[memaddr] = TIM_COMPARE_LOW;
          }
          memaddr++;
        }
    }

    // Add needed delay at end of byte cycle (according to datasheet,
    // Reset pulse should last at least 50 us
    while(memaddr < BUFFER_SIZE) {
        LED_BYTE_Buffer[memaddr] = 0;
        memaddr++;
    }

//    while (transfer_in_progress);

//    GPIO_ResetBits(GPIOA, GPIO_Pin_9);
    // Restart DMA
//    DMA_Init(DMA1_Channel5, &DMA_InitStructure);
//    DMA_Cmd(DMA1_Channel5, ENABLE);
    transfer_in_progress = true;



//    GPIO_SetBits(GPIOA, GPIO_Pin_9);

//    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
//    TIM_OC3Init(TIM1, &TIM_OCInitStructure);
//    TIM_DMACmd(TIM1, TIM_DMA_Update, ENABLE);
//    TIM_Cmd(TIM1, ENABLE);
//    TIM_CtrlPWMOutputs(TIM1, ENABLE);

//    // Wait for end of transmission
//    while(!DMA_GetFlagStatus(DMA1_FLAG_TC5)) {};
////    TIM_Cmd(TIM1, DISABLE);
//
//    // Disable DMA
//    DMA_Cmd(DMA1_Channel5, DISABLE);
//    // Clean End Of Transmission flag
//    DMA_ClearFlag(DMA1_FLAG_TC5);

}

//void DMA1_Channel5_IRQHandler(void)
//{
//    if (DMA_GetITStatus(DMA1_IT_TC5)) {
//        DMA_Cmd(DMA1_Channel5, DISABLE);
//        DMA_ClearITPendingBit(DMA1_IT_TC5);
//        transfer_in_progress = false;
//    }
//}
