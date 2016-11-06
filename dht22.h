#ifndef __DHT22_H__
#define __DHT22_H__
#include "stdbool.h"
//#include "stdio.h"

#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_misc.h"

#include "common_lib/utils.h"

// Keep value in sync!
#define DHT22_STATE_SIZE 7
typedef enum {
    DHT22_NONE = 0,
    DHT22_INIT_PULL_DOWN  =1,
    DHT22_INIT_RELEASE = 2,
    DHT22_PULL_DOWN = 3,
    DHT22_RELEASE_1 = 4,
    DHT22_RELEASE_0 = 5,
    DHT22_EOT = 6,
} dht22_state;

void dht22_init(GPIO_TypeDef *gpio_, u16 pin_, u8 source_pin_, TIM_TypeDef *delay_timer_, TIM_TypeDef *interrupt_timer_);
//void dht22_trigger_state_machine(u32 timer, u8 bit);
bool dht22_reset_pulse();
//bool dht22_check_tolerance(u32 timer);
void dht22_decode_data();
u8 dht22_get_temperature();
u8 dht22_get_rh();
#endif //__DHT22_H__
