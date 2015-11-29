#ifndef __IR_H__
#define __IR_H__
#include <stm32f10x.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_exti.h>
#include <stdbool.h>
#include <stdio.h>

#define TOLERANCE 20

typedef enum {
    IR_NEC_NONE = 0,
    IR_NEC_NDEF,
    IR_NEC_FIRST_BURST,
    IR_NEC_SECOND_BURST,
    IR_NEC_SECOND_BURST_REPEAT,
    IR_NEC_1
} ir_nec_state;

void ir_nec_init(u16 pin, GPIO_TypeDef *gpio);
void ir_nec_state_machine(unsigned int time);
void ir_nec_reset_transmission();
u8 ir_nec_get_last_command();
#endif // __IR_H__
