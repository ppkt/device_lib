#include "ir.h"

u16 pin;
GPIO_TypeDef *gpio;
ir_nec_state ir_nec_current_state;
u8 cnt = 0;

unsigned int data, prev_data;
u16 ir_nec_pulse_lengths[6];
bool ir_nec_repeat_last_command = false;

u8 global_device;
u8 global_command;

void ir_nec_init(u16 p, GPIO_TypeDef *g) {
    pin = p;
    gpio = g;
    ir_nec_current_state = IR_NEC_NONE;

    ir_nec_pulse_lengths[IR_NEC_NONE] = 0;
    ir_nec_pulse_lengths[IR_NEC_FIRST_BURST] = 9000;
    ir_nec_pulse_lengths[IR_NEC_SECOND_BURST] = 4500;
    ir_nec_pulse_lengths[IR_NEC_1] = 560;
    ir_nec_pulse_lengths[IR_NEC_NDEF] = 1;
    ir_nec_pulse_lengths[IR_NEC_SECOND_BURST_REPEAT] = 2250;
}

void ir_print_data(unsigned int p_data) {
//    static u8 dev_id;
//  static u8 dev_id2;
    static u8 cmd_id;
    static u8 cmd_id2;

//    dev_id =  (p_data & 0xFF000000) >> 24;
//  dev_id2 = (p_data & 0x00FF0000) >> 16;
    cmd_id =  (p_data & 0x0000FF00) >> 8;
    cmd_id2 = (p_data & 0x000000FF);

    if (cmd_id + cmd_id2 == 0xFF) {
//        printf("%u %u\r\n", dev_id, cmd_id);
        global_command = cmd_id;
        EXTI_GenerateSWInterrupt(EXTI_Line1);
    }
}

void ir_nec_reset_transmission() {
    if (ir_nec_current_state != IR_NEC_NONE) {
        ir_nec_current_state = IR_NEC_NONE;

        if (ir_nec_repeat_last_command) {
            ir_print_data(prev_data);
        } else {
            ir_print_data(data);
            prev_data = data;
        }

        data = 0;
        cnt = 0;
        ir_nec_repeat_last_command = false;
    }
}

bool ir_nec_check_tolerance(unsigned int received, ir_nec_state current_state) {
    unsigned int expected = ir_nec_pulse_lengths[current_state];

    if (current_state == IR_NEC_NONE ||
        current_state == IR_NEC_NDEF) {
        return true;
    } else if (current_state == IR_NEC_SECOND_BURST) {
        // We can receive long (4.5 ms) or short (2.25 ms) burst.
        if (received < 3000) {
            ir_nec_current_state = IR_NEC_SECOND_BURST_REPEAT;
            // ^ nasty hack, but we are can determine type of burst after receiving
            expected = ir_nec_pulse_lengths[IR_NEC_SECOND_BURST_REPEAT];
        } else {
            expected = ir_nec_pulse_lengths[IR_NEC_SECOND_BURST];
        }
    }

    u32 min = expected - (TOLERANCE / 100.0) * expected;
    u32 max = expected + (TOLERANCE / 100.0) * expected;
    if ((received >= min) && (received <= max)) {
        return true;
    }
    return false;
}

void ir_nec_state_machine(unsigned int time) {
//  BitAction bit = 1 - GPIO_ReadInputDataBit(gpio, pin); // Invert received value
//  GPIO_WriteBit(GPIOC, GPIO_Pin_8, bit);

    if (!ir_nec_check_tolerance(time, ir_nec_current_state)) {
        ir_nec_reset_transmission();
        return;
    }

    switch (ir_nec_current_state) {
        case IR_NEC_NONE:
            ir_nec_current_state = IR_NEC_FIRST_BURST;
            break;
        case IR_NEC_FIRST_BURST:
            ir_nec_current_state = IR_NEC_SECOND_BURST;
            break;
        case IR_NEC_SECOND_BURST:
            ir_nec_current_state = IR_NEC_1;
            break;
        case IR_NEC_1:
            ir_nec_current_state = IR_NEC_NDEF; // we can receive either 0 or 1
            break;
        case IR_NEC_NDEF:
            ir_nec_current_state = IR_NEC_1;
            if (time < 1000) {
                data = data << 1 | 1;
            } else {
                data = data << 1 | 0;
            }

//          if (cnt++ == 32) { //all data received
//
//          }
            break;
        case IR_NEC_SECOND_BURST_REPEAT:
            // repeat last message
//          ir_nec_repeat_last_command = true;
            ir_nec_current_state = IR_NEC_1;
            break;
        default:
            ir_nec_reset_transmission();
            break;
    }
}

u8 ir_nec_get_last_command() {
    u8 command = global_command;
    global_command = 0;
    return command;
}
