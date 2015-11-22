#include "esp8266.h"

static char custom_command[80];
//static uint8_t custom_char;

static char AT_command[] = "AT";

//static char OK_reply[] = "OK";
//static char ready_reply[] = "ready";

static char terminator[] = "\r\n";

bool busy = 0;

static Operation _operation;
static Type _type;

static Esp8266_mode _mode = ESP8266_MODE_SOFTAP;

void esp8266_enable_interrupts() {
    NVIC_InitTypeDef NVIC_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    // interrupt for decoding command
    EXTI_InitStructure.EXTI_Line = EXTI_Line1;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void esp8266_init(void) {
//    esp8266_enable_interrupts();
}

void esp8266_send_command(Type type, Operation operation) {
    while(busy) {}
    busy = 1;

    switch (operation) {
        case AT:
            usart1_print(AT_command);
            break;

        case AT_RST:
            usart1_print(AT_command);
            usart1_print("+RST");
            break;

        case AT_GMR:
            usart1_print(AT_command);
            usart1_print("+GMR");
            break;

        case ATE0:
            usart1_print(AT_command);
            usart1_print("E0");
            break;

        case ATE1:
            usart1_print(AT_command);
            usart1_print("E1");
            break;

        case AT_CWMODE_CUR:
            usart1_print(AT_command);

            if (type == TYPE_SET_EXECUTE) {
                usart1_print("+CWMODE_CUR=");
                usart1_print(custom_command);
            } else if (type == TYPE_QUERY) {
                usart1_print("+CWMODE_CUR?");
            }
            break;

        case AT_CWMODE_DEF:
            usart1_print(AT_command);

            if (type == TYPE_SET_EXECUTE) {
                usart1_print("+CWMODE_DEF=");
                usart1_print(custom_command);
            } else if (type == TYPE_QUERY) {
                usart1_print("+CWMODE_DEF?");
            }
            break;

        default:
            return;

    }

    _type = type;
    _operation = operation;

    usart1_print(terminator);
}

bool esp8266_parse_ok(char* string) {
    if (strcmp(string, "OK") == 0)
        return true;
    return false;
}

bool esp8266_parse_ready(char* string) {
    if (strcmp(string, "ready") == 0)
        return true;
    return false;
}

bool esp8266_parse_cwmode(char* string) {
    // First, we are waiting on mode from ESP i.e.: +CWMODE_CUR:2
    // then, just for OK

    static uint8_t state = 0;

    if (state == 0) {
        // Waiting for reply with current state
        static char reply[] = "+CWMODE_";
        char* ptr = strstr(string, reply);

        if (!ptr)
            return false;

        ptr += strlen(reply) + 4; // skip +CWMODE_ + CUR/DEF + ":" in reply, see above

        _mode = *ptr - '0';

        state = 1;

    } else if (state == 1) {
        // Waiting for OK
        if (esp8266_parse_ok(string)) {
            state = 0;
            return true;
        }
    }

    return true;
}

void esp8266_parse_string(char *incoming) {
    switch (_operation) {
        case AT:
        case AT_GMR:
        case ATE0:
        case ATE1:
            if (!esp8266_parse_ok(incoming))  // Not the response we are looking for
                return;
            break;


        case AT_CWMODE_CUR:
        case AT_CWMODE_DEF:
            if (_type == TYPE_QUERY) {
                if (!esp8266_parse_cwmode(incoming))
                    return; // Still waiting
            } else {
                if (!esp8266_parse_ok(incoming))
                    return; // Still waiting
            }
            break;


        case AT_RST:
            if (!esp8266_parse_ready(incoming))
                return;
            break;


        default:
            return;
    }

    busy = false;

}

void esp8266_wait_for_response() {
    // Parse incoming lines
    uint16_t attempt = 0;
    static uint16_t max_attempt = 10000;
    while (busy && attempt++ < max_attempt) {
        __WFI();

    }

    if (busy) {
        GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_SET);
    }

}

void esp8266_at(void) {
    esp8266_send_command(TYPE_SET_EXECUTE, AT);
    esp8266_wait_for_response();
}

void esp8266_reset(void) {
    esp8266_send_command(TYPE_SET_EXECUTE, AT_RST);
    esp8266_wait_for_response();
}

void esp8266_get_version(void) {
    esp8266_send_command(TYPE_SET_EXECUTE, AT_GMR);
    esp8266_wait_for_response();
}

void esp8266_echo_off(void) {
    esp8266_send_command(TYPE_SET_EXECUTE, ATE0);
    esp8266_wait_for_response();
}

void esp8266_echo_on(void) {
    esp8266_send_command(TYPE_SET_EXECUTE, ATE1);
    esp8266_wait_for_response();
}

void esp8266_set_mode(Esp8266_mode new_mode, bool persistent) {

    sprintf(custom_command, "%d", new_mode);

    if (persistent)
        esp8266_send_command(TYPE_SET_EXECUTE, AT_CWMODE_DEF);
    else
        esp8266_send_command(TYPE_SET_EXECUTE, AT_CWMODE_CUR);

    esp8266_wait_for_response();
}

Esp8266_mode esp8266_get_mode(bool persistent) {
    if (persistent)
        esp8266_send_command(TYPE_QUERY, AT_CWMODE_DEF);
    else
        esp8266_send_command(TYPE_QUERY, AT_CWMODE_CUR);

    esp8266_wait_for_response();

    return _mode;
}

void esp8266_new_line(char* line) {
    esp8266_parse_string(line);

    free(line);
}
