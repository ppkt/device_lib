#include "esp8266.h"

static char custom_command[80];

static char AT_command[] = "AT";
static char terminator[] = "\r\n";



bool busy = 0;

static Operation _operation;
static Type _type;

static Esp8266_mode _mode = ESP8266_MODE_SOFTAP;

static uint8_t _last_error = 0;

void esp8266_init(void) {}

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
        case ATE1:
            usart1_print(AT_command);
            usart1_print("E");

            if (operation == ATE0)
                usart1_print("0");
            else
                usart1_print("1");
            break;

        case AT_CWMODE_CUR:
        case AT_CWMODE_DEF:
            usart1_print(AT_command);
            usart1_print("+CWMODE_");

            if (operation == AT_CWMODE_CUR)
                usart1_print("CUR");
            else
                usart1_print("DEF");


            if (type == TYPE_SET_EXECUTE) {
                usart1_print("=");
                usart1_print(custom_command);
            } else if (type == TYPE_QUERY) {
                usart1_print("?");
            }
            break;

        case AT_CWJAP_CUR:
        case AT_CWJAP_DEF:
            usart1_print(AT_command);
            usart1_print("+CWJAP_");

            if (operation == AT_CWJAP_CUR)
                usart1_print("CUR");
            else
                usart1_print("DEF");

            if (type == TYPE_SET_EXECUTE) {
                usart1_print("=");
                usart1_print(custom_command);
            } else if (type == TYPE_QUERY) {
                usart1_print("?");
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

bool esp8266_parse_cwjap_set(char* string) {
    static uint8_t state = 0;

    if (state == 0) {
        // Happy case, everything is OK
        if (esp8266_parse_ok(string)) {
            return true;
        }

        // Fail case
        static char reply[] = "+CWJAP:";
        char* ptr = strstr(string, reply);

        if (ptr) {
            // Here we received error code in case of failed attempt to join AP
            // but we are still waiting for FAIL command

            state = 1;
            ptr += strlen(reply);
            _last_error = *ptr - '0';

            // You can attach debugger here and check state of _last_error
            // variable:
            // 1 - connection timeout
            // 2 - wrong password
            // 3 - AP not found
            // 4 - connection fail

            return false;
        }
    } else if (state == 1) {
        static char reply[] = "FAIL";
        char* ptr = strstr(string, reply);

        if (ptr) {
            state = 0;
            return true;
        }
    }

    return false;
}

bool esp8266_parse_cwjap_query(char* string) {
    // First, we are waiting for result of query in form:
    // +CWJAP_CUR:<ssid>,<bssid>,<channel>,<rssi>
    // or
    // No AP
    // after this, a single OK should be received

    static uint8_t state = 0;
    static char match[] = "+CWJAP_";

    if (state == 0) {
        if (strcmp(string, "No AP") == 0) {
            // No connection with Access Point
            state = 1;
            _last_error = 1;
            return false;
        }

        if (strncmp(string, match, strlen(match)) == 0) {
            // ^ Reply should start with this string
            // Connected
            // TODO: Added returning information about Access Point (in struct)
            state = 1;
            _last_error = 0;
            return false;
        }
    } else if (state == 1) {
        if (esp8266_parse_ok(string)) {
            state = 0;
            return true;
        }
    }
    return false;
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

        case AT_CWJAP_CUR:
        case AT_CWJAP_DEF:
            if (_type == TYPE_SET_EXECUTE) {
                if (!esp8266_parse_cwjap_set(incoming))
                    return;
            } else if (_type == TYPE_QUERY) {
                if (!esp8266_parse_cwjap_query(incoming))
                    return;
            }
            break;

        default:
            return;
    }

    busy = false;

}

void esp8266_wait_for_response() {
    // Parse incoming lines
    uint16_t attempt = 0;
    static uint16_t max_attempt = 20;
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

uint8_t esp8266_join_ap(char* ssid, char* pwd, char* bssid, bool persistent) {
    // TODO: special characters (, " /) in SSID and PWD should be escaped by "/"
    // character: ab/,c > ab///,c
    //            12345"/" > 12345/"///"


    if (strlen(bssid) > 0) {
        sprintf(custom_command, "\"%s\",\"%s\",\"%s\"", ssid, pwd, bssid);
    } else {
        sprintf(custom_command, "\"%s\",\"%s\"", ssid, pwd);
    }

    if (persistent) {
        esp8266_send_command(TYPE_SET_EXECUTE, AT_CWJAP_DEF);
    } else {
        esp8266_send_command(TYPE_SET_EXECUTE, AT_CWJAP_CUR);
    }

    esp8266_wait_for_response();

    uint8_t error = _last_error;
    _last_error = 0;
    return error;
}

uint8_t esp8266_get_ap_info(bool persistent) {
    if (persistent)
        esp8266_send_command(TYPE_QUERY, AT_CWJAP_DEF);
    else
        esp8266_send_command(TYPE_QUERY, AT_CWJAP_CUR);
    esp8266_wait_for_response();

    uint8_t error = _last_error;
    _last_error = 0;
    return error;
}

void esp8266_new_line(char* line) {
    esp8266_parse_string(line);

    free(line);
}
