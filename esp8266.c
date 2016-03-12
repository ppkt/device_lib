#include "esp8266.h"

static char custom_command[ESP8266_MAX_DATA_LENGTH];

static char AT_command[] = "AT";
static char terminator[] = "\r\n";

bool busy = 0;

static Operation _operation;
static Type _type;

static Esp8266_mode _mode = ESP8266_MODE_SOFTAP;

static uint8_t _last_error = 0;

// pointer to callback function, called on each incoming packet
static void (*_callback)(char* string, uint8_t size) = 0;

// pointer to timer
TIM_TypeDef *timer;

void esp8266_init(TIM_TypeDef *delay_timer) {
    timer = delay_timer;
    esp8266_at();
    delay_ms(timer, 100);
    esp8266_at();
    delay_ms(timer, 100);
    esp8266_echo_off();
    delay_ms(timer, 100);
}

void esp8266_selfcheck(void) {
    // Performs self check of available AT commands, note - AP name and password
    // are provided during compilation
    // Note: all commands are executed in "temporary mode", i.e. these values
    // won't be stored in flash memory

    uint8_t error = 0;

    // check if chip is alive
    esp8266_at();
    delay_ms(timer, 100);

    // get some information about AP
    error = esp8266_get_ap_info(false);
    delay_ms(timer, 100);
//    if (error)
//        hacf();

    // get IP address
    char* ip_address = esp8266_get_ip_address(false);
    free(ip_address);
    delay_ms(timer, 100);

    // try setting and checking different modes
    esp8266_set_mode(ESP8266_MODE_SOFTAP, false);
    delay_ms(timer, 100);

    esp8266_get_mode(false);
    delay_ms(timer, 100);

    esp8266_set_mode(ESP8266_MODE_STATION, false);
    delay_ms(timer, 100);

    esp8266_get_mode(false);
    delay_ms(timer, 100);

    esp8266_set_mode(ESP8266_MODE_SOFTAP_AND_STATION, false);
    delay_ms(timer, 100);

    esp8266_get_mode(false);
    delay_ms(timer, 100);

    esp8266_get_mode(true);
    delay_ms(timer, 100);

    // join AP
    error = esp8266_join_ap(AP_NAME, AP_PASSWORD, "", false);
    if (error)
        hacf();

    /********************************RESET*************************************/
    esp8266_reset();
    delay_ms(timer, 100);

    esp8266_at();
    delay_ms(timer, 100);

    error = esp8266_get_ap_info(false);
    if (error)
        hacf();

    delay_ms(timer, 100);

    esp8266_get_version();
    delay_ms(timer, 100);

    esp8266_at();
    delay_ms(timer, 100);

    esp8266_echo_on();
    delay_ms(timer, 100);

    esp8266_at();
    delay_ms(timer, 100);

    esp8266_echo_off();
    delay_ms(timer, 100);

    esp8266_at();
    delay_ms(timer, 100);

    error = esp8266_get_ap_info(false);
    if (error)
        hacf();
    delay_ms(timer, 100);

    esp8266_at();
    delay_ms(timer, 100);

//    esp8266_set_static_ip("192.168.0.250", "", "", false);

//    ip_address = esp8266_get_ip_address(false);
//    free(ip_address);
//    delay_ms(TIM2, 100);

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

        case AT_CIPSTA_CUR:
        case AT_CIPSTA_DEF:
            usart1_print(AT_command);
            usart1_print("+CIPSTA_");

            if (operation == AT_CIPSTA_CUR)
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

        case AT_CIPSTART:
            usart1_print(AT_command);
            usart1_print("+CIPSTART=");
            usart1_print(custom_command);
            break;

        case AT_CIPSEND:
            usart1_print(AT_command);
            usart1_print("+CIPSEND=");
            usart1_print(custom_command);
            break;

        case AT_CIPSEND_DATA:
            usart1_print(custom_command);
            break;

        case AT_CIPCLOSE:
            usart1_print(AT_command);
            usart1_print("+CIPCLOSE");
            break;

        case AT_CIPSTAMAC_CUR:
        case AT_CIPSTAMAC_DEF:
            usart1_print(AT_command);
            usart1_print("+CIPSTAMAC_");

            if (operation == AT_CIPSTAMAC_CUR)
                usart1_print("CUR");
            else
                usart1_print("DEF");

            if (type == TYPE_QUERY) {
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

bool esp8266_parse_error(char* string) {
    if (strcmp(string, "ERROR") == 0)
        return true;
    return false;
}

bool esp8266_parse_ready(char* string) {
    if (strcmp(string, "ready") == 0)
        return false;
    // Note: code below will work only if AP information is stored permanently
    if (strcmp(string, "WIFI GOT IP") == 0)
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

bool esp8266_parse_cipsta(char* string) {

    static uint8_t state = 0;
    static char match[] = "+CIPSTA_";

    if (state == 0) {
        if (strncmp(string, match, strlen(match)) == 0) {
            // Reply in format: +CIPSTA_CUR:ip:"<IP>"
            char* ip_address = &string[0] + strlen(match) + 7 + 1;

            state = 1;
            strncpy(custom_command, ip_address, strlen(ip_address) - 1);

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

bool esp8266_parse_cipstart(char* string) {
    static uint8_t state = 0;

    if (state == 0) {
        if (strcmp(string, "ALREADY CONNECTED") == 0) { // Connection already exists
            _last_error = 1;
            state = 1;
            return false;
        }

        if (strcmp(string, "CONNECT") == 0) { // Connection established
            state = 1;
            return false;
        }
    }

    if (state == 1) {
        if (esp8266_parse_error(string)) { // Problem during establishing connection
            if (_last_error != 0)
                _last_error = 2;
            state = 0;
            return true;
        }

        if (esp8266_parse_ok(string)) { // New connection created
            state = 0;
            return true;
        }
    }

    return false;
}

bool esp8266_parse_cipsend(char *string) {
    if (strcmp(string, "SEND OK") == 0) {
        return true;
    }
    return false;
}

bool esp8266_parse_cipclose(char *string) {
    // if there is no connection it's safe to ignore "error" message
    if (esp8266_parse_error(string) || esp8266_parse_ok(string)) {
        return true;
    }
    return false;
}

bool esp8266_parse_cipstamac(char *string) {
    static uint8_t state = 0;
    static char match[] = "+CIPSTAMAC_";

    if (state == 0) {
        if (strncmp(string, match, strlen(match)) == 0) {
            // Reply in format: +CIPSTAMAC_DEF="<MAC>"
            char* mac_address = &string[0] + strlen(match) + 4 + 1;

            state = 1;
            strncpy(custom_command, mac_address, strlen(mac_address) - 1);

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

bool esp8266_parse_packet(char *string) {
    static char match[] = "+IPD";

    if (strncmp(string, match, strlen(match)) == 0) {
        // Incoming packet is in format:
        // +IPD,<len>:<data>

        char* packet = &string[0] + strlen(match) + 1; // remove "+IPD,"

        // start of data
        char* data = strstr(packet, ":") + 1;

        // FIXME: parse received length of packet instead using strlen
        if (_callback)
            _callback(data, strlen(data));

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

        case AT_CIPSTA_CUR:
        case AT_CIPSTA_DEF:
            if (_type == TYPE_SET_EXECUTE) {
                if (!esp8266_parse_ok(incoming))
                    return;
            } else if (_type == TYPE_QUERY) {
                if (!esp8266_parse_cipsta(incoming))
                    return;
            }
            break;

        case AT_CIPSTART:
            if (!esp8266_parse_cipstart(incoming))
                return;
            break;

        case AT_CIPSEND:
            if (_type == TYPE_SET_EXECUTE) {
                if (!esp8266_parse_ok(incoming))
                    return;
            }
            break;

        case AT_CIPSEND_DATA:
            if (!esp8266_parse_cipsend(incoming))
                return;
            break;

        case AT_CIPCLOSE:
            if (!esp8266_parse_cipclose(incoming))
                return;
            break;

        case AT_CIPSTAMAC_CUR:
        case AT_CIPSTAMAC_DEF:
            if (!esp8266_parse_cipstamac(incoming))
                return;

            break;

        default:
            if (!esp8266_parse_packet(incoming))
                return;
            break;
    }

    busy = false;
    _operation = NOP;

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
    // Note - bssid is NOT required, please provide empty string in this case

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

void esp8266_set_static_ip(char* ip_address, char* gateway, char* netmask,
                           bool persistent) {
    // Note: gateway and netmask are NOT mandatory, but you have to provide
    // both addresses. Provide empty string if there addresses are not used

    // Note: setting IP address disables internal DHCP client and vice versa

    if (strlen(gateway) > 0 && strlen(netmask) > 0)
        sprintf(custom_command, "\"%s\",\"%s\",\"%s\"",
                ip_address, gateway, netmask);
    else
        sprintf(custom_command, "\"%s\"", ip_address);

    if (persistent)
        esp8266_send_command(TYPE_SET_EXECUTE, AT_CIPSTA_DEF);
    else
        esp8266_send_command(TYPE_SET_EXECUTE, AT_CIPSTA_CUR);

    esp8266_wait_for_response();
}

char* esp8266_get_ip_address(bool persistent) {
    if (persistent)
        esp8266_send_command(TYPE_QUERY, AT_CIPSTA_DEF);
    else
        esp8266_send_command(TYPE_QUERY, AT_CIPSTA_CUR);
    esp8266_wait_for_response();

    return strdup(custom_command);
}

uint8_t esp8266_establish_connection(Esp8266_protocol protocol, char *ip_address,
                                     uint16_t port) {

    if (protocol == ESP8266_PROTOCOL_UDP)
        sprintf(custom_command, "\"UDP\",\"%s\",%d", ip_address, port);
    else  // TODO: Not implemented / tested
        return 1;

    esp8266_send_command(TYPE_SET_EXECUTE, AT_CIPSTART);
    esp8266_wait_for_response();

    uint8_t error = _last_error;
    _last_error = 0;
    return error;
}

// Establish two-way connection (ie. for sending AND receiving data)
uint8_t esp8266_establish_two_way_connection(Esp8266_protocol protocol,
                                             char *ip_address,
                                             uint16_t source_port,
                                             uint16_t destination_port,
                                             uint8_t mode,
                                             void (*callback)(char* string,
                                                              uint8_t size)) {

    _callback = callback;

    if (protocol == ESP8266_PROTOCOL_UDP)
        sprintf(custom_command, "\"UDP\",\"%s\",%d,%d,%d", ip_address,
                source_port, destination_port, mode);
    else  // TODO: Not implemented / tested
        return 1;

    esp8266_send_command(TYPE_SET_EXECUTE, AT_CIPSTART);
    esp8266_wait_for_response();

    uint8_t error = _last_error;
    _last_error = 0;
    return error;
}

uint8_t esp8266_send_data(char* buffer) {
    size_t buffer_length = strlen(buffer);
    if (buffer_length  > ESP8266_MAX_DATA_LENGTH)
        return 1;

    sprintf(custom_command, "%d", buffer_length);

    esp8266_send_command(TYPE_SET_EXECUTE, AT_CIPSEND);
    esp8266_wait_for_response();

    delay_ms(timer, 100);

    strcpy(custom_command, buffer);

    esp8266_send_command(TYPE_SET_EXECUTE, AT_CIPSEND_DATA);
    esp8266_wait_for_response();

    return 0;
}

void esp8266_close_connection(void) {
    esp8266_send_command(TYPE_SET_EXECUTE, AT_CIPCLOSE);
    esp8266_wait_for_response();
}

uint8_t esp8266_udp_send(char* ip_address, uint16_t port, char* data) {
    uint8_t error = esp8266_establish_connection(ESP8266_PROTOCOL_UDP,
                                                 ip_address, port);
    delay_ms(timer, 100);
    if (error)
        return 1;

    error = esp8266_send_data(data);
    delay_ms(timer, 100);

    if (error)
        return 2;

    esp8266_close_connection();
    delay_ms(timer, 100);

    return 0;
}

char* esp8266_get_mac_address(bool persistent) {
    if (persistent)
        esp8266_send_command(TYPE_QUERY, AT_CIPSTAMAC_DEF);
    else
        esp8266_send_command(TYPE_QUERY, AT_CIPSTAMAC_CUR);
    esp8266_wait_for_response();

    return strdup(custom_command);
}

void esp8266_new_line(char* line) {
    esp8266_parse_string(line);

    free(line);
}
