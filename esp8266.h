#ifndef __ESP8266_H__
#define __ESP8266_H__

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "stm32f10x_exti.h"

#include "common_lib/usart.h"
#include "common_lib/utils.h"

#include "main.h"

typedef enum {
    AT_CWLAP = 0, // Get AP list
    AT_CIFSR, // Get IP address
    AT_CWJAP,  // Connected AP info
    AT_CWQAP, // Disconnect from AP
    AT_CIPSERVER, // TCP Server
    AT, // Check module presence
    AT_RST, // Restarts device
    ATE0, // Disable echo
    ATE1, // Enable echo
    AT_CWMODE_DEF, // Wifi mode (1 = Station, 2 = AP, 3 = both)
    AT_CWMODE_CUR,
    AT_CIPMUX, // 0 for single connection mode, 1 for multiple connection
    AT_CIPSTART, // establish connetion with remote host
    AT_CIPSEND, // begin sending data
    AT_SEND_DATA, // send data
    AT_CLOSE, // close connection
    AT_GMR, // get version info
} Operation;

typedef enum {
    TYPE_SET_EXECUTE = 0,
    TYPE_QUERY,
    TYPE_TEST
} Type;

typedef enum {
    PARAMETER_CUR, // Value from RAM
    PARAMETER_DEF, // Value from Flash
} ParameterType;

typedef enum {
    ESP8266_MODE_STATION = 1,
    ESP8266_MODE_SOFTAP,
    ESP8266_MODE_SOFTAP_AND_STATION,
} Esp8266_mode;

void esp8266_init(void);
void esp8266_new_line(char* line);
void esp8266_at(void);
void esp8266_reset(void);
void esp8266_get_version(void);
void esp8266_echo_off(void);
void esp8266_echo_on(void);
void esp8266_set_mode(Esp8266_mode new_mode, bool persistent);
Esp8266_mode esp8266_get_mode(bool persistent);

#endif // __ESP8266_H__
