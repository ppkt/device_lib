#ifndef __ESP8266_H__
#define __ESP8266_H__

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "stm32f10x_exti.h"

#include "common_lib/usart.h"
#include "common_lib/utils.h"

#define ESP8266_MAX_DATA_LENGTH (2048)

typedef enum {
    NOP,  // No operation
    AT_CWJAP_DEF,  // Connect to AP or get info about connected AP
    AT_CWJAP_CUR,
    AT, // Check module presence
    AT_RST, // Restarts device
    ATE0, // Disable echo
    ATE1, // Enable echo
    AT_CWMODE_DEF, // Wifi mode (1 = Station, 2 = AP, 3 = both)
    AT_CWMODE_CUR,
    AT_GMR, // get version info
    AT_CIPSTA_DEF, // set/get IP address in Station Mode
    AT_CIPSTA_CUR,
    AT_CIPSTART, // start new IP connection
    AT_CIPSEND, // send data (after establishing connection)
    AT_CIPSEND_DATA, // fake command to trigger sending of actual data
    AT_CIPCLOSE, // close connection
    AT_CIPSTAMAC_DEF, // sef/get MAC address in Station Mode
    AT_CIPSTAMAC_CUR,
} Operation;

typedef enum {
    TYPE_SET_EXECUTE = 0,
    TYPE_QUERY,
    TYPE_TEST
} Type;

typedef enum {
    PARAMETER_CUR = 0, // Value from RAM (temporary)
    PARAMETER_DEF, // Value from Flash (persistent)
} ParameterType;

typedef enum {
    ESP8266_MODE_STATION = 1,
    ESP8266_MODE_SOFTAP,
    ESP8266_MODE_SOFTAP_AND_STATION,
} Esp8266_mode;

typedef enum {
    ESP8266_PROTOCOL_UDP,
    ESP8266_PROTOCOL_TCP,
} Esp8266_protocol;

void esp8266_init(TIM_TypeDef *delay_timer);
void esp8266_selfcheck(void);
void esp8266_new_line(char* line);
void esp8266_at(void);
void esp8266_reset(void);
void esp8266_get_version(void);
void esp8266_echo_off(void);
void esp8266_echo_on(void);
void esp8266_set_mode(Esp8266_mode new_mode, bool persistent);
Esp8266_mode esp8266_get_mode(bool persistent);
uint8_t esp8266_join_ap(char* ssid, char* pwd, char* bssid, bool persistent);
uint8_t esp8266_get_ap_info(bool persistent);
void esp8266_set_static_ip(char* ip_address, char* gateway, char* netmask,
                           bool persistent);
char* esp8266_get_ip_address(bool persistent);
uint8_t esp8266_establish_connection(Esp8266_protocol protocol,
                                     char* ip_address, uint16_t port);
uint8_t esp8266_establish_two_way_connection(Esp8266_protocol protocol,
                                             char *ip_address,
                                             uint16_t source_port,
                                             uint16_t destination_port,
                                             uint8_t mode,
                                             void (*callback)(char* string,
                                                              uint8_t size));
uint8_t esp8266_send_data(char* buffer);
void esp8266_close_connection(void);
uint8_t esp8266_udp_send(char* ip_address, uint16_t port, char *data);
char* esp8266_get_mac_address(bool persistent);

#endif // __ESP8266_H__
