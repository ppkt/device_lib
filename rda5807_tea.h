#ifndef __RDA5807_TEA_H__
#define __RDA5807_TEA_H__
#include "i2c_dma.h"
#include "utils.h"
#include <stdio.h>
#include <stdbool.h>

// Commands for module in TEA5767HN compability mode

#define RDA5970_TEA_ADDRESS 0x60

typedef struct radio {
    char *name;
    unsigned int frequency;
    struct radio *next;
    struct radio *prev;
} radio;

typedef struct {
    radio *head;
    radio *tail;
    radio *current;
} radio_list;

radio_list *stations;

void rda5807_tea_init();
void rda5807_tea_set_frequency(unsigned int new_frequency);
bool rda5807_tea_mute();

#endif // __RDA5807_TEA_H__
