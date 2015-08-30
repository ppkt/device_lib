#ifndef __RDA5807_TEA_H__
#define __RDA5807_TEA_H__

#include <stdio.h>
#include <stdbool.h>

#include "common_lib/i2c_dma.h"
#include "common_lib/utils.h"

// Commands for module in TEA5767HN compability mode

#define RDA5970_TEA_ADDRESS 0x60

void rda5807_tea_init();
void rda5807_tea_set_frequency(unsigned int new_frequency);
bool rda5807_tea_mute();

#endif // __RDA5807_TEA_H__
