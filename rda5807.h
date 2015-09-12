#ifndef __RDA5807_H__
#define __RDA5807_H__

#include <stdio.h>
#include <stdbool.h>

#include "common_lib/i2c_dma.h"
#include "common_lib/utils.h"

#define RDA5970_ADDRESS 0x11
#define RDA5970_ID 0x58

void rda5807_init(TIM_TypeDef *timer);
void rda5807_set_frequency(u16 new_frequency);
//void rda5807_print_rds(void);
void rda5807_set_mute(bool value);
bool rda5807_toggle_mute();
void rda5807_set_bass_boost(bool value);
bool rda5807_toggle_bass_boost();
void rda5807_set_volume(u8 new_volume);

#endif // __RDA5807_H__
