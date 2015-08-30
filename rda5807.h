#ifndef __RDA5807_H__
#define __RDA5807_H__
#include "i2c_dma.h"
#include "stdio.h"

#define RDA5970_ADDRESS 0x11
#define RDA5970_ID 0x58

void rda5807_init();
void rda5807_set_frequency(u16 new_frequency);
void rda5807_print_rds(void);

#endif // __RDA5807_H__
