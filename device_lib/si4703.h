#pragma once

#include <common_lib/i2c.h>
#include <common_lib/usart.h>
#include <common_lib/utils.h>
#include <libopencm3/stm32/i2c.h>

#define SI4703_DEVICEID_REG 0x00
#define SI4703_CHIPID_REG 0x01
#define SI4703_POWERCFG_REG 0x02
#define SI4703_CHANNEL_REG 0x03
#define SI4703_SYSCONFIG1_REG 0x04
#define SI4703_SYSCONFIG2_REG 0x05
#define SI4703_SYSCONFIG3_REG 0x06
#define SI4703_TEST1_REG 0x07
#define SI4703_TEST2_REG 0x08
#define SI4703_BOOTCONFIG_REG 0x09
#define SI4703_STATUSRSSI_REG 0x0A
#define SI4703_READCHAN_REG 0x0B
#define SI4703_RDSA_REG 0x0C
#define SI4703_RDSB_REG 0x0D
#define SI4703_RDSC_REG 0x0E
#define SI4703_RDSD_REG 0x0F

#define SI4703_POWERCFG_DSMUTE (1u << 15u)
#define SI4703_POWERCFG_DMUTE (1u << 14u)
#define SI4703_POWERCFG_MONO (1u << 13u)
#define SI4703_POWERCFG_RDSM (1u << 11u)
#define SI4703_POWERCFG_SKMODE (1u << 10u)
#define SI4703_POWERCFG_SEEKUP (1u << 9u)
#define SI4703_POWERCFG_SEEK (1u << 8u)
#define SI4703_POWERCFG_DISABLE (1u << 6u)
#define SI4703_POWERCFG_ENABLE (1u << 0u)

#define SI4703_CHANNEL_TUNE (1u << 15u)

#define SI4703_SYSCONFIG1_RDSIEN (1u << 15u)
#define SI4703_SYSCONFIG1_STCIEN (1u << 14u)
#define SI4703_SYSCONFIG1_RDS (1u << 12u)
#define SI4703_SYSCONFIG1_DE (1u << 11u)
#define SI4703_SYSCONFIG1_AGCD (1u << 10u)

typedef struct {
  union {
    uint16_t raw;
    struct __attribute__((packed)) {
      uint16_t volume : 4;
      uint16_t space : 2;
      uint16_t band : 2;
      uint16_t seekth : 8;
    };
  };
} si4703_sysconfig2_reg;

typedef struct {
  union {
    uint16_t raw;
    struct __attribute__((packed)) {
      uint16_t skcnt : 4;
      uint16_t sksnr : 4;
      uint16_t volext : 1;
      uint16_t : 3;
      uint16_t smutea : 2;
      uint16_t smuter : 2;
    };
  };
} si4703_sysconfig3_reg;

#define SI4703_TEST1_XOSCEN 1u << 15u

#define SI4703_STATUSRSSI_RDSR 1u << 15u
#define SI4703_STATUSRSSI_STC 1u << 14u
#define SI4703_STATUSRSSI_SF_BL 1u << 13u
#define SI4703_STATUSRSSI_AFCRL 1u << 12u
#define SI4703_STATUSRSSI_RDSS 1u << 11u
#define SI4703_STATUSRSSI_BLERA 1u << 10u
#define SI4703_STATUSRSSI_ST 1u << 8u

typedef struct {
  union {
    uint16_t raw;
    struct __attribute__((packed)) {
      uint16_t readchan : 10;
      uint16_t blerd : 2;
      uint16_t blerc : 2;
      uint16_t blerb : 2;
    };
  };
} si4703_readchan_reg;

typedef struct {
  union {
    uint16_t raw;
    struct __attribute__((packed)) {
      uint16_t mfgid : 12;
      uint16_t pn : 4;
    };
  };
} si4703_device_id;

typedef struct {
  union {
    uint16_t raw;
    struct __attribute__((packed)) {
      uint16_t firmware : 6;
      uint16_t dev : 4;
      uint16_t rev : 6;
    };
  };
} si4703_chip_id;

typedef struct {
  i2c_device i2c;
  uint16_t memory[16];
  struct {
    uint16_t programme_identification;
    uint8_t pty;
  } rds;
} si4703_device;

error_t si4703_init(si4703_device *device, uint32_t i2c, const pin *rst,
                    const pin *sen, uint32_t timer);
error_t si4703_set_channel(si4703_device *device, uint16_t frequency);
