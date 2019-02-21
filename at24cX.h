#pragma once

#include <memory.h>
#include <stdlib.h>

#include <common_lib/i2c.h>
#include <common_lib/utils.h>

#define AT24CX_ADDRESS 0x50

/**
 * Initialize structures used by device
 */
i2c_device *
at24cX_init_address(uint32_t i2c, uint8_t address);

/**
 * Initialize structures used by device, assume default address
 */
i2c_device *
at24cX_init(uint32_t i2c);

/**
 * Read `bytes` bytes from device starting from `address`. Note - if `address`
 * plus `bytes` exceeds size of chip, read will wrap and start from beginning
 * (as described in datasheet). There is no limit of amount of received data,
 * but it's user role to call free() on received buffer.
 */
uint8_t *
at24cX_random_read_bytes(const i2c_device *dev, uint16_t address,
                         uint16_t bytes);

/**
 * Wrapper on function above, read only one byte.
 */
uint8_t
at24cX_random_read_byte(const i2c_device *dev, uint16_t address);

/**
 * Not sure if it's legal but sets internal address for current address
 * read operations
 */
void
at24cX_set_address(const i2c_device *dev, uint16_t address);

/**
 * AT24CX chip keeps address of last read / written byte incremented by 1.
 * This function will read this byte (which will increment counter by 1).
 * Address will roll over if end of last page is reached to first byte on
 * first page.
 * Refer to "Current Address Read" section in datasheet for more details
 */
uint8_t *
at24cX_current_address_read_bytes(const i2c_device *dev, uint16_t bytes);

/**
 * Wrapper on function above, read only byte
 */
uint8_t
at24cX_current_address_read_byte(const i2c_device *dev);

/**
 * Perform page write. After sending `address`, write data from `tx` to device.
 * If page boundary is reached, a write operation will wrap and start from
 * beginning of the *same page*. If `bytes` is greater than page size,
 * this operation will "roll over" and previously transmitted data will be
 * overwritten (so there is no point of sending more bytes than page size
 * on device).
 * Check datasheet "Page Write" chapter for more details.
 * Note: this function will block until `t_wr` is reached (10 ms)
 */
void
at24cX_page_write_bytes(const i2c_device *dev, uint16_t address,
                        const uint8_t *tx, uint8_t bytes);

/**
 * A wrapper for function above, write only one byte
 */
void
at24cX_write_byte(const i2c_device *dev, uint16_t address, uint8_t byte);
