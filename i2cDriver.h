#ifndef _FLASH_H_
#define _FLASH_H_
#include <stdint.h>

/*
 * Enables I2C peripherial.
 */
void I2C_Enable();

/*
 * addr - I2C address of recieving device
 * transfer_direction - 0 for write transfer, 1 for read transfer
 * num_bytes - Number of bytes to be sent 
 */
void I2C_Init(uint16_t addr, uint8_t transfer_direction, uint8_t num_bytes);
