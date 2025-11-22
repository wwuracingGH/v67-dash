#include "i2cDriver.h"
#include <stdint.h>
#include "stm32f042xx.h"

void I2C_Enable(){
    // Make sure I2C disabled.
    I2C->CR1 &= ~I2C_CR1_PE_Msk;
    
    // Ensure Analog Filter is On and Digital Filter is off.
    I2C->CR1 &= ~I2C_CR1_ANFOFF_Msk;
    I2C->CR1 &= ~I2C_CR1_DNF_Msk;

    // Configure Timing (because Nicole loves doing math).
    
    // We don't set NOSTRETCH because we are the master.
    
    // Enable I2C.
    I2C->CR1 &= I2C_CR1_PE_Msk;
}

void I2C_Init(uint16_t addr, uint8_t transfer_direction, uint8_t num_bytes) {
    I2C->CR2 &= I2C_CR2_ADD10_Msk; //Set 7-bit ADDR mode

    // Set SADDR to device we're addressing to
    I2C->CR2 &= ~I2C_CR2_SADD_Msk;
    I2C->CR2 &= addr << I2C_CR2_SADD_Pos;
    // Transfer direction
    I2C->CR2 &= ~I2C_CR2_RD_WRN_Msk;
    I2C->CR2 &= transfer_direction << I2C_CR2_RD_WRN_Pos;
    
    // Number of bytes
    I2C->CR2 &= ~I2C_CR2_NBYTES_Msk;
    I2C->CR2 &= num_bytes << I2C_CR2_NACK_Pos;

    // START bit
    I2C->CR2 &= I2C_CR2_START_Msk;    
}
