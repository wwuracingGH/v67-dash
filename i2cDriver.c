#include "i2cDriver.h"
#include <stdint.h>
#ifndef STM32F042x6
#define STM32F042x6
#endif
#include "vendor/CMSIS/Device/ST/STM32F0/Include/stm32f0xx.h"

void I2C_Enable(){
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
    RCC->CFGR3 |= RCC_CFGR3_I2C1SW_SYSCLK; /* set the clock to use the system clock - 48mhz */

    // Make sure I2C disabled.
    I2C1->CR1 &= ~I2C_CR1_PE_Msk;
    
    // Ensure Analog Filter is On and Digital Filter is off.
    I2C1->CR1 &= ~I2C_CR1_ANFOFF_Msk;
    I2C1->CR1 &= ~I2C_CR1_DNF_Msk;

    I2C1->TIMINGR |= 0xB << I2C_TIMINGR_PRESC_Pos | 0x13 << I2C_TIMINGR_SCLL | 0xF << I2C_TIMINGR_SCLH
                  | 0x4 << I2C_TIMINGR_SCLDEL_Pos | 0x2 << I2C_TIMINGR_SDADEL_Pos;
    
    // We don't set NOSTRETCH because we are the master.
    
    // Enable I2C.
    I2C1->CR1 &= I2C_CR1_PE_Msk;
}

void I2C_Send(uint16_t addr, uint8_t transfer_direction, uint8_t num_bytes, uint8_t* bytes) {
    I2C1->CR2 &= I2C_CR2_ADD10_Msk; /* Set 7-bit ADDR mode */

    /* Set SADDR to device we're addressing to */
    I2C1->CR2 &= ~I2C_CR2_SADD_Msk;
    I2C1->CR2 &= addr << I2C_CR2_SADD_Pos;

    /* Transfer direction */
    I2C1->CR2 &= ~I2C_CR2_RD_WRN_Msk;
    I2C1->CR2 &= transfer_direction << I2C_CR2_RD_WRN_Pos;
    
    /* Number of bytes */
    I2C1->CR2 &= ~I2C_CR2_NBYTES_Msk;
    I2C1->CR2 &= num_bytes << I2C_CR2_NACK_Pos | I2C_CR2_AUTOEND;

    while(!I2C1->ISR & I2C_ISR_TXE);
    I2C1->TXDR = bytes[0];
    
    /* START bit */
    I2C1->CR2 |= I2C_CR2_START_Msk;

    /* transfer bytes */
    for(int i = 1; i < num_bytes; i++){
        while(!I2C1->ISR & I2C_ISR_TXE);
        I2C1->TXDR = bytes[i];
    } 
}
