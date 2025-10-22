/**
 * NICOLE SWIERSTRA
 * 
 * VIKING 66 VCU CODE
 *
 * This is the code for the vcu of the wwu racing viking 66 car meant for competition in 2024
 *
 * documentation can be found somewhere in the q drive and maybe in the readme if I ever get to it.
 */

#include <stdint.h>
#ifndef STM32F042x6
#define STM32F042x6
#endif
#include "vendor/CMSIS/Device/ST/STM32F0/Include/stm32f0xx.h"
#include "canDefinitions.h"
#include "vendor/qfplib/qfplib.h"

#define RTOS_maxTaskNum 16
#define RTOS_maxEventNum 8
#define RTOS_maxStateNum 8
#include "llrttsos.h"
_RTOS_IMPLEMENTATION_

struct {
} car_state;

typedef struct _canmsg{
    volatile uint32_t id;
    volatile uint32_t len;
    volatile uint64_t data;
} CAN_msg;

enum Pin_Mode {
    MODE_INPUT   = 0b00,
    MODE_OUTPUT  = 0b01,
    MODE_ALTFUNC = 0b10,
    MODE_ANALOG  = 0b11
};

void clock_init();
void GPIO_Init();
void CAN_Init();
void flash_Init();

void send_CAN(uint16_t, uint8_t, uint8_t*);
void process_CAN(uint16_t id, uint8_t length, uint64_t data);
void recieve_CAN();

int main(){
    /* setup */
    clock_init();
    flash_Init();
    GPIO_Init(); /* must be called first */
    RTOS_init();
    CAN_Init();

    RTOS_start_armeabi(48000000);
    __enable_irq(); /* enable interrupts */

    /* non rt program bits */
    for(;;){
        RTOS_ExecuteTasks();
    }
}

/* runs every 1 ms */
void systick_handler(){  
    RTOS_Update();
}

void flash_Init(){
    while ((FLASH->SR & FLASH_SR_BSY) != 0);

    if ((FLASH->CR & FLASH_CR_LOCK) != 0) {
        FLASH->KEYR = (uint32_t)0x45670123;
        FLASH->KEYR = (uint32_t)0xCDEF89AB;
    }
}

void clock_init() /* turns on hsi48 and sets as system clock */
{
    /* wait one clock cycle before accessing flash memory @48MHZ */
    FLASH->ACR |= 0b001 << FLASH_ACR_LATENCY_Pos;

    /* Enables HSI48 oscillator */
    RCC->CR2  |= RCC_CR2_HSI48ON;
    while (!(RCC->CR2 & RCC_CR2_HSI48RDY));
    
    /* no peripheral prescaler div or hsi prescaler div */
    RCC->CFGR &= ~(0b111 << RCC_CFGR_PPRE_Pos);
    RCC->CFGR &= ~(0b1111 << RCC_CFGR_HPRE_Pos);

    /* sets system clock as HSI48 oscillator */
    RCC->CFGR |= 0b11 << RCC_CFGR_SW_Pos;
    while (!(RCC->CFGR & (0b11 << RCC_CFGR_SWS_Pos)));
}

void CAN_Init (){
    RCC->APB1ENR |= RCC_APB1ENR_CANEN;
    CAN->MCR |= CAN_MCR_INRQ; /* goes from normal mode into initialization mode */
    
    while (!(CAN->MSR & CAN_MSR_INAK));
   
    /* wakes it up */
    CAN->MCR &= ~CAN_MCR_SLEEP;
    while (CAN->MSR & CAN_MSR_SLAK);

    /* set bittiming - just read wikipedia if you don't know what that is */
    /* TODO: why is it still 1/2 of what it should be */
    CAN->BTR |= 23 << CAN_BTR_BRP_Pos | 1 << CAN_BTR_TS1_Pos | 0 << CAN_BTR_TS2_Pos;
    CAN->MCR &= ~CAN_MCR_INRQ; /* clears the initialization request and starts the actual can */
    
    while (CAN->MSR & CAN_MSR_INAK);

    /* blank filter - tells the can to read every message */
    CAN->FMR |= CAN_FMR_FINIT; 
    CAN->FA1R |= CAN_FA1R_FACT0;
    CAN->sFilterRegister[0].FR1 = 0; /* Its like a filter, but doesn't filter anything! */
    CAN->sFilterRegister[0].FR2 = 0;
    CAN->FMR &=~ CAN_FMR_FINIT;
    CAN->IER |= CAN_IER_FMPIE0;
}
    
void GPIO_Init(){
    /* turn on gpio clocks */
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN; 
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN; 
}

void send_CAN(uint16_t id, uint8_t length, uint8_t* data){
    /* all mailboxes full */
    while(!(CAN->TSR & CAN_TSR_TME_Msk));

    /* find first empty mailbox */
    int j = (CAN->TSR & CAN_TSR_CODE_Msk) >> CAN_TSR_CODE_Pos;
    
    /* set dlc to length */
    CAN->sTxMailBox[j].TDTR = length;

    /* clears data high/low registers */
    CAN->sTxMailBox[j].TDLR = 0;
    CAN->sTxMailBox[j].TDHR = 0;
    
    /* writes to high/low registers */
    for(int i = 0; i < length && i < 4; i++) 
        CAN->sTxMailBox[j].TDLR |= ((data[i] & 0xFF) << i * 8);
    for(int i = 0; i < length - 4; i++)
        CAN->sTxMailBox[j].TDHR |= ((data[i+4] & 0xFF) << i * 8);
   
    /* writes id and queues message */
    CAN->sTxMailBox[j].TIR = (uint32_t)((id << CAN_TI0R_STID_Pos) | CAN_TI0R_TXRQ);
}

void recieve_CAN(){
    /* while mailboxes aren't empty */
    while ((CAN->RF0R & CAN_RF0R_FMP0) != 0) { 
        uint8_t  can_len    = CAN->sFIFOMailBox[0].RDTR & 0xF;
        uint64_t can_data   = CAN->sFIFOMailBox[0].RDLR 
            + ((uint64_t)CAN->sFIFOMailBox[0].RDHR << 32);
        uint16_t can_id     = CAN->sFIFOMailBox[0].RIR >> CAN_RI0R_STID_Pos;
        CAN->RF0R |= CAN_RF0R_RFOM0; /* release mailbox */

        process_CAN(can_id, can_len, can_data);
    }
}

void process_CAN(uint16_t id, uint8_t length, uint64_t data){
    (void)(length);

    switch (id){
        case 0:
        break;
    }
}
