/**
 * NICOLE SWIERSTRA
 * 
 * VIKING 67 DASH CODE
 *
 *
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
uint32_t battery_percentage;
uint32_t start_time;


/*
// =============
current_time = os_time - start_time;

apply_timecode(current_time);

//==== recieve can =======

DASH_Command cmd;
start_time -= current_time - cmd.timecode_update;
*/

void clock_init();
void GPIO_Init();
void CAN_Init();
void my_func();

void send_CAN(uint16_t, uint8_t, uint8_t*);
void process_CAN(uint16_t id, uint8_t length, uint64_t data);
void recieve_CAN();

#define SEG_A (1 << 0)
#define SEG_B (1 << 1)
#define SEG_C (1 << 2)
#define SEG_D (1 << 7)
#define SEG_E (1 << 3)
#define SEG_F (1 << 5)
#define SEG_G (1 << 6)
#define SEG_P (1 << 4)

uint8_t buffer[] = {
    0x0,
    0x0,
    0x0,
    0,
    0,
    0,
    0,
    0
};

uint8_t number_lut[] = {
    SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F, /* 0 */
    SEG_B | SEG_C, /* 1 */
    SEG_A | SEG_B | SEG_G | SEG_E | SEG_D, /* 2 */
    SEG_A | SEG_B | SEG_G | SEG_C | SEG_D, /* 3 */
    SEG_F | SEG_G | SEG_B | SEG_C, /* 4 */
    SEG_A | SEG_F | SEG_G | SEG_C | SEG_D, /* 5 */
    SEG_A | SEG_F | SEG_E | SEG_D | SEG_C | SEG_G, /* 6 */
    SEG_A | SEG_B | SEG_C, /* 7 */
    SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F | SEG_G, /* 8 */
    SEG_A | SEG_B | SEG_C | SEG_D | SEG_G | SEG_F, /* 9 */
};

/* take in seconds, and return a uint8_t array of values for the buffer */
void apply_timecode(uint32_t bsd_sec) {
    /* 0000 0000 0000 0000 */
	/* Dsec  sec dsec csec */
	//int csec = (bsd_sec) & 0b1111;
	int dsec = (bsd_sec >> 8) & 0b1111;
	int sec = (bsd_sec >> 12) & 0b1111;
	int Dsec = (bsd_sec >> 16) & 0b1111;
    int hundSec = (bsd_sec >> 20) & 0b1111;

	/* buffer: */
	/* 6    5     3   2 */
	buffer[6] = number_lut[hundSec];
	buffer[5] = number_lut[Dsec];
    buffer[4] = 0b00000000; /* blank */
	buffer[3] = number_lut[sec]|SEG_P;
	buffer[2] = number_lut[dsec];
    

}
void apply_battery(uint32_t bsd_batt) {
    int ones_batt = (bsd_batt) & 0b1111;
	int tens_batt = (bsd_batt >> 4) & 0b1111;

    buffer[1] = number_lut[tens_batt];
    buffer[0] = number_lut[ones_batt];
}

void ping() {
    uint8_t* hw = "helo wld";
    send_CAN(1, 8, hw);
}

int main(){
    /* setup */
    clock_init();
    GPIO_Init();
    RTOS_init();
    CAN_Init();
    GPIOA->ODR ^= GPIO_ODR_10;

    RTOS_start_armeabi(48000000);
    __enable_irq(); /* enable interrupts */
    
    int default_state = RTOS_addState(0, 0);
    RTOS_switchState(default_state);
    RTOS_scheduleTask(default_state, my_func, 1);
    RTOS_scheduleTask(default_state, recieve_CAN, 1);
    RTOS_scheduleTask(default_state, ping, 100);

    /* non rt program bits */
    for(;;){
        RTOS_ExecuteTasks();
    }
}


uint32_t dubdabble (uint32_t poodle){
    uint32_t output = 0;

    for (int i = 0; i < 32; i++){
        for (int i = 0; i < 8; i++) { /* we need to check the 5 lowest digits */
            if ((output >> (4 * i) & 0xF) >= 5) { /* we check the ith digit is greater than or equal to 5 */
                output += 3 << (4 * i); /* we add 3 to that digit if it is */
            }
        }

        /* this is what shifting something in looks like */
        output <<= 1; /* shift the output by one */
        output |= poodle >> 31; /* adds the top bit of poodle to the end of output */
        poodle <<= 1; /* shift the input by one */
    }

    return output;
}

void my_func(){
    uint32_t ms = RTOS_getMainTick();
    apply_timecode(dubdabble(ms));
    apply_battery(dubdabble(battery_percentage));
    

    static uint8_t i = 0;

    if (i <= 7) {
        GPIOA->ODR = ~(1 << i);
        GPIOB->ODR = (buffer[i] & 0b11 | (buffer[i] & 0b11111100) << 1);
    }

    i++;
    if(i > 8) i = 0;
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
    RCC->AHBENR |= RCC_AHBENR_GPIOFEN; 

    GPIOA->MODER |= (MODE_OUTPUT << GPIO_MODER_MODER10_Pos);
    /* set pin mode for output here - put pin into output mode - led pin is port A 10 */
    GPIOA->MODER |= 0x5555
            |  (MODE_ALTFUNC   << GPIO_MODER_MODER11_Pos)  /* CAN TX       */
            |  (MODE_ALTFUNC   << GPIO_MODER_MODER12_Pos); /* CAN RX       */
    GPIOA->OTYPER = 0xFF;
    GPIOA->PUPDR = 0x5555;
    GPIOB->MODER |= 0x15555;
    GPIOF->MODER |= 1;

    GPIOA->AFR[1] &= ~((0xF << GPIO_AFRH_AFSEL11_Pos) | (0xF << GPIO_AFRH_AFSEL12_Pos));
    GPIOA->AFR[1] |= (4 << GPIO_AFRH_AFSEL11_Pos) | (4 << GPIO_AFRH_AFSEL12_Pos); /* can AFR */
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
        case DL_CANID_DASH_COMMAND:
            DASH_Command dc = *(DASH_Command*)&data;
            battery_percentage = dc.battery_percentage;
        break;
    }
}
