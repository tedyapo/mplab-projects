/*
 * File:   main.c
 * Author: tyapo
 *
 * Created on August 9, 2019, 12:36 PM
 */

// ATSAMD21E18A Configuration Bit Settings

// 'C' source line config statements

// Config Source code for XC32 compiler.
// USER_WORD_0
#pragma config NVMCTRL_BOOTPROT = SIZE_0BYTES
#pragma config NVMCTRL_EEPROM_SIZE = SIZE_0BYTES
#pragma config BOD33USERLEVEL = 0x7 // Enter Hexadecimal value
#pragma config BOD33_EN = ENABLED
#pragma config BOD33_ACTION = RESET
#pragma config WDT_ENABLE = DISABLED
#pragma config WDT_ALWAYSON = DISABLED
#pragma config WDT_PER = CYC16384
#pragma config WDT_WINDOW_0 = SET

// USER_WORD_1
#pragma config WDT_WINDOW_1 = 0x5 // Enter Hexadecimal value
#pragma config WDT_EWOFFSET = CYC16384
#pragma config WDT_WEN = DISABLED
#pragma config BOD33_HYST = DISABLED
#pragma config NVMCTRL_REGION_LOCKS = 0xFFFF // Enter Hexadecimal value


#include <sam.h>

void delay()
{
    for(volatile uint32_t i=0; i<10000; ++i);
}

void main(void) {  
    PORT_REGS->GROUP[0].PORT_DIRSET = PORT_PA00;
    PORT_REGS->GROUP[0].PORT_DIRSET = PORT_PA28;
    PORT_REGS->GROUP[0].PORT_OUTSET = PORT_PA00;
    PORT_REGS->GROUP[0].PORT_OUTCLR = PORT_PA28;
    while(1){
      //        PORT_REGS->GROUP[0].PORT_OUTTGL = PORT_PA00;
	PORT_REGS->GROUP[0].PORT_OUTTGL = PORT_PA28;
        delay();
    }
    return;
}
