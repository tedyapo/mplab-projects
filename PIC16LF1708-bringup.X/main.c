/*
 * File:   main.c
 * Author: tyapo
 *
 * Created on May 31, 2020, 7:52 PM
 */


// PIC16LF1708 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1
#pragma config FOSC = INTOSC    // Oscillator Selection Bits (INTOSC oscillator: I/O function on CLKIN pin)
#pragma config WDTE = OFF       // Watchdog Timer Enable (WDT disabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable (PWRT disabled)
#pragma config MCLRE = ON       // MCLR Pin Function Select (MCLR/VPP pin function is MCLR)
#pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config BOREN = OFF      // Brown-out Reset Enable (Brown-out Reset disabled)
#pragma config CLKOUTEN = ON    // Clock Out Enable (CLKOUT function is enabled on the CLKOUT pin)
#pragma config IESO = OFF       // Internal/External Switchover Mode (Internal/External Switchover Mode is disabled)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is enabled)

// CONFIG2
#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
#pragma config PPS1WAY = ON     // Peripheral Pin Select one-way control (The PPSLOCK bit cannot be cleared once it is set by software)
#pragma config ZCDDIS = ON      // Zero-cross detect disable (Zero-cross detect circuit is disabled at POR)
#pragma config PLLEN = OFF      // Phase Lock Loop enable (4x PLL is enabled when software sets the SPLLEN bit)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
#pragma config LPBOR = OFF      // Low-Power Brown Out Reset (Low-Power BOR is disabled)
#pragma config LVP = ON         // Low-Voltage Programming Enable (Low-voltage programming enabled)

#include <xc.h>
#include <stdint.h>
#include <stdio.h>

// receiver states
typedef enum {STATE_RESET = 0,        // waiting for preamble sequence
              STATE_RECEIVING,        // receiving bit stream
              STATE_DONE}             // received; waiting to be processed
  rx_state_t;

// NEC IR code and associated rx state
typedef struct {
  uint8_t n_bits;
  rx_state_t state;
  union {
    uint32_t code;
    struct {
      // note: the order here is implementation-defined
      //       this struct is not portable between compilers
      uint8_t command_b;            
      uint8_t command;
      uint8_t address_b;            
      uint8_t address;
    };
  };
} NEC_IR_code_t;

NEC_IR_code_t ir_code = {0, STATE_RESET};


// Note: timings based on 4 MHz fosc with timer0 prescale=64

// preamble
// measured 216-218 ticks
#define PREAMBLE_MIN_TICKS 195
#define PREAMBLE_MAX_TICKS 239

// logic 0
// measured 18 ticks
#define ZERO_MIN_TICKS 16
#define ZERO_MAX_TICKS 20

// logic 1
// measured 35 ticks
#define ONE_MIN_TICKS 32
#define ONE_MAX_TICKS 39

// timing stats for analysis
uint8_t stats[33];

void __interrupt() ISR(void)
{
  uint8_t time = TMR0;
  TMR0 = 0;

  // if timer0 overflowed, pulse too long to be valid --> reset state
  if (INTCONbits.TMR0IF){
    time = 0; // invalid time will cause state reset below
  }
  INTCONbits.TMR0IF = 0;
  
  switch(ir_code.state){
  case STATE_RESET:
    // check for valid preamble time
    if (time >= PREAMBLE_MIN_TICKS && time <= PREAMBLE_MAX_TICKS){
      ir_code.n_bits = 0;
      ir_code.state = STATE_RECEIVING;
      stats[0] = time;
    }
    break;
  case STATE_RECEIVING:
    // shift the previous bits and check for valid new bit
    ir_code.code <<= 1;
    if (time >= ONE_MIN_TICKS && time <= ONE_MAX_TICKS){
      stats[1+ir_code.n_bits] = time;
      ir_code.code |= 1;
      ir_code.n_bits++;
    } else if (time >= ZERO_MIN_TICKS && time <= ZERO_MAX_TICKS){
      stats[1+ir_code.n_bits] = time;
      ir_code.n_bits++;      
    } else {
      // not a valid bit
      ir_code.state = STATE_RESET;
      break;
    }
    if (32 == ir_code.n_bits){
      // full word received; check for valid code
      if ( (ir_code.address == ((~ir_code.address_b) & 0xff)) &&
           (ir_code.command == ((~ir_code.command_b) & 0xff)) ){ 
        ir_code.state = STATE_DONE;
      } else {
        ir_code.state = STATE_RESET;
      }
    }
    break;
  case STATE_DONE:
    // wait for main loop to reset state
    // note: no new codes are received until main loop resets receiver
    break;
  default:
    // any unexpected state gets reset
    ir_code.state = STATE_RESET;
  }
  
  INTCONbits.INTF = 0;
}

// write a character to the serial port
// ptintf() calls this to output characters
void putch(char value)
{
  // wait for room in the FIFO
  while(!PIR1bits.TXIF){ /* spin */ }
  TXREG = value;
  // note: TXIF not valid for two cycles
  __asm("NOP");
}

void process_remote_command(NEC_IR_code_t* code){
  switch(code->command){
  case 0x07: // '-' button
    break;
  case 0x6f: // '+' button
    break;
  default:
    break;
  }
}

void main(void) {
  OSCCONbits.SCS = 0b10; // use internal oscillator   
  OSCCONbits.IRCF = 0b1101; // 4 MHz HF INTOSC

  // PORTA configuration
  ANSELA = 0;         // PORTA all digital
  TRISA = 0b00000100; // RA2/INT is input
  ODCONA = 0;         // no open-drain pins
  SLRCONA = 0xff;     // limited slew rate on all pins
  INLVLA = 0xff;      // schmitt-trigger CMOS inputs
  WPUA = 0;           // no weak pull-ups on PORTA

  // PORTB configuration
  ANSELB = 0;         // PORTB all digital    
  TRISB = 0;          // PORTB all outputs
  ODCONB = 0;         // no open-drain pins
  SLRCONB = 0xff;     // limited slew rate on all pins
  INLVLB = 0xff;      // schmitt-trigger CMOS inputs
  WPUB = 0;           // no weak pull-ups on PORTB

  // PORTC configuration
  ANSELC = 0;         // PORTC all digital    
  TRISC = 0;          // PORTC all outputs
  ODCONC = 0;         // no open-drain pins
  SLRCONC = 0xff;     // limited slew rate on all pins
  INLVLC = 0xff;      // schmitt-trigger CMOS inputs
  WPUC = 0;           // no weak pull-ups on PORTC

  // PPS configuration
  // NB: RA2 is int
  // NB: RB7 is TX
  INTPPS = 0b00010; // INT on RA2 (ds p. 139, 142) 
  RB7PPS = 0b10100; // TX on RB7 (ds p. 140)

  // timer0 configutation
  OPTION_REGbits.PSA = 0; // assign prescaler to timer0
  OPTION_REGbits.PS = 0b101; // prescale by 64
  OPTION_REGbits.T0CS = 0; // timer0 clocked from Fosc/4

  // interrupt configuration
  INTCONbits.GIE = 1; // enable interrupts
  OPTION_REGbits.INTEDG = 0; // interrupt on falling edge
  INTCONbits.INTE = 1; // enable INT pin interrupts

  // init the EUSART
  TXSTAbits.TXEN = 1;
  TXSTAbits.SYNC = 0;
  RCSTAbits.SPEN = 1;

  // 9600 baud for 4 MHz clock
  TXSTAbits.BRGH = 1;
  BAUDCONbits.BRG16 = 0;
  SPBRG = 25;

  while(1){
    // poll for received code
    if (STATE_DONE == ir_code.state){
      
      // a code was received, send it out the serial port
      printf("0x%08lx\n", (unsigned long)ir_code.code);

      // dump timing stats
      //for(int i=0; i<33; i++){
      //  printf("%d %d\n", i, (int)stats[i]);
      //}
      
      process_remote_command(&ir_code);
      
      // listen for next code
      ir_code.state = STATE_RESET;
    }
  }
    
  return;
}
