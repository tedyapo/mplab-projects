/*
 * File:   main.c
 * Author: tyapo
 *
 * Created on May 24, 2020, 1:45 PM
 */

// CONFIG1
#pragma config FOSC = INTOSC    // Oscillator Selection Bits (INTOSC oscillator: I/O function on CLKIN pin)
#pragma config WDTE = OFF       // Watchdog Timer Enable (WDT disabled)
#pragma config PWRTE = ON       // Power-up Timer Enable (PWRT enabled)
#pragma config MCLRE = ON       // MCLR Pin Function Select (MCLR/VPP pin function is MCLR)
#pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config BOREN = OFF      // Brown-out Reset Enable (Brown-out Reset disabled)
#pragma config CLKOUTEN = ON    // Clock Out Enable (CLKOUT function is enabled on the CLKOUT pin)
#pragma config IESO = ON        // Internal/External Switchover Mode (Internal/External Switchover Mode is enabled)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is enabled)

// CONFIG2
#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
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
              STATE_DONE} rx_state_t; // command received; waiting to be processed

// NEC IR code and associated rx state
typedef struct {
  uint8_t n_bits;
  rx_state_t state;
  union {
    uint32_t code;
    struct {
      // note: the order here is compiler implementation-defined
      //       this struct is not portable between compilers
      uint8_t command_b;            
      uint8_t command;
      uint8_t address_b;            
      uint8_t address;
    };
  };
} NEC_IR_code_t;

static NEC_IR_code_t ir_code = {0, STATE_RESET};

//static rx_state_t rx_state = STATE_RESET;
//static uint32_t received_code;
//static uint8_t n_bits;

// Note: timings based on 4 MHz fosc with timer0 prescale=64

// preamble
// measured 216-218 ticks
#define PREAMBLE_MIN 195
#define PREAMBLE_MAX 239

// logic 0
// measured 18 ticks
#define ZERO_MIN 16
#define ZERO_MAX 20

// logic 1
// measured 35 ticks
#define ONE_MIN 32
#define ONE_MAX 39

uint8_t stats[33];

void __interrupt() ISR(void)
{
  uint8_t time = TMR0;
  TMR0 = 0;

  // if timer0 overflowed, pulse too long --> reset state
  if (INTCONbits.TMR0IF){
    time = 0; // invalid time will cause state reset below
  }
  INTCONbits.TMR0IF = 0;
  
  switch(ir_code.state){
  case STATE_RESET:
    // check for valid preamble time
    if (time >= PREAMBLE_MIN && time <= PREAMBLE_MAX){
      ir_code.n_bits = 0;
      ir_code.state = STATE_RECEIVING;
      stats[0] = time;
    }
    break;
  case STATE_RECEIVING:
    // shift the previous bits and check for valid new bit
    ir_code.code <<= 1;
    if (time >= ONE_MIN && time <= ONE_MAX){
      stats[1+ir_code.n_bits] = time;
      ir_code.code |= 1;
      ir_code.n_bits++;
    } else if (time >= ZERO_MIN && time <= ZERO_MAX){
      stats[1+ir_code.n_bits] = time;
      ir_code.n_bits++;      
    } else {
      // not a valid bit
      ir_code.state = STATE_RESET;
      break;
    }
    if (32 == ir_code.n_bits){
      // check for valid code
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
void putch(char value)
{
  // wait for room in the FIFO
  while(!PIR1bits.TXIF){ /* spin */ }
  TXREG = value;
  __asm("NOP");
}

// send a byte as ASCII-formatted binary over the serial port
// e.g. 10110110
void send_binary_byte(uint8_t value)
{
  for (uint8_t i=0; i<8; i++){
    putch((value & (1 << (7-i))) ? '1' : '0');
  }
}

// send a byte as ASCII-formatted hex over the serial port
// e.g. 1F
void send_hex_byte(uint8_t value)
{
  char hex_table[] = "0123456789ABCDEF";
  putch( hex_table[((value >> 4) & 0xf)]);
  putch( hex_table[((value >> 0) & 0xf)]);
}


static uint8_t DAC_value = 0;

void process_remote_code(NEC_IR_code_t* code){
  switch(code->command){
  case 0x07: // '-' button
    if (DAC_value > 0){
      DAC_value--;
    }
    break;
  case 0x6f: // '+' button
    if (DAC_value < 31){
      DAC_value++;
    }
    break;
  default:
    break;
  }
  DACCON1bits.DACR = DAC_value;
}

// NB: RA2 is int
// NB: RB7 is TX

void main(void) {
  OSCCONbits.SCS = 0b10; // use internal oscillator 
  OSCCONbits.IRCF = 0b1101; // 4 MHz

  ANSELA = 0;
  TRISA = 0b00000100; // RA2/INT is input
  WPUA = 0;
  
  ANSELB = 0; // PORTB all digital    
  TRISB = 0; // all outputs
  WPUB = 0;

  ANSELC = 0;
  TRISC = 0; // PORTC all outputs
    
  OPTION_REGbits.PSA = 0; // assign prescaler to timer0
  OPTION_REGbits.PS = 0b101; // prescale by 64
  OPTION_REGbits.T0CS = 0; // timer0 clocked from Fosc/4
    
  INTCONbits.GIE = 1; // enable interrupts
  OPTION_REGbits.INTEDG = 0; // interrupt on falling edge
  INTCONbits.INTE = 1; // enable INT pin interrupts

  // init the AUSART
  TXSTAbits.TXEN = 1;
  TXSTAbits.SYNC = 0;
  RCSTAbits.SPEN = 1;

  // 9600 baud for 4 MHz clock
  TXSTAbits.BRGH = 1;
  BAUDCONbits.BRG16 = 0;
  SPBRG = 25;

  // init DAC for remote control test
  //DACCON0bits.DACPSS = 0;
  //DACCON0bits.DACEN = 1;
  //DACCON0bits.DACOE1 = 1;

  while(1){
    // poll for received code
    if (STATE_DONE == ir_code.state){
      // a code was received, send it out the serial port
      /*
        send_hex_byte(ir_code.address);
        send_hex_byte(ir_code.address_b);
        send_hex_byte(ir_code.command);
        send_hex_byte(ir_code.command_b);
      */
      for(int i=0; i<33; i++){
        printf("%d %d\n", i, (int)stats[i]);
      }
      // process_remote_code(ir_code);
      // listen for next code
      ir_code.state = STATE_RESET;
    }
  }
    
  return;
}
