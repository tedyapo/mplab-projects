/*
 * File:   main.c
 * Author: tyapo
 *
 * Created on June 19, 2020, 11:01 AM
 */

// PIC12LF1572 Configuration Bit Settings
// CONFIG1
#pragma config FOSC = INTOSC    //  (INTOSC oscillator; I/O function on CLKIN pin)
#pragma config WDTE = ON        // Watchdog Timer Enable (WDT enabled)
#pragma config PWRTE = ON       // Power-up Timer Enable (PWRT enabled)
#pragma config MCLRE = OFF      // MCLR Pin Function Select (MCLR/VPP pin function is digital input)
#pragma config CP = ON          // Flash Program Memory Code Protection (Program memory code protection is enabled)
#pragma config BOREN = OFF      // Brown-out Reset Enable (Brown-out Reset disabled)
#pragma config CLKOUTEN = OFF    // Clock Out Enable (CLKOUT function is enabled on the CLKOUT pin)

// CONFIG2
#pragma config WRT = ALL        // Flash Memory Self-Write Protection (000h to 7FFh write protected, no addresses may be modified by EECON control)
#pragma config PLLEN = OFF      // PLL Enable (4x PLL disabled)
#pragma config STVREN = OFF     // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will not cause a Reset)
#pragma config BORV = HI        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), high trip point selected.)
#pragma config LPBOREN = OFF     // Low Power Brown-out Reset enable bit (LPBOR is disabled)
#pragma config LVP = OFF        // Low-Voltage Programming Enable (High-voltage on MCLR/VPP must be used for programming)

#include <xc.h>
#include <stdint.h>

// NB: RA4 is switched to analog input for reading resistor ratio
void init()
{
  // PORTA
  ANSELA     = 0b00000000; // all lines digital
  LATA       = 0b00000000; // all outputs low
  TRISA      = 0b00000000; // all RAx outputs
  INLVLA     = 0;          // TTL input levels on all
  SLRCONA    = 0b11011111; // high speed on RA5
  nWPUEN     = 0;          // enable weak pull-ups
  WPUA       = 0b00001000; // pull-ups on RA3

  // PWM1
  PWM1INTE   = 0b00000001; // interrupt on PWM period match
  PWM1CLKCON = 0b00000010; // run from LFINTOSC, no prescale --> 31 kHz
  PWM1PH     = 1;
  PWM1DC     = 2;
  PWM1PR     = 517;  // 60 Hz flash rate
  //PWM1PR     = 52;  // 600 Hz flash rate  
  PWMLD      = 0b00000001;
  PWM1CON    = 0b10000000; // enable module; disable output pin

  // enable peripheral interrupts (to wake from sleep with PWM)
  PEIE       = 1;
  PWM1IE     = 1;
  GIE        = 0;

  // WDT set to 128 ms period
  WDTCON     = 0b0001111;
}

uint16_t resistor_ratio()
{
  // configure RA4/AN3 as analog input
  TRISA4 = 1;
  ANSA4 = 1;
  // set RA2 high to power volatge divider
  LATA2 = 1;
  // clock from Fosc/2, left-justify result, Vdd as ref
  ADCON1 = 0b00000000;
  // select channel AN3, enable ADC
  ADCON0 = 0b00001101;
  NOP(); // acquisition time
  nDONE = 1; // start conversion
  while(nDONE){ /* spin */ }
  ADCON0 = 0;
  LATA2 = 0;
  TRISA4 = 0;
  ANSA4 = 0;
  return ADRES;
}

// returns battery voltage in mV
uint16_t battery_voltage()
{
  FVRCON = 0b10000001; // enable FVR at 1.024V
  ADCON0 = 0b01111101; // enable ADC and set FVR as input channel
  ADCON1 = 0b10000000; // Vref+ is Vdd; Fosc/2 clock, right just
  while(!FVRCONbits.FVRRDY){ /* spin */ } // wait for FVR stable
  NOP(); // 8 us > 5 us min acquisition time
  ADCON0bits.GO_nDONE = 1; // start ADC conversion
  while(ADCON0bits.GO_nDONE){ /* spin */ } // wait for conversion complete
  FVRCON = 0; // turn off FVR  
  ADCON0bits.ADON = 0; // turn off ADC
  // note: ADRES = 1023 * 1.024 / Vdd
  //       ==> Vdd = (1023 * 1.024) / ADRES
  //       ==> Vdd = 1047.552 / ADRES
  //       ==> Vdd (mV) = 1047552 / ADRES
  return 1047552L / ADRES;
}


uint16_t reset_count;  
extern void main_loop(void);

void main(void) {
  init();

  // osctune varies from -32 at 2.8V to +31 at 3.6 V
  int16_t vbatt = battery_voltage();
  if (vbatt > 3600) vbatt = 3600;
  if (vbatt < 2800) vbatt = 2800;
  vbatt -= 2800; // 0-800
  OSCTUNE = -32 + (63 * vbatt + 400) / 800;

  // measure brightness-setting resistors
  uint32_t ratio = resistor_ratio();
  
  // set PWM rate for between 1-10 year run rate based on resistor ratio
  PWM1PR = 520 - ratio*479 / 65536;
  PWMLD      = 0b00000001;  

  // how many flashes before each reset/re-initialization
  reset_count = 65535 - (7610 - ratio * 7016 / 65536);
  
  main_loop(); // never returns  
  return;
}
