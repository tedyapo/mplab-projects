/*
 * File:   bringup.c
 * Author: tyapo
 *
 * Created on June 21, 2019, 10:23 PM
 */

// CONFIG1
#pragma config FEXTOSC = ECH    // External Oscillator mode selection bits (EC above 8MHz; PFM set to high power)
#pragma config RSTOSC = EXT1X   // Power-up default value for COSC bits (EXTOSC operating per FEXTOSC bits)
//#pragma config FEXTOSC = ECH    // External Oscillator mode selection bits (EC above 8MHz; PFM set to high power)
//#pragma config RSTOSC = HFINT32 // Power-up default value for COSC bits (HFINTOSC with OSCFRQ= 32 MHz and CDIV = 1:1)
#pragma config CLKOUTEN = OFF   // Clock Out Enable bit (CLKOUT function is disabled; i/o or oscillator function on OSC2)
#pragma config CSWEN = ON       // Clock Switch Enable bit (Writing to NOSC and NDIV is allowed)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable bit (FSCM timer enabled)

// CONFIG2
#pragma config MCLRE = ON       // Master Clear Enable bit (MCLR pin is Master Clear function)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config LPBOREN = OFF    // Low-Power BOR enable bit (ULPBOR disabled)
#pragma config BOREN = ON       // Brown-out reset enable bits (Brown-out Reset Enabled, SBOREN bit is ignored)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (VBOR) set to 1.9V on LF, and 2.45V on F Devices)
#pragma config ZCD = OFF        // Zero-cross detect disable (Zero-cross detect circuit is disabled at POR.)
#pragma config PPS1WAY = ON     // Peripheral Pin Select one-way control (The PPSLOCK bit can be cleared and set only once in software)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable bit (Stack Overflow or Underflow will cause a reset)

// CONFIG3
#pragma config WDTCPS = WDTCPS_31// WDT Period Select bits (Divider ratio 1:65536; software control of WDTPS)
#pragma config WDTE = OFF       // WDT operating mode (WDT Disabled, SWDTEN is ignored)
#pragma config WDTCWS = WDTCWS_7// WDT Window Select bits (window always open (100%); software control; keyed access not required)
#pragma config WDTCCS = SC      // WDT input clock selector (Software Control)

// CONFIG4
#pragma config BBSIZE = BB512   // Boot Block Size Selection bits (512 words boot block size)
#pragma config BBEN = OFF       // Boot Block Enable bit (Boot Block disabled)
#pragma config SAFEN = OFF      // SAF Enable bit (SAF disabled)
#pragma config WRTAPP = OFF     // Application Block Write Protection bit (Application Block not write protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block not write protected)
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration Register not write protected)
#pragma config WRTSAF = OFF     // Storage Area Flash Write Protection bit (SAF not write protected)
#pragma config LVP = ON         // Low Voltage Programming Enable bit (Low Voltage programming enabled. MCLR/Vpp pin function is MCLR.)

// CONFIG5
#pragma config CP = OFF         // UserNVM Program memory code protection bit (UserNVM code protection disabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdint.h>

#define _XTAL_FREQ 20000000UL

void init()
{
  // setup ports
  TRISA = 0b11111110;
  TRISB = 0b11111000;
  TRISC = 0b10001111;
  TRISD = 0b00000000;
  TRISE = 0b11111100;

  ANSELA = 0; // PORTA all digital
  ANSELB = 0; // PORTB all digital
  ANSELC = 0; // PORTC all digital

  // setup EUSART
  RC6PPS = 0x0F; // TX1 on RC6
  RX1DTPPSbits.RX1DTPPS = 0x17; // RX1 on RC7
  RCSTA1bits.SPEN = 1;
  TXSTA1bits.SYNC = 0;
  TXSTA1bits.TXEN = 1;
  RCSTA1bits.CREN = 1;
  TXSTA1bits.BRGH = 1;
  BAUDCON1bits.BRG16 = 1;
  SP1BRG = 42; // 115.2k @ 20 MHz clock

  // setup MSSP for SPI master
  RB1PPS = 0x17;  // SCK2
  RB2PPS = 0x18;  // SDO2
  //SSP2CON1bits.SSPM = 0b1010; // SPI master, use SSP2ADD for BRG
  //SSP2ADDbits.SSPADD = 0x4f; // 100 kHz SPI @ 32 MHz clock
  SSP2CON1bits.SSPM = 0b0001; // SPI master, 1.25 MHz @ 20 MHz clock
  SSP2CON1bits.SSPEN = 1; // enable MSSP
  SSP2CON1bits.CKP = 0; // idle clock low
  SSP2STATbits.CKE = 1; // transmit on active->idle transition
  LATBbits.LATB0 = 1; // SS high

//#define FREQ_COUNTER_MODE
#ifdef FREQ_COUNTER_MODE
  // setup timer 1: gate mode based on timer0 overflows
  T1CKIPPSbits.T1CKIPPS = 0x04; // T1CKI on RA4
  T1CLKbits.CS = 0b0000; // timer1 clocked from T1CKIPPS
  T1CONbits.CKPS = 0b00; // 1:1 prescale
  T1CONbits.nSYNC = 1; // do not sync
  T1GCONbits.GE = 1; // enable timer 1 gating
  T1GCONbits.GPOL = 1; // gate active high
  T1GCONbits.GTM = 1; // gate toggle mode
  T1GCONbits.GSPM = 1; // gate single-pulse mode
  T1GATEbits.GSS = 0b00001; // gate on timer0 overflow
  PIE4bits.TMR1IE = 1; // enable timer1 interrupts
  INTCONbits.PEIE = 1; // enable periperhal interrupts
  INTCONbits.GIE = 1; // global interrupt enable
  
  // setup timer0
  T0CON0bits.T016BIT = 1; // 16-bit timer
  T0CON0bits.T0OUTPS = 0b000; // 1:1 postscale
  T0CON1bits.T0CS = 0b010; // Fosc/4
  T0CON1bits.T0CKPS = 0b0111;// 1:128 prescale

#else 

  // setup timer0
  T0CKIPPSbits.T0CKIPPS = 0x04; // T0CKI on RA4
  T0CON0bits.T016BIT = 1; // 16-bit timer
  T0CON0bits.T0OUTPS = 0b000; // 1:1 postscale
  T0CON1bits.T0CS = 0b000; // active-high T0CKI input
  T0CON1bits.T0ASYNC = 1; // async increment
  T0CON1bits.T0CKPS = 0b0000;// 1:1 prescale

  // externally gated counter mode
  // setup timer 1: gate mode based on timer0 overflows

  T1CLKbits.CS = 0b1001; // timer1 clocked from timer1 overflow
  T1CONbits.CKPS = 0b00; // 1:1 prescale
  T1CONbits.nSYNC = 1; // do not sync
  T1GCONbits.GE = 0; // disable timer 1 gating
#endif

}

void putchar(char c)
{
  while(!PIR3bits.TX1IF);
  TX1REG = c;
}

void send_hex(uint16_t value)
{
  char hexchar[16] = {'0', '1', '2', '3', '4', '5', '6', '7',
                      '8', '9', 'a', 'b', 'c', 'd', 'e', 'f'};
  putchar('0');
  putchar('x');
  putchar(hexchar[(value & 0xf000)>>12]);
  putchar(hexchar[(value & 0x0f00)>> 8]);
  putchar(hexchar[(value & 0x00f0)>> 4]);
  putchar(hexchar[(value & 0x000f)>> 0]);
  putchar('\n');
}

void send_ascii_int(uint32_t value)
{
  uldiv_t x;
  uint8_t leading_found = 0;
  uint32_t place = 1000000000;
  while (place > 1){
    x = uldiv(value, place);
    if (x.quot > 0 || leading_found){
      leading_found = 1;
      putchar('0' + x.quot);
    }
    value = x.rem;
    place = place / 10;
  }
  putchar('0' + value);
}


char getchar()
{

  if (RC1STAbits.OERR){
    RC1STAbits.CREN = 0; // clear overrun error
    RC1STAbits.CREN = 1; // re-enable RX
  }
  while(!PIR3bits.RC1IF);
  return RC1REG;
}

uint8_t char_avail()
{
  return !PIR3bits.RC1IF;
}

uint8_t timer1_overflow_count;

void __interrupt() ISR(void)
{
  if (PIR4bits.TMR1IF){
    timer1_overflow_count++;
    PIR4bits.TMR1IF = 0;
  }
}

void count_frequency()
{
  T0CON0bits.T0EN = 0; // stop timer 0
  TMR0H = 0;
  TMR0L = 0;

  timer1_overflow_count = 0;

  T1CONbits.TMR1ON = 0; // stop timer 1
  TMR1H = 0;
  TMR1L = 0;
  T1CONbits.TMR1ON = 1; // enable timer 1

  T1GCONbits.GGO_nDONE = 1; // arm timer1 gating
  T0CON0bits.T0EN = 1; // start timer 0
  while(T1GCONbits.GGO_nDONE); // wait for frequency to be counted

  send_hex(((uint16_t)TMR1H << 8) | TMR1L);
  send_ascii_int(((uint32_t)timer1_overflow_count << 16) | ((uint32_t)TMR1H << 8) | TMR1L);
}

// include 74LVC prescaling bits
uint32_t measure_gated_osc(void)
{
  INTCONbits.GIE = 0; // global interrupt disable
  LATAbits.LATA0 = 0; // disable delay-line osc

  LATCbits.LATC4 = 0; // reset LVC prescaler
  LATCbits.LATC4 = 1;
  LATCbits.LATC5 = 1; // reset ECL prescaler
  LATCbits.LATC5 = 0;

  T0CON0bits.T0EN = 0; // stop timer0
  TMR0H = 0;
  TMR0L = 0;

  T1CONbits.TMR1ON = 0; // stop timer1
  TMR1H = 0;
  TMR1L = 0;
  T1CONbits.TMR1ON = 1; // enable timer1

  T0CON0bits.T0EN = 1; // start timer0
  asm("BANKSEL PORTA");
  asm("BCF PORTA, 0"); // enable delay-line osc

  _delay(_XTAL_FREQ/4 - 2); // delay 1s

  asm("BANKSEL PORTA");
  asm("BSF PORTA, 0"); // disable delay-line osc
  T0CON0bits.T0EN = 0; // stop timer0

  T1CONbits.TMR1ON = 0; // stop timer1 (DL osc already stopped by gate)

  return(((uint32_t)TMR1H << 28) | ((uint32_t)TMR1L << 20) |
         ((uint32_t)TMR0H << 12) | ((uint32_t)TMR0L <<  4) |
         (PORTC & 0x0f));
}


void set_delay(int16_t value)
{
  LATD = value & 0xff;
  LATE = (value >> 8) & 0x03;
}

void set_fine_tune(int16_t value)
{
  // Write to MCP4821 DAC
  LATBbits.LATB0 = 0;
  SSP2BUF = 0b00110000 | ((value & 0x0f00) >> 8);
  while(!SSP2STATbits.BF);
  SSP2BUF = value & 0xff;
  while(!SSP2STATbits.BF);
  LATBbits.LATB0 = 1;
}

void main(void) {
  init();

  while(1){
    for (uint16_t delay_val = 0 ; delay_val < 1024; delay_val++){
      uint16_t tune_val = 0;
      set_fine_tune(tune_val);
      set_delay(delay_val);
      uint32_t freq = measure_gated_osc();
      send_ascii_int(delay_val);
      putchar(' ');
      send_ascii_int(tune_val);
      putchar(' ');
      send_ascii_int(freq);
      putchar('\n');
    }
  }

  return;
}
