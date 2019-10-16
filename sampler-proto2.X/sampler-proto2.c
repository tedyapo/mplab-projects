/*
 * File:   sampler-proto2.c
 * Author: tyapo
 *
 * Created on September 30, 2019
 */

// PIC18F67K40 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1L
#pragma config FEXTOSC = ECH    // External Oscillator mode Selection bits (EC (external clock) above 8 MHz; PFM set to high power)
#pragma config RSTOSC = EXTOSC_4PLL// Power-up default value for COSC bits (EXTOSC with 4x PLL, with EXTOSC operating per FEXTOSC bits)

// CONFIG1H
#pragma config CLKOUTEN = OFF   // Clock Out Enable bit (CLKOUT function is disabled)
#pragma config CSWEN = ON       // Clock Switch Enable bit (Writing to NOSC and NDIV is allowed)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)

// CONFIG2L
#pragma config MCLRE = EXTMCLR  // Master Clear Enable bit (If LVP = 0, MCLR pin is MCLR; If LVP = 1, RG5 pin function is MCLR )
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (Power up timer disabled)
#pragma config LPBOREN = OFF    // Low-power BOR enable bit (ULPBOR disabled)
#pragma config BOREN = OFF      // Brown-out Reset Enable bits (Brown-out Reset disabled)

// CONFIG2H
#pragma config BORV = VBOR_2P45 // Brown Out Reset Voltage selection bits (Brown-out Reset Voltage (VBOR) set to 2.45V)
#pragma config ZCD = OFF        // ZCD Disable bit (ZCD disabled. ZCD can be enabled by setting the ZCDSEN bit of ZCDCON)
#pragma config PPS1WAY = ON     // PPSLOCK bit One-Way Set Enable bit (PPSLOCK bit can be cleared and set only once; PPS registers remain locked after one clear/set cycle)
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config DEBUG = OFF      // Debugger Enable bit (Background debugger disabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Extended Instruction Set and Indexed Addressing Mode disabled)

// CONFIG3L
#pragma config WDTCPS = WDTCPS_31// WDT Period Select bits (Divider ratio 1:65536; software control of WDTPS)
#pragma config WDTE = OFF       // WDT operating mode (WDT Disabled)

// CONFIG3H
#pragma config WDTCWS = WDTCWS_7// WDT Window Select bits (window always open (100%); software control; keyed access not required)
#pragma config WDTCCS = SC      // WDT input clock selector (Software Control)

// CONFIG4L
#pragma config WRT0 = OFF       // Write Protection Block 0 (Block 0 (000800-003FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection Block 1 (Block 1 (004000-007FFFh) not write-protected)
#pragma config WRT2 = OFF       // Write Protection Block 2 (Block 2 (008000-00BFFFh) not write-protected)
#pragma config WRT3 = OFF       // Write Protection Block 3 (Block 3 (00C000-00FFFFh) not write-protected)
#pragma config WRT4 = OFF       // Write Protection Block 3 (Block 4 (010000-013FFFh) not write-protected)
#pragma config WRT5 = OFF       // Write Protection Block 3 (Block 5 (014000-017FFFh) not write-protected)
#pragma config WRT6 = OFF       // Write Protection Block 3 (Block 6 (018000-01BFFFh) not write-protected)
#pragma config WRT7 = OFF       // Write Protection Block 3 (Block 7 (01C000-01FFFFh) not write-protected)

// CONFIG4H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-30000Bh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block (000000-0007FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)
#pragma config SCANE = ON       // Scanner Enable bit (Scanner module is available for use, SCANMD bit can control the module)
#pragma config LVP = ON         // Low Voltage Programming Enable bit (Low voltage programming enabled. MCLR/VPP pin function is MCLR. MCLRE configuration bit is ignored)

// CONFIG5L
#pragma config CP = OFF         // UserNVM Program Memory Code Protection bit (UserNVM code protection disabled)
#pragma config CPD = OFF        // DataNVM Memory Code Protection bit (DataNVM code protection disabled)

// CONFIG5H

// CONFIG6L
#pragma config EBTR0 = OFF      // Table Read Protection Block 0 (Block 0 (000800-003FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection Block 1 (Block 1 (004000-007FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection Block 2 (Block 2 (008000-00BFFFh) not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection Block 3 (Block 3 (00C000-00FFFFh) not protected from table reads executed in other blocks)
#pragma config EBTR4 = OFF      // Table Read Protection Block 4 (Block 4 (010000-013FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR5 = OFF      // Table Read Protection Block 5 (Block 5 (014000-017FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR6 = OFF      // Table Read Protection Block 6 (Block 6 (018000-01BFFFh) not protected from table reads executed in other blocks)
#pragma config EBTR7 = OFF      // Table Read Protection Block 7 (Block 7 (01C000-01FFFFh) not protected from table reads executed in other blocks)

// CONFIG6H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot Block (000000-0007FFh) not protected from table reads executed in other blocks)


#include <xc.h>
#include <stdint.h>

#define _XTAL_FREQ 64000000UL

#include "calibration.h"

void putchar(char c)
{
  while(!PIR3bits.TX1IF);
  TX1REG = c;
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

// indicator LED on RG0
void set_LED(uint8_t state)
{
  LATGbits.LATG0 = state;
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

// note: see if subtraction is faster than uldiv here
void send_ascii_int(int32_t value)
{
  uldiv_t x;
  uint8_t leading_found = 0;
  uint32_t place = 1000000000;
  if (value < 0){
    putchar('-');
    value = -value;
  }
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

typedef enum {CS_EXTERNAL = 0, CS_INTERNAL = 1, CS_FEEDBACK = 2} clock_source_t;
void select_clock_source(clock_source_t cs)
{
  switch(cs){
  case CS_EXTERNAL:
    LATAbits.LATA5 = 0; // CLK-SEL0
    LATAbits.LATA6 = 0; // CLK-SEL1
    break;
  case CS_INTERNAL:
    LATAbits.LATA5 = 0; // CLK-SEL0
    LATAbits.LATA6 = 1; // CLK-SEL1
    break;
  case CS_FEEDBACK:
    LATAbits.LATA5 = 1; // CLK-SEL0
    LATAbits.LATA6 = 1; // CLK-SEL1
    break;
  }
}

void set_delay(uint16_t value)
{
  LATD = value & 0xff;
  LATF = (LATE & 0xfc) | ((value >> 8) & 0x03);
}

uint8_t internal_clock_sample()
{
  // toggle INT-CLK to sample input
  LATAbits.LATA3 = 1;   // sample input
  LATAbits.LATA3 = 0;   // transfer result to flip-flop
  asm("NOP");
  return PORTFbits.RF6; // COMP-OUT has sampled bit
}

uint8_t external_clock_sample()
{
  while (!PORTAbits.RA2); // wait for SAMPLE-CLK high
  while (PORTAbits.RA2);  // wait for SAMPLE-CLK low
  return PORTFbits.RF6;  // COMP-OUT has sampled bit
}

// Write to MCP4921 DAC for VREF
void set_DAC(uint16_t value)
{
  LATBbits.LATB5 = 0; // VREF-CSb

  SSP1BUF = 0b00110000 | ((value & 0x0f00) >> 8);
  while(!SSP1STATbits.BF);
  SSP1BUF = value & 0xff;
  while(!SSP1STATbits.BF);

  LATBbits.LATB5 = 1; // VREF-CSb
}

// Write to MCP4821 DAC for FTUNE
void set_FTUNE(uint16_t value)
{
  LATBbits.LATB4 = 0; // FTUNE-CSb

  SSP2BUF = 0b00110000 | ((value & 0x0f00) >> 8);
  while(!SSP2STATbits.BF);
  SSP2BUF = value & 0xff;
  while(!SSP2STATbits.BF);

  LATBbits.LATB4 = 1; // FTUNE-CSb
}

///---------------------///  

void init()
{
  // setup ports

  // RA0: x
  // RA1: x
  // RA2: SAMPLE-CLK
  // RA3: INT-CLK
  // RA4: TC
  // RA5: CLK-SEL0
  // RA6: CLK-SEL1
  // RA7: x
  TRISA = 0b10010111;
  ANSELA = 0x00;
  WPUA = 0x00;
  ODCONA = 0x00;
  SLRCONA = 0x08;
  INLVLA = 0xff;
  
  // RB0: FTUNE-SDO
  // RB1: FTUNE-SCK
  // RB2: VREF-SDO
  // RB3: VREF-SCK
  // RB4: FTUNE-CSb
  // RB5: VREF-CSb
  TRISB = 0b11000000;
  ANSELB = 0x00;
  WPUB = 0x00;
  ODCONB = 0x00;
  SLRCONB = 0x00;
  INLVLB = 0xff;
  
  // RC0: Q1
  // RC1: Q0
  // RC2: Q2
  // RC3: Q3
  // RC4: COUNTER-RESET
  // RC5: x
  // RC6: TX
  // RC7: RX
  TRISC = 0b10101111;
  WPUC = 0x00;
  ODCONC = 0x00;
  SLRCONC = 0x00;
  INLVLC = 0xff;
  
  // RD0: DELAY0
  // RD1: DELAY1
  // RD2: DELAY2
  // RD3: DELAY3
  // RD4: DELAY4
  // RD5: DELAY5
  // RD6: DELAY6
  // RD7: DELAY7
  TRISD = 0x00;
  ANSELD = 0x00;
  WPUD = 0x00;
  ODCOND = 0x00;
  SLRCOND = 0x00;
  INLVLD = 0xff;
  
  // RE0:
  // RE1:
  // RE2:
  // RE3: 
  // RE4: 
  // RE5: 
  // RE6:
  // RE7:
  TRISE = 0xff;
  ANSELE = 0x00;
  WPUE = 0x00;
  ODCONE = 0x00;
  SLRCONE = 0x00;
  INLVLE = 0xff;

  // RF0: DELAY8
  // RF1: DELAY9
  // RF2: FTUNE-LDACn
  // RF3: VREF-LDACn
  // RF4: x
  // RF5: x
  // RF6: COMP-OUT
  // RF7: TEMP-ADC
  TRISF = 0xf0;
  ANSELF = 0x80;
  WPUF = 0x00;
  ODCONF = 0x00;
  SLRCONF = 0x00;
  INLVLF = 0xff;

  // RG0: LED
  // RG1:
  // RG2:
  // RG3:
  // RG4:
  // RG5:
  // RG6:
  // RG7: EXT-TEMP
  TRISG = 0xfe;
  ANSELG = 0x80;
  WPUG = 0x00;
  ODCONG = 0x00;
  SLRCONG = 0x00;
  INLVLG = 0xff;

  // RH0:
  // RH1:
  // RH2:
  // RH3:
  TRISH = 0xff;
  WPUH = 0x00;
  ODCONH = 0x00;
  SLRCONH = 0x00;
  INLVLH = 0xff;

  // setup MSSP1 for SPI master (VREF)
  RB3PPS = 0x19;  // VREF-SCK MSSP1
  RB2PPS = 0x1A;  // VREF-SDI MSSP1
  SSP1CON1bits.SSPEN = 0; // disable MSSP1
  SSP1CON1bits.SSPM = 0b0000; // SPI master, 16 MHz @ 64 MHz clock
  SSP1CON1bits.CKP = 0; // idle clock low
  SSP1STATbits.CKE = 1; // transmit on active->idle transition
  SSP1CON1bits.SSPEN = 1; // enable MSSP1
  LATBbits.LATB5 = 1; // VREF-CSb high

  // setup MSSP2 for SPI master (FTUNE)
  RB1PPS = 0x1B;  // FTUNE-SCK MSSP2
  RB0PPS = 0x1C;  // FTUNE-SDI MSSP2
  SSP2CON1bits.SSPEN = 0; // enable MSSP2
  SSP2CON1bits.SSPM = 0b0000; // SPI master, 16 MHz @ 64 MHz clock
  SSP2CON1bits.CKP = 0; // idle clock low
  SSP2STATbits.CKE = 1; // transmit on active->idle transition
  SSP2CON1bits.SSPEN = 1; // enable MSSP2
  LATBbits.LATB4 = 1; // FTUNE-CSb high

  // setup EUSART
  RC6PPS = 0x0C; // TX1 on RC6
  RX1PPSbits.RX1PPS = 0x17; // RX1 on RC7
  RCSTA1bits.SPEN = 1;
  TXSTA1bits.SYNC = 0;
  TXSTA1bits.TXEN = 1;
  RCSTA1bits.CREN = 1;
  TXSTA1bits.BRGH = 1;
  BAUDCON1bits.BRG16 = 1;
//  SP1BRG = 138; // 115200 @ 64 MHz clock
//  SP1BRG = 34; // 460.8k @ 64 MHz clock
//  SP1BRG = 16; // 921600 @ 64 MHz clock
  SP1BRG = 7; // 2 Mbaud @ 64 MHz clock

  // set LDAC low: DAC loads on rising CSn
  LATFbits.LATF2 = 0; // FTUNE-LDACn
  LATFbits.LATF3 = 0; // VREF-LDACn

  // setup timer0
  T0CKIPPSbits.T0CKIPPS = 0x04; // T0CKI on RA4
  T0CON0bits.T016BIT = 1; // 16-bit timer
  T0CON0bits.T0OUTPS = 0b000; // 1:1 postscale
  T0CON1bits.T0CS = 0b000; // active-high T0CKI input
  T0CON1bits.T0ASYNC = 1; // async increment
  T0CON1bits.T0CKPS = 0b0000;// 1:1 prescale

  // setup timer1
  TMR1CLKbits.CS = 0b1000; // timer1 clocked from timer1 overflow
  T1CONbits.CKPS = 0b00; // 1:1 prescale
  T1CONbits.nSYNC = 1; // do not sync
  T1CONbits.RD16 = 0; // 8-bit reads
  T1GCONbits.GE = 0; // disable timer 1 gating

  return;

#ifdef ADC_ENABLED
  // enable internal temperature sensor
  FVRCONbits.TSEN = 1; // enable temp. sensor
  FVRCONbits.TSRNG = 1; // high range for more resolution

  // setup fixed voltage reference
  FVRCONbits.FVREN = 1; // enable fixed voltage reference
  FVRCONbits.ADFVR = 0b10; // 2.048 V output for ADC ref

  // setup ADC
  ADCON0bits.ADON = 1; // enable ADC
  ADCON1bits.ADFM = 1; // right justified result
  ADCON1bits.ADCS = 0b110; // 3.2 us conversion @ 20 MHz
  ADCON1bits.ADPREF = 0b11; // Vref uses FVR
#endif
}

#if COUNTER_ENABLED
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

  T1CONbits.ON = 0; // stop timer 1
  TMR1H = 0;
  TMR1L = 0;
  T1CONbits.ON = 1; // enable timer 1

  T1GCONbits.GGO_nDONE = 1; // arm timer1 gating
  T0CON0bits.T0EN = 1; // start timer 0
  while(T1GCONbits.GGO_nDONE); // wait for frequency to be counted

  send_hex(((uint16_t)TMR1H << 8) | TMR1L);
  send_ascii_int(((uint32_t)timer1_overflow_count << 16) | ((uint32_t)TMR1H << 8) | TMR1L);
}

uint16_t read_NVM(uint16_t address)
{
  NVMCON1bits.NVMREGS = 1; // access DIA
  NVMADRH = (address & 0xff00) >> 8;
  NVMADRL = address & 0x00ff;
  NVMCON1bits.RD = 1;
  return (((uint16_t)NVMDATH) << 8) | NVMDATL;
}

uint16_t measure_temperature()
{
  float sum = 0;
  ADCON0bits.CHS = 0b111100; // select temp. sensor
  __delay_ms(1);
  for (uint8_t i=0; i<10; i++){
    ADCON0bits.GOnDONE = 1;
    while(ADCON0bits.GOnDONE){
      continue;
    }

    uint16_t value = ((uint16_t)ADRESH << 8) | ADRESL;
    float ft = 90.f + (((float)value - read_NVM(DIA_TSHR2))*read_NVM(DIA_FVRA2X))/
      (-3.684f*1023);
    sum += ft;
  }
  uint16_t t = 10*sum;
  return t;
}

uint16_t external_temperature()
{
  ADCON0bits.CHS = 0b001100; // select RB4
  __delay_ms(1);

  float sum = 0;
  for (uint8_t i=0; i<10; i++){
    ADCON0bits.GOnDONE = 1;
    while(ADCON0bits.GOnDONE){
      continue;
    }

    uint16_t value = ((uint16_t)ADRESH << 8) | ADRESL;
    float ft = ((float)value * read_NVM(DIA_FVRA2X)) / 1023 - 500;
    sum += ft;
  }
  return sum;
}
#endif


// include 74LVC prescaling bits
typedef enum {GATE_TIME_10ms = 0, GATE_TIME_100ms = 1, GATE_TIME_1s = 2} gate_time_t;
uint32_t measure_gated_osc(gate_time_t gate_time)
{
  INTCONbits.GIE = 0; // global interrupt disable

  select_clock_source(CS_INTERNAL);

  LATCbits.LATC4 = 0; // reset LVC prescaler
  LATCbits.LATC4 = 1; // enable LVC prescaler

  T0CON0bits.T0EN = 0; // stop timer0
  TMR0H = 0;
  TMR0L = 0;

  T1CONbits.TMR1ON = 0; // stop timer1
  TMR1H = 0;
  TMR1L = 0;
  T1CONbits.TMR1ON = 1; // enable timer1

  T0CON0bits.T0EN = 1; // start timer0

//  asm("BANKSEL PORTA");
//  asm("BCF PORTA, 0"); // enable delay-line osc
 // asm("BANKSEL PORTA");
  //asm("BSF PORTA, 0"); // disable delay-line osc

  switch(gate_time){
  case GATE_TIME_10ms:
    LATAbits.LATA5 = 1; // CLK-SEL0 set to feedback
    _delay(_XTAL_FREQ/400 - 2); // delay 10ms
    LATAbits.LATA5 = 0; // CLK-SEL0 set to internal clock
    break;
  case GATE_TIME_100ms:
    LATAbits.LATA5 = 1; // CLK-SEL0 set to feedback
    _delay(_XTAL_FREQ/40 - 2); // delay 10ms
    LATAbits.LATA5 = 0; // CLK-SEL0 set to internal clock
    break;
  case GATE_TIME_1s:
    LATAbits.LATA5 = 1; // CLK-SEL0 set to feedback
    _delay(_XTAL_FREQ/4 - 2); // delay 1s
    LATAbits.LATA5 = 0; // CLK-SEL0 set to internal clock
    break;
  }

  T1CONbits.TMR1ON = 0; // stop timer1 (DL osc already stopped by gate)

  LATCbits.LATC4 = 0; // reset LVC prescaler

  return(((uint32_t)TMR1H << 28) | ((uint32_t)TMR1L << 20) |
         ((uint32_t)TMR0H << 12) | ((uint32_t)TMR0L <<  4) |
         (PORTC & 0x0c) | ((PORTC & 0x1)<<1) | ((PORTC & 0x2)>>1));
}


int16_t DAC_to_V(float counts)
{
  //float voltage = 1.024f * (-2.f + 4.f * counts/4095.f); 
  //float voltage = 1.191221e-8f*(84992.f*counts - 171756585.f);
  float voltage = -2.04437f + (2.09233f+2.04437f)*counts/4095.f;
  return (int)(10000 * voltage);
}

void set_timebase(uint16_t idx)
{
  set_FTUNE(calibration[idx].ftune);
  __delay_us(100);
  set_delay(calibration[idx].delay);
}

void calibrate_timebase()
{
  set_FTUNE(0);
  __delay_us(100);
  set_delay(0);
  uint32_t freq = measure_gated_osc(GATE_TIME_100ms);
  send_ascii_int(-1);
  putchar(' ');
  send_ascii_int(freq);
  putchar('\n');
  set_delay(1023);
  freq = measure_gated_osc(GATE_TIME_100ms);
  send_ascii_int(-2);
  putchar(' ');
  send_ascii_int(freq);
  putchar('\n');
}


void internal_averaged_SAR()
{
  uint16_t delay_step = 1;
  uint8_t oversample = 254;
 
  select_clock_source(CS_INTERNAL);

  while(1){
    calibrate_timebase();
    select_clock_source(CS_INTERNAL);
    for (uint16_t delay_idx=0; delay_idx<1024; delay_idx+=delay_step){
      set_timebase(delay_idx);
      float sum = 0;
      for (uint8_t i=0; i<oversample; i++){
        uint16_t idx = 0;
        uint16_t step = 2047;
        while (step > 0){
          set_DAC(idx+step);
          __delay_us(10);
          uint8_t count = 0;
          if (internal_clock_sample()){
            idx += step;
          }
          step >>= 1;
        }
        sum += idx;
      }
      float idx = sum/oversample;
        
      send_ascii_int(delay_idx);
      putchar(' ');
      send_ascii_int(DAC_to_V(idx));
      //send_ascii_int(idx);
//#define VISUAL
#ifdef VISUAL
      int16_t len = 1+(idx-1651)/10;
      if (len < 1) len = 1;
      if (len > 60) len = 60;
      for (int16_t i = 0; i < len; i++){
        putchar(' ');
      }
      putchar('*');
#endif
      putchar('\n');
    }
  }
}

void external_averaged_SAR()
{
  uint16_t delay_step = 1;
  uint8_t oversample = 100;
 
  select_clock_source(CS_EXTERNAL);

  while(1){
    calibrate_timebase();
    select_clock_source(CS_EXTERNAL);
    for (uint16_t delay_idx=0; delay_idx<1024; delay_idx+=delay_step){
      set_timebase(delay_idx);
      float sum = 0;
      for (uint8_t i=0; i<oversample; i++){
        uint16_t idx = 0;
        uint16_t step = 2047;
        while (step > 0){
          set_DAC(idx+step);
          __delay_us(10);
          uint8_t count = 0;
          if (external_clock_sample()){
            idx += step;
          }
          step >>= 1;
        }
        sum += idx;
      }
      uint16_t idx = sum/oversample;
        
      send_ascii_int(delay_idx);
      putchar(' ');
      send_ascii_int(DAC_to_V(idx));
      //send_ascii_int(idx);
//#define VISUAL
#ifdef VISUAL
      int16_t len = 1+(idx-1651)/10;
      if (len < 1) len = 1;
      if (len > 60) len = 60;
      for (int16_t i = 0; i < len; i++){
        putchar(' ');
      }
      putchar('*');
#endif
      putchar('\n');
    }
  }
}

void internal_SAR()
{
  uint16_t delay_step = 1;
  uint8_t oversample = 11;
 
  select_clock_source(CS_INTERNAL);

  while(1){
    calibrate_timebase();
    select_clock_source(CS_INTERNAL);
    for (uint16_t delay_idx=0; delay_idx<1024; delay_idx+=delay_step){
      set_timebase(delay_idx);
      uint16_t idx = 0;
      uint16_t step = 2047;
      while (step > 0){
        set_DAC(idx+step);
        __delay_us(20);
        uint8_t count = 0;
        for (uint8_t i=0; i<oversample; i++){
          count += internal_clock_sample();
        }
        if (count > (oversample>>1)){
          idx += step;
        }
        step >>= 1;
      }
        
      send_ascii_int(delay_idx);
      putchar(' ');
      send_ascii_int(DAC_to_V(idx));
      //send_ascii_int(idx);
//#define VISUAL
#ifdef VISUAL
      int16_t len = 1+(idx-1651)/10;
      if (len < 1) len = 1;
      if (len > 60) len = 60;
      for (int16_t i = 0; i < len; i++){
        putchar(' ');
      }
      putchar('*');
#endif
      putchar('\n');
    }
  }
}


void external_SAR()
{
  uint16_t delay_step = 1;
  uint8_t oversample = 10;
 
  while(1){
    calibrate_timebase();
    select_clock_source(CS_EXTERNAL);
    for (uint16_t delay_idx=0; delay_idx<1024; delay_idx+=delay_step){
      set_timebase(delay_idx);
      uint16_t idx = 0;
      uint16_t step = 2047;
      while (step > 0){
        set_DAC(idx+step);
        __delay_us(10);
        uint8_t count = 0;
        for (uint8_t i=0; i<oversample; i++){
          count += external_clock_sample();
        }
        if (count > (oversample>>1)){
          idx += step;
        }
        step >>= 1;
      }
        
      send_ascii_int(delay_idx);
      putchar(' ');
      send_ascii_int(DAC_to_V(idx));
//      send_ascii_int(idx);
//#define VISUAL
#ifdef VISUAL
      int16_t len = 1+(idx-2013)/6;
      if (len < 0) len = 0;
      if (len > 45) len = 45;
      for (uint8_t i = 0; i < len; i++){
        putchar(' ');
      }
      putchar('*');
#endif

      putchar('\n');
    }
  }
}

void internal_count()
{
  uint16_t dac_step = 1;
  uint16_t delay_step = 1;
 
  select_clock_source(CS_INTERNAL);

  while(1){
    calibrate_timebase();
    for (uint16_t delay_idx=0; delay_idx<1024; delay_idx+=delay_step){

      set_timebase(delay_idx);
      uint16_t count = 0;
      for (uint16_t i=0; i<4096; i+=dac_step){
        set_DAC(i);
        __delay_us(10);
        count += external_clock_sample();
      }
    
      send_ascii_int(delay_idx);
      putchar(' ');
      send_ascii_int(DAC_to_V(dac_step*count));
//#define VISUAL
#ifdef VISUAL
      uint16_t len = 1+(dac_step*count/16);
      for (uint8_t i = 0; i < len; i++){
        putchar(' ');
      }
      putchar('*');
#endif
      putchar('\n');
    }
  }
}

void external_count()
{
  select_clock_source(CS_EXTERNAL);

  uint16_t dac_step = 1;
  uint16_t delay_step = 1;
  uint8_t oversample = 1;

  while(1){
    calibrate_timebase();
    select_clock_source(CS_EXTERNAL);

    for (uint16_t delay_idx=275; delay_idx<375; delay_idx+=delay_step){
      set_timebase(delay_idx);
      uint16_t count = 0;
      for (uint16_t i=0; i<4096; i+=dac_step){
        set_DAC(i);
        if (i){
          __delay_us(5);
        } else {
          __delay_us(10);
        }
        for (uint8_t j=0; j<oversample; j++){
          count += external_clock_sample();
        } 
      }
    
      send_ascii_int(delay_idx);
      putchar(' ');
      send_ascii_int(DAC_to_V(count/oversample));
//#define VISUAL
#ifdef VISUAL
      uint16_t len = 1+(count/oversample-1750)/5;
      for (uint8_t i = 0; i < len; i++){
        putchar(' ');
      }
      putchar('*');
#endif

      putchar('\n');
    }
  }
}

void measure_point(uint8_t type, uint16_t d, uint16_t f, gate_time_t gate_time)
{
  uint32_t freq = measure_gated_osc(gate_time);
  send_ascii_int(type);
  putchar(' ');
  send_ascii_int(d);
  putchar(' ');
  send_ascii_int(f);
  putchar(' ');
  send_ascii_int(freq);
  putchar('\n');
}

void thermal_memory_test(void)
{
  set_FTUNE(0);
    __delay_us(1000);
  uint16_t d=0;
  while(1){
    if (!(d&1)){
      set_delay(0);
      measure_point(0, 0, 0, GATE_TIME_10ms);
    }

    if (d&1){
      set_delay(1023);
      measure_point(0, 1023, 0, GATE_TIME_10ms);
    }

    set_delay(511);
    measure_point(0, 511, 0, GATE_TIME_10ms);
    d++;
  }

  set_delay(0);
  measure_point(0, 0, 0, GATE_TIME_10ms);
}

// collect data for timebase calibration
void calibrate()
{
  set_FTUNE(660);
  __delay_us(1000);
  for (uint16_t d=0; d<1024; d++){
    set_delay(0);
    measure_point(0, 0, 0, GATE_TIME_100ms);

    set_delay(1023);
    measure_point(0, 1023, 0, GATE_TIME_100ms);

/*
    if (d & 1){
      set_delay(d/2);
      measure_point(0, d/2, 0, GATE_TIME_100ms);
    } else {
      set_delay(1023-d/2);
      measure_point(0, 1023-d/2, 0, GATE_TIME_100ms);
    }
*/

    set_delay(d);
    measure_point(0, d, 0, GATE_TIME_100ms);

  }

  set_delay(0);
  for (uint16_t i = 0; i<2048; i++){

    set_FTUNE(0);
    __delay_us(1000);
    set_delay(0);
    measure_point(1, 0, 0, GATE_TIME_100ms);

    set_delay(1023);
    measure_point(1, 1023, 0, GATE_TIME_100ms);

    set_delay(0);

/*
    if (i & 1){
      set_FTUNE(i/2);
      __delay_us(1000);
      measure_point(1, 0, i/2, GATE_TIME_100ms);
    } else {
      set_FTUNE(2047-i/2);
      __delay_us(1000);
      measure_point(1, 0, 2047-i/2, GATE_TIME_100ms);
    }
*/
    
    set_FTUNE(i);
    __delay_us(1000);
    measure_point(1, 0, i, GATE_TIME_100ms);

  }
}

// collect data for timebase calibration: each bit tested
void bits_calibrate()
{
  set_FTUNE(0);
    __delay_us(1000);

  for (uint16_t d=0; d<1024; d++){
    for (uint16_t b=0; b<10; b++){
      set_delay(0);
      measure_point(0, 0, 0, GATE_TIME_100ms);
      
      set_delay(1<<b);
      measure_point(0, 1<<b, 0, GATE_TIME_100ms);
    }

    set_delay(d);
    measure_point(0, d, 0, GATE_TIME_100ms);
  }
}

void test_calibration()
{
  for (uint16_t i=0; i<1024; i++){
    uint16_t d;

    if (i & 1){
      d = i/2;
    } else {
      d = 1023-i/2;
    }

    set_FTUNE(0);
    __delay_us(1000);
    set_delay(0);
    measure_point(0, 0, 0, GATE_TIME_100ms);

    set_delay(1023);
    measure_point(0, 1023, 0, GATE_TIME_100ms);

    set_FTUNE(calibration[d].ftune);
    __delay_us(1000);
    set_delay(calibration[d].delay);
    measure_point(0, d, 0, GATE_TIME_100ms);
  }
}

void DC_CAL()
{
  while(1){
    set_DAC(2024);
    for(int i=0; i< 20; i++){
      _delay(_XTAL_FREQ/4 - 2); // delay 1s
    }
    set_DAC(2026);
    for(int i=0; i< 20; i++){
      _delay(_XTAL_FREQ/4 - 2); // delay 1s
    }
  }
}

void prob_dist()
{
  uint16_t min_delay = 250;
  uint16_t max_delay = 750;
  uint16_t min_dac = 1525;
  uint16_t max_dac = 2275;

  uint16_t num_tests = 255;
  while(1){
    for (uint16_t dac=min_dac; dac <= max_dac; dac++){
      set_DAC(dac);
      __delay_us(100);
      for (uint16_t delay=min_delay; delay <= max_delay; delay++){
        set_timebase(delay);
        uint16_t count = 0;
        for (uint16_t i=0; i<num_tests; i++){
          count += external_clock_sample();
        }
        send_ascii_int(delay);
        putchar(' ');
        send_ascii_int(dac);
        putchar(' ');
        send_ascii_int(count);
        putchar('\n');
      }
    }
  }
}

void main(void) {
  init();

  //DC_CAL();

  internal_averaged_SAR();
  //prob_dist();

  while(1){
    calibrate();
    //bits_calibrate();
    //test_calibration();
  }
  //thermal_memory_test();

//#define CAL_TEST
#ifdef CAL_TEST
  for (uint16_t i=0; i<980; i++){
    set_FTUNE(steps[i].ftune);
    set_delay(steps[i].delay);
    __delay_us(100);
    uint32_t freq = measure_gated_osc(GATE_TIME_100ms);
    send_ascii_int(i);
    putchar(' ');
    send_ascii_int(freq);
    putchar('\n');
  }
#endif

#define CAL_GATHER
#ifdef CAL_GATHER
  // collect data for timebase calibration
  set_FTUNE(0);
    __delay_us(1000);
  for (uint16_t d=0; d<1024; d++){
    if (!(d&1)){
      set_delay(0);
      measure_point(0, 0, 0, GATE_TIME_1s);
    }

    if (d&1){
      set_delay(1023);
      measure_point(0, 1023, 0, GATE_TIME_1s);
    }

    set_delay(d);
    measure_point(0, d, 0, GATE_TIME_1s);

  }

  set_delay(0);
  measure_point(0, 0, 0, GATE_TIME_1s);
  
  set_delay(0);
  uint16_t j = 0;
  for (uint16_t i = 0; i<2048; i+=16){


    if (!(j&1)){
      set_FTUNE(0);
      __delay_us(1000);
      set_delay(0);
      measure_point(1, 0, 0, GATE_TIME_1s);
    }

    if (j&1){
      set_FTUNE(0);
      __delay_us(1000);
      set_delay(1023);
      measure_point(1, 1023, 0, GATE_TIME_1s);
    }

    set_FTUNE(i);
    __delay_us(1000);
    set_delay(0);
    measure_point(1, 0, i, GATE_TIME_1s);

    j++;
  }

  set_FTUNE(0);
  __delay_us(1000);
  set_delay(1023);
  measure_point(1, 1023, 0, GATE_TIME_1s);

#endif

/*
  set_delay(0);
  for (uint16_t i = 0; i<2048; i+=8){
    set_FTUNE(i);
    __delay_us(100);
    uint32_t freq = measure_gated_osc(GATE_TIME_1s);
    send_ascii_int(i);
    putchar(' ');
    send_ascii_int(freq);
    putchar('\n');
  }
*/
/*
  set_FTUNE(0);
  for (uint16_t d=0; d<1024; d+=1023){
    set_delay(d);
    for (uint16_t i=0; i<10000; i++){
      uint32_t freq = measure_gated_osc(GATE_TIME_10ms);
      send_ascii_int(d);
      putchar(' ');
      send_ascii_int(10);
      putchar(' ');
      send_ascii_int(freq);
      putchar('\n');
    }
    for (uint16_t i=0; i<1000; i++){
      uint32_t freq = measure_gated_osc(GATE_TIME_100ms);
      send_ascii_int(d);
      putchar(' ');
      send_ascii_int(100);
      putchar(' ');
      send_ascii_int(freq);
      putchar('\n');
    }
    for (uint16_t i=0; i<100; i++){
      uint32_t freq = measure_gated_osc(GATE_TIME_1s);
      send_ascii_int(d);
      putchar(' ');
      send_ascii_int(1000);
      putchar(' ');
      send_ascii_int(freq);
      putchar('\n');
    }
  }
*/
/*
  set_FTUNE(0);
  for (int i=0; i<1024; i++){
    set_LED(i&1);
    set_delay(i);
    uint32_t freq = measure_gated_osc();
    send_ascii_int(i);
    putchar(' ');
    send_ascii_int(freq);
    putchar('\n');
  }
*/
  /*

  for(uint16_t i=0; i<5000; i++){
    __delay_us(1000);
  }
  internal_SAR();
  return;
*/
}
