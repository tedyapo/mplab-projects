/*
 * File:   comparator-sampler.c
 * Author: tyapo
 *
 * Created on June 21, 2019, 10:23 PM
 */

// CONFIG1
#pragma config FEXTOSC = ECH    // External Oscillator mode selection bits (EC above 8MHz; PFM set to high power)
#pragma config RSTOSC = EXT4X   // Power-up default value for COSC bits (EXTOSC with 4x PLL, with EXTOSC operating per FEXTOSC bits)
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

#include <xc.h>
#include <stdint.h>

#define _XTAL_FREQ 32000000UL


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

// indicator LED on RC4
void set_LED(uint8_t state)
{
  LATCbits.LATC4 = state;
}

typedef enum {CS_EXTERNAL = 0, CS_INTERNAL = 1} clock_source_t;
void select_clock_source(clock_source_t cs)
{
  LATEbits.LATE2 = cs;
}

void set_delay(uint16_t value)
{
  LATD = value & 0xff;
  LATE = (LATE & 0xfc) | ((value >> 8) & 0x03);
}

// internal clock mode
// 1. bit-banged clock and comparator reads
// 2. sysclock output (or PWM or other) and serial input
// external clock mode
// * must use serial port reads, and enable, disable clock by switching source

void init_single_sample()
{
  select_clock_source(CS_INTERNAL);
  RC0PPS = 0x00;  // LATC0 on RC0
  // toggle clock
  LATCbits.LATC0 = 0;
}

uint8_t internal_clock_single_sample()
{
  LATCbits.LATC0 = 1; // sample input
  LATCbits.LATC0 = 0; // transfer result to flip-flop
  return PORTBbits.RB3;
}

void enable_reference_clock()
{
  select_clock_source(CS_INTERNAL);
  // Note: maximum SPI input clock is 3.4 MHz
  //  => must divide fclk by at least 16
  // /16 -> 2 MHz
  // /32 -> 1 MHz
  // /64 -> 500 kHz
  // /128 -> 256 kHz
  //  can also use 500 kHz MFINTOSC fo other frequencies
  //  NCO can't output to portc directly
  //  looks like NCO1 can be input to CLC1, which can be routed to PORTC
  
  // setup reference clock output on RC0
  TRISCbits.TRISC0 = 0;
  RC0PPS = 0x1B;  // clock reference on RC0
  SLRCONCbits.SLRC0 = 0;
  CLKRCONbits.CLKREN = 1; // enable reference clock
  CLKRCONbits.CLKRDC = 0b10; // 50% duty cycle
  CLKRCONbits.CLKRDIV = 0b100; // 16:1
  CLKRCLKbits.CLKRCLK = 0b0000; // base clock = fosc
}

// Write to MCP4921 DAC
// !!! note: speed this up by tying off LDAC outside this routine
// !!!       use falling edge of CSn instead
void set_DAC(uint16_t value)
{
  // lower CSn
  LATBbits.LATB0 = 0;
  
  SSP2BUF = 0b00110000 | ((value & 0x0f00) >> 8);
  while(!SSP2STATbits.BF);
  SSP2BUF = value & 0xff;
  while(!SSP2STATbits.BF);

  // raise CSn
  LATBbits.LATB0 = 1;
}

void naive_scan(void)
{
  uint16_t delay_step = 8;
  uint16_t vref_step = 16;
    
  init_single_sample();
    
  for (uint16_t vref = 0; vref < 4096; vref += vref_step){
    set_DAC(vref);
    // wait for vref to settle
    // need 4.5 us delay for full steps; probably less for single-bit steps
    _delay(8*5); // 8 cycles per us        
    for (uint16_t delay = 0; delay < 1024; delay += delay_step){
      set_delay(delay);
      uint8_t sample = internal_clock_single_sample();
      send_ascii_int(vref);
      putchar(' ');
      send_ascii_int(delay);
      putchar(' ');
      send_ascii_int(sample);
      putchar('\n');
    }
  }
}

void counting_scan(void)
{
  uint16_t delay_step = 8;
  uint16_t vref_step = 16;
    
  init_single_sample();
  
  for (uint16_t delay = 0; delay < 1024; delay += delay_step){
    set_delay(delay);  
    uint16_t count = 0;
    for (uint16_t vref = 0; vref < 4096; vref += vref_step){
      set_DAC(vref);
      // wait for vref to settle
      // need 4.5 us delay for full steps; probably less for single-bit steps
      _delay(8*5); // 8 cycles per us        
      
      if (internal_clock_single_sample()){
        count++;
      }
    }
    send_ascii_int(delay);
    putchar(' ');
    send_ascii_int(count*vref_step);
    putchar('\n');
  }
}

void init()
{
  // setup ports
  // PORTA unused
  TRISA = 0b11110111;
  ANSELA = 0x00;
  WPUA = 0x00;
  ODCONA = 0x00;
  SLRCONA = 0x08;
  INLVLA = 0xff;
  
  // RB0: VREF-CS
  // RB1: VREF-SCK
  // RB2: VREF-SDI
  // RB3: COMPARATOR
  // RB4: COMPARATOR-CLK
  TRISB = 0b11111000;
  ANSELB = 0x00;
  WPUB = 0x00;
  ODCONB = 0x00;
  SLRCONB = 0x00;
  INLVLB = 0xff;
  
  // RC0: sample clock output
  // RC1: NC
  // RC2: NC
  // RC3: NC
  // RC4: status LED
  // RC5: LDACn
  // RC6: TX
  // RC7: RX
  TRISC = 0b10001110;
  ANSELC = 0x00;
  WPUC = 0x00;
  ODCONC = 0x00;
  SLRCONC = 0x00;
  INLVLC = 0xff;
  
  // RD0: D0
  // RD1: D1
  // RD2: D2
  // RD3: D3
  // RD4: D4
  // RD5: D5
  // RD6: D6
  // RD7: D7
  TRISD = 0x00;
  ANSELD = 0x00;
  WPUD = 0x00;
  ODCOND = 0x00;
  SLRCOND = 0x00;
  INLVLD = 0xff;
  
  // RE0: D8
  // RE1: D9
  // RE2: CLK-SEL
  TRISE = 0xf8;
  ANSELE = 0x00;
  WPUE = 0x00;
  ODCONE = 0x00;
  SLRCONE = 0x00;
  INLVLE = 0xff;
  
    // setup MSSP for SPI master
  RB1PPS = 0x17;  // SCK2
  RB2PPS = 0x18;  // SDO2
  //SSP2CON1bits.SSPM = 0b1010; // SPI master, use SSP2ADD for BRG
  //SSP2ADDbits.SSPADD = 0x4f; // 100 kHz SPI @ 32 MHz clock
  SSP2CON1bits.SSPM = 0b0000; // SPI master, 8 MHz @ 32 MHz clock
  SSP2CON1bits.SSPEN = 1; // enable MSSP
  SSP2CON1bits.CKP = 0; // idle clock low
  SSP2STATbits.CKE = 1; // transmit on active->idle transition
  LATBbits.LATB0 = 1; // SS high

  // setup EUSART
  RC6PPS = 0x0F; // TX1 on RC6
  RX1DTPPSbits.RX1DTPPS = 0x17; // RX1 on RC7
  RCSTA1bits.SPEN = 1;
  TXSTA1bits.SYNC = 0;
  TXSTA1bits.TXEN = 1;
  RCSTA1bits.CREN = 1;
  TXSTA1bits.BRGH = 1;
  BAUDCON1bits.BRG16 = 1;
  //SP1BRG = 68; // 115.2k @ 32 MHz clock
  //SP1BRG = 34; // 230.4k @ 32 MHz clock
  SP1BRG = 17; // 460.8k @ 32 MHz clock]
  //SP1BRG = 3; // 2457600 @ 32 MHz clock]

  return;

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
  
  // set LDACn low: DAC loads on rising CSn
  LATCbits.LATC5 = 0;
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

  //_delay(_XTAL_FREQ/4 - 2); // delay 1s
  _delay(_XTAL_FREQ/40 - 2); // delay 100ms
  //_delay(_XTAL_FREQ/400 - 2); // delay 10ms

  asm("BANKSEL PORTA");
  asm("BSF PORTA, 0"); // disable delay-line osc
  T0CON0bits.T0EN = 0; // stop timer0

  T1CONbits.TMR1ON = 0; // stop timer1 (DL osc already stopped by gate)

  return(((uint32_t)TMR1H << 28) | ((uint32_t)TMR1L << 20) |
         ((uint32_t)TMR0H << 12) | ((uint32_t)TMR0L <<  4) |
         (PORTC & 0x0f));
}


int16_t DAC_to_V(uint16_t counts)
{
  float voltage = 1.024f * (-2.f + 5.f * counts/4095.f); 
  return (int)(10000 * voltage);
}

void internal_SAR()
{
  uint16_t delay_step = 1;
  uint8_t oversample = 2;
 
  init_single_sample();

  while(1){
    for (uint16_t delay_idx=0; delay_idx<1024; delay_idx+=delay_step){
//    for (uint16_t delay_idx=200; delay_idx<350; delay_idx+=delay_step){
      set_delay(delay_idx);
      uint16_t idx = 0;
      uint16_t step = 2047;
      while (step > 0){
        set_DAC(idx+step);
        __delay_us(10);
        uint8_t count = 0;
        for (uint8_t i=0; i<oversample; i++){
          uint8_t sample = internal_clock_single_sample();
          count += sample;
        }
        if (count >= (oversample>>1)){
          idx += step;
        }
        step >>= 1;
      }
        
      send_ascii_int(delay_idx);
      putchar(' ');
      send_ascii_int(DAC_to_V(idx));
//      send_ascii_int(idx);
      putchar('\n');
    }
  }
}


void internal_mode()
{
  uint16_t dac_step = 1;
  uint16_t delay_step = 1;
 
  init_single_sample();

  while(1){
    for (uint16_t delay_idx=0; delay_idx<1024; delay_idx+=delay_step){
      set_delay(delay_idx);
      uint16_t count = 0;
      for (uint16_t i=0; i<4096; i+=dac_step){
        set_DAC(i);
        __delay_us(10);
        uint8_t sample = internal_clock_single_sample();
        count += sample;
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

void external_mode()
{
  select_clock_source(CS_EXTERNAL);

  uint16_t dac_step = 8;
  uint16_t delay_step = 8;
  uint8_t oversample = 1;

  while(1){
    for (uint16_t delay_idx=0; delay_idx<1024; delay_idx+=delay_step){
      set_delay(delay_idx);
      uint16_t count = 0;
      for (uint16_t i=0; i<4096; i+=dac_step){
        set_DAC(i);
        __delay_us(10);
        for (uint8_t j=0; j<(1<<oversample); j++){
          while(!PORTBbits.RB4);
          while(PORTBbits.RB4);
          count += PORTBbits.RB3;
        } 
      }
    
      send_ascii_int(delay_idx);
      putchar(' ');
      send_ascii_int(DAC_to_V((dac_step*count)>>oversample));
#define VISUAL
#ifdef VISUAL
      uint16_t len = 1+((dac_step*count/16)>>oversample);
      for (uint8_t i = 0; i < len; i++){
        putchar(' ');
      }
      putchar('*');
#endif
      putchar('\n');
    }
  }
}

void eye_diagram()
{
  select_clock_source(CS_EXTERNAL);

  uint16_t dac_step = 1;
  uint16_t delay_step = 1;
  uint8_t oversample = 1;

  LATAbits.LATA3 = 0;
  while(1){
  for (uint8_t pattern=0; pattern<127; pattern++){

    // advance pattern
    LATAbits.LATA3 = 1;
    LATAbits.LATA3 = 0;

    for (uint16_t delay_idx=0; delay_idx<1024; delay_idx+=delay_step){
      set_delay(delay_idx);
      uint16_t count = 0;
      for (uint16_t i=0; i<4096; i+=dac_step){
        set_DAC(i);
        __delay_us(10);
        for (uint8_t j=0; j<(1<<oversample); j++){
          while(!PORTBbits.RB4);
          while(PORTBbits.RB4);
          count += PORTBbits.RB3;
        } 
      }
 
      send_ascii_int(pattern);
      putchar(' ');
      send_ascii_int(delay_idx);
      putchar(' ');
      send_ascii_int(DAC_to_V((dac_step*count)>>oversample));
#define VISUAL
#ifdef VISUAL
      uint16_t len = 1+((dac_step*count/16)>>oversample);
      for (uint8_t i = 0; i < len; i++){
        putchar(' ');
      }
      putchar('*');
#endif
      putchar('\n');
    }
  }
  }
}

void eye_diagram_fast()
{
  select_clock_source(CS_EXTERNAL);

  static uint16_t foo[2];
  static uint16_t counts[257];

  LATAbits.LATA3 = 0;
  while(1){
  for (uint8_t pattern=0; pattern<127; pattern++){

    // advance pattern
    LATAbits.LATA3 = 1;
    LATAbits.LATA3 = 0;

    for (uint16_t base_idx=0; base_idx<1024; base_idx+=256){
      for (uint16_t j=0; j<256; j++){
        counts[j] = 0;
      }
      for (uint16_t i=0; i<4096; i++){
        set_LED(1);
        set_DAC(i);
        __delay_us(1000);
        set_LED(0);
        for (uint16_t delay_idx=0; delay_idx<256; delay_idx++){
          set_delay(base_idx + delay_idx);
          while(!PORTBbits.RB4);
          while(PORTBbits.RB4);
          counts[delay_idx] += PORTBbits.RB3; 
        }
      }
 
      for (uint16_t idx=0; idx<256; idx++){
        send_ascii_int(pattern);
        putchar(' ');
        send_ascii_int(base_idx+idx);
        putchar(' ');
        send_ascii_int(DAC_to_V(counts[idx]));
#define VISUAL
#ifdef VISUAL
        uint16_t len = 1+(counts[idx]/16);
        for (uint8_t i = 0; i < len; i++){
          putchar(' ');
        }
        putchar('*');
#endif
        putchar('\n');
      }
    }
  }
  }
}


void external_SAR_eye()
{
  uint16_t delay_step = 1;
  uint8_t oversample = 32;

  select_clock_source(CS_EXTERNAL);

  LATAbits.LATA3 = 0;
  uint16_t pattern = 0;
  while(1){
    // advance pattern
    LATAbits.LATA3 = 1;
    LATAbits.LATA3 = 0;
    pattern++;
    
    for (uint16_t delay_idx=0; delay_idx<1024; delay_idx+=delay_step){
      set_delay(delay_idx);
      uint16_t idx = 2047;
      uint16_t step = 1024;
      while (step > 0){
        set_DAC(idx);
        __delay_us(10);
        uint8_t count = 0;
        for (uint8_t i=0; i<oversample; i++){
          while(!PORTBbits.RB4);
          while(PORTBbits.RB4);
          count += PORTBbits.RB3; 
        }
        if (count >= (oversample>>1)){
          idx += step;
        } else {
          idx -= step;
        }
        step >>= 1;
      }

      send_ascii_int(pattern);
      putchar(' ');  
      send_ascii_int(delay_idx);
      putchar(' ');
      send_ascii_int(DAC_to_V(idx));
      putchar('\n');
    }
  }
}



void main(void) {
  init();
  for(uint16_t i=0; i<5000; i++){
    __delay_us(1000);
  }
  internal_SAR();
//  eye_diagram_fast();
//  external_SAR_eye();
  return;
}
