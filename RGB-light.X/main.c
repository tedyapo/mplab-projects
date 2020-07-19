/*
 * File:   main.c
 * Author: tyapo
 *
 * Created on June 16, 2020, 11:31 AM
 */

#include <xc.h>
#include <pic12f1572.h>
#include <stdint.h>

// PIC12F1572 Configuration Bit Settings
// CONFIG1
#pragma config FOSC = INTOSC    //  (INTOSC oscillator; I/O function on CLKIN pin)
#pragma config WDTE = OFF       // Watchdog Timer Enable (WDT disabled)
#pragma config PWRTE = ON       // Power-up Timer Enable (PWRT enabled)
#pragma config MCLRE = OFF      // MCLR Pin Function Select (MCLR/VPP pin function is digital input)
#pragma config CP = ON          // Flash Program Memory Code Protection (Program memory code protection is enabled)
#pragma config BOREN = OFF      // Brown-out Reset Enable (Brown-out Reset disabled)
#pragma config CLKOUTEN = ON    // Clock Out Enable (CLKOUT function is enabled on the CLKOUT pin)

// CONFIG2
#pragma config WRT = ALL        // Flash Memory Self-Write Protection (000h to 7FFh write protected, no addresses may be modified by EECON control)
#pragma config PLLEN = OFF      // PLL Enable (4x PLL disabled)
#pragma config STVREN = OFF     // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will not cause a Reset)
#pragma config BORV = HI        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), high trip point selected.)
#pragma config LPBOREN = ON     // Low Power Brown-out Reset enable bit (LPBOR is enabled)
#pragma config LVP = OFF        // Low-Voltage Programming Enable (High-voltage on MCLR/VPP must be used for programming)

#include <xc.h>


void setup()
{
  // GPIO pin assignemnts
  //
  // RA0 : TX
  // RA1 : RX
  // RA2 : Blue LED  (PWM3)
  // RA3 : NC
  // RA4 : Green LED (PWM2)
  // RA5 : Red LED   (PWM1)

  ANSELA     = 0b00000000; // all digital I/O
  LATA       = 0b00000000; // clear all
  TRISA      = 0b00000010; // RX is input; all else output 
  INLVLA     = 0b00000010; // RX is st input level
  nWPUEN     = 1;          // disable weak pull-ups

  // PWM1 on RA5, PWM2 on RA4, RX on RA1, TX on RA0
  APFCON = 0b00000011;
}

/*

[ ] add command to store color to HEF
[ ] read HEF on power-up/reset to get initial color
[ ] PWMs run colors in the background
[ ] polling or interrupts can be used for comms?
[ ] 

 */

// 8 bit address
enum
{
  ADDRESS_HOST = 0,
  ADDRESS_SELF = 1,
  ADDRESS_BROADCAST = 255
};

typedef struct
{
  uint8_t address;
  uint8_t command;
  union {
    uint8_t payload[7];
    struct {
      uint16_t red16;
      uint16_t green16;
      uint16_t blue16;
    };
    struct {
      uint8_t red8;
      uint8_t green8;
      uint8_t blue8;
    };    
  };
} packet_t;

// command code is upper 5 bits of command
#define COMMAND_CODE(x) ((x & 0xf8)>>3)
// payload length (bytes) is lower 3 bits of command
#define PAYLOAD_LEN(x) (x & 7)
#define COMMAND(code, len) ((code << 3) | len)

enum {COMMAND_NOP = 0,
      COMMAND_RESET,
      COMMAND_SLEEP,
      COMMAND_SET_BAUD,
      COMMAND_SET_MODE,
      COMMAND_RGB8,
      COMMAND_RGB16};

void transmit(char c)
{
  while(!TXIF) { /* spin */ }
  TXREG = c;
  asm("NOP");  
}

void send_packet(packet_t* packet)
{
  transmit(packet->address);
  transmit(packet->command);
  for (uint8_t i=0; i < PAYLOAD_LEN(packet->command); i++){
    transmit(packet->payload[i]);
  }
}

// !!! use better name; this is not getchar
// !!! on WDT timeout, should broadcast reset; also reset here
char receive()
{
  CLRWDT();
  while(!RCIF) { /* spin */ }
  if (FERR || OERR){
    // receive error: broadcast a reset, then reset self
    packet_t packet;
    packet.address = ADDRESS_BROADCAST;
    packet.command = COMMAND(COMMAND_RESET, 0);
    send_packet(&packet);
    RESET();
  }
  return RCREG;
}

void set_pwm8(uint8_t red, uint8_t green, uint8_t blue)
{
}

void set_pwm16(uint16_t red, uint16_t green, uint16_t blue)
{
}

// !! NB: check for invalid command/length combinations
void process_packet(packet_t* packet)
{
  switch(COMMAND_CODE(packet->command)){
  case COMMAND_NOP:
    break;
  case COMMAND_RESET:
    break;
  case COMMAND_SLEEP:
    break;
  case COMMAND_SET_BAUD:
    break;
  case COMMAND_SET_MODE:
    break;
  case COMMAND_RGB8:
    set_pwm8(packet->red8, packet->green8, packet->blue8);
    break;
  case COMMAND_RGB16:
    set_pwm16(packet->red16, packet->green16, packet->blue16);    
    break;
  default:
    // unknown command --> broadcast reset
    break;
  }
}

void mainloop()
{
  packet_t packet;

  // packet loop
  while(1){
    SWDTEN = 0; // disable WDT while waiting for packet to begin
    packet.address = receive();
    CLRWDT();
    SWDTEN = 1; // re-enable WDT to test for receive timeouts
  
    if (ADDRESS_SELF == packet.address){
      // packet for us: receive and execute; do not forward
      packet.command = receive();
      for (uint8_t i=0; i < PAYLOAD_LEN(packet.command); i++){
        packet.payload[i] = receive();
      }
      process_packet(&packet);
    } else if (ADDRESS_BROADCAST == packet.address ||
               ADDRESS_HOST == packet.address){
      // broadcast or host-bound packet: forward packet
      //                                 execute if broadcast
      transmit(packet.address);
      packet.command = receive();
      transmit(packet.command);
      for (uint8_t i=0; i < PAYLOAD_LEN(packet.command); i++){
        packet.payload[i] = receive();      
        transmit(packet.payload[i]);
      }
      if (ADDRESS_BROADCAST == packet.address){
        process_packet(&packet);
      }
    } else {
      // packet for someone else: decrement address and forward; do not execute
      packet.address--;
      transmit(packet.address);
      packet.command = receive();
      transmit(packet.command);
      for (uint8_t i=0; i < PAYLOAD_LEN(packet.command); i++){
        char c = receive();
        transmit(c);
      }    
    }
  }
}

void main()
{
  setup();
  mainloop();
}

#ifdef FOOBAR
------------------------------------
void start_pwm(uint8_t mode_idx)
{
  PWM1PR     = period_table[mode_idx].pwm_period;
  PWM2PR     = period_table[mode_idx].pwm_period;
  PWM3PR     = period_table[mode_idx].pwm_period;
  
  PWM1PH     = PHASE1;
  PWM2PH     = PHASE2;
  PWM3PH     = PHASE2;
  
  PWM1DC     = DUTY_CYCLE1;
  PWM2DC     = DUTY_CYCLE2;
  PWM3DC     = DUTY_CYCLE2;
    
  PWM1OF     = OFFSET_COUNT;
  PWMLD      = 0b00000011;
  PWM2CLKCON = 0b00000010;
  PWM1CLKCON = 0b00000010;

  PWM1OFCON  = 0b00000000;

  PWM1CON    = 0b11010000;
  PWM2CON    = 0b11000000;

}

void stop_pwm()
{
  APFCON     = 0b00000000;    // RA4, RA5 normal GPIO
}

// pulse quickly a number of times to make a bright blink
void pulse(uint8_t length)
{
  while (length--){
    LATA5 = 0;         // enable power to 74LVC1G123
    LATA4 = 1;         // trigger pulse
    LATA = 0b00100000; // disable power to 74LVC1G123, remove trigger
  }
}

int main(int argc, char** argv)
{
  // selected run-time mode index
  uint8_t mode_idx;

  mode_idx = read_mode_idx(); // get current mode from high-endurance flash
  setup();
  
  // enter run-time selection mode on power-on reset
  if (!nPOR){
    uint8_t cycle = N_CYCLES;

    mode_idx = DEFAULT_MODE_IDX;
    IOCAN      = 0b001000;      // flag negative RA3 edge detections

    do {
      IOCIE = 1; // enable IOC interrupts, but just wake from sleep (GIE=0)
      start_pwm(mode_idx);  // preview brightness
      WDTCONbits.WDTPS = 0b01100; // 4s timeout for brightness preview
      SLEEP();
      if (IOCAF3){
        IOCIE = 0; // disable IOC interrupts; don't wake during debounce
        WDTCONbits.WDTPS = 0b00110; // 64ms timeout for switch debounce
        SLEEP();    // debounce switch
        IOCAF3 = 0; // discard any switch bounce
        IOCIE = 1; // enable IOC interrupts for wake
        // switch must be released to count
        if (!RA3){
          mode_idx++;
          if (mode_idx == N_MODES){
            mode_idx = 0;
          }
          cycle = N_CYCLES;
        }
      } else {
        // blink out years of runtime
        cycle--;
        uint8_t blink_count;
        IOCIE = 0; // disable IOC interrupts; don't wake during blinks
        stop_pwm();
        WDTCONbits.WDTPS = 0b01001; // 512ms timeout for blinks
        SLEEP();
        for (blink_count = 0;
             blink_count < period_table[mode_idx].years;
             ++blink_count){
          pulse(N_PULSES);
          SLEEP();
        }
      }
    } while (cycle > 0);
    IOCIE = 0; // disable IOC interrupts
    IOCAN = 0b000000;      // disable negative RA3 edge detections
    write_mode_idx(mode_idx);   // store selected mode to high-endurance flash
  }

  // exit run-time selection mode
  nPOR = 1;                   // exit startup mode
  WDTCONbits.WDTPS = 0b10010; // 256s WDT timeout to wake and refresh RAM    
  start_pwm(mode_idx);
  SLEEP();                    // PWMs run in sleep, pulsing LED
  RESET();                    // on WDT timeout, reset processor to refresh RAM
}

#endif
