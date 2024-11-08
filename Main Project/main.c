
//
// This file is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu.
//

// ----------------------------------------------------------------------------

#include <stdio.h>
#include "diag/Trace.h"
#include <string.h>
#include "stm32f0xx.h"
#include "cmsis/cmsis_device.h"
#include "cmsis/stm32f0xx.h"
#include "stm32f0xx_hal.h"

// ----------------------------------------------------------------------------
//
// STM32F0 led blink sample (trace via $(trace)).
//
// In debug configurations, demonstrate how to print a greeting message
// on the trace device. In release configurations the message is
// simply discarded.
//
// To demonstrate POSIX retargetting, reroute the STDOUT and STDERR to the
// trace device and display messages on both of them.
//
// Then demonstrates how to blink a led with 1Hz, using a
// continuous loop and SysTick delays.
//
// On DEBUG, the uptime in seconds is also displayed on the trace device.
//
// Trace support is enabled by adding the TRACE macro d efinition.
// By default the trace messages are forwarded to the $(trace) output,
// but can be rerouted to any device or completely suppressed, by
// changing the definitions required in system/src/diag/trace_impl.c
// (currently OS_USE_TRACE_ITM, OS_USE_TRACE_SEMIHOSTING_DEBUG/_STDOUT).
//
// The external clock frequency is specified as a preprocessor definition
// passed to the compiler via a command line option (see the 'C/C++ General' ->
// 'Paths and Symbols' -> the 'Symbols' tab, if you want to change it).
// The value selected during project creation was HSE_VALUE=48000000.
//
/// Note: The default clock settings take the user defined HSE_VALUE and try
// to reach the maximum possible system clock. For the default 8MHz input
// the result is guaranteed, but for other values it might not be possible,
// so please adjust the PLL settings in system/src/cmsis/system_stm32f0xx.c
//


// ----- main() ---------------------------------------------------------------

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"

/*Timer prescaler and period value presets*/

#define myTIM2_PRESCALER ((uint16_t)0x0000) //no prescaling
#define myTIM2_PERIOD ((uint32_t)0xFFFFFFFF) //max setting for overflow

#define myTIM3_PRESCALER (0xBB74) //47999 for 1ms prescaler
#define myTIM3_PERIOD (100) //10ms base value

/*Initialization Method definitions*/

void myGPIOA_Init(void);
void myGPIOB_Init(void);
void myTIM2_Init(void);
void myTIM3_Init(void);
void myEXTI_Init(void);
void myADC_Init(void);
void myDAC_Init(void);
void mySPI_Init(void);

/* Functional Method definitions*/

void oled_Write(unsigned char);
void oled_Write_Cmd(unsigned char);
void oled_Write_Data(unsigned char);
void perma_print(void);
void oled_config(void);
void refresh_OLED(void);
void ADC_reader(void); // for reading ADC values and setting DAC value
void wait(uint32_t wait_time); //Use tim3 to generate a delay

/*Global Variable definitions*/


unsigned int Freq = 0;  // measured period value
unsigned int Res = 0;   // measured resistance value

uint16_t edge_count = 0; //for measuring frequency of source
uint16_t input_line = 1; //to tell which line (555 or function) we are currently measuring
uint32_t POT_val = 0; //raw data from the ADC
uint16_t bufferindex = 0; //index to move through buffer array to print to screen
uint16_t characterindex = 0; //index to move through each string in buffer index
SPI_HandleTypeDef SPI_Handle;


//
// LED Display initialization commands
//
unsigned char oled_init_cmds[] =
{
    0xAE,
    0x20, 0x00,
    0x40,
    0xA0 | 0x01,
    0xA8, 0x40 - 1,
    0xC0 | 0x08,
    0xD3, 0x00,
    0xDA, 0x32,
    0xD5, 0x80,
    0xD9, 0x22,
    0xDB, 0x30,
    0x81, 0xFF,
    0xA4,
    0xA6,
    0xAD, 0x30,
    0x8D, 0x10,
    0xAE | 0x01,
    0xC0,
    0xA0
};


//
// Character specifications for LED Display (1 row = 8 bytes = 1 ASCII character)
// Example: to display '4', retrieve 8 data bytes stored in Characters[52][X] row
//          (where X = 0, 1, ..., 7) and send them one by one to LED Display.
// Row number = character ASCII code (e.g., ASCII code of '4' is 0x34 = 52)
//
unsigned char Characters[][8] = {
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b01011111, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // !
    {0b00000000, 0b00000111, 0b00000000, 0b00000111, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // "
    {0b00010100, 0b01111111, 0b00010100, 0b01111111, 0b00010100,0b00000000, 0b00000000, 0b00000000},  // #
    {0b00100100, 0b00101010, 0b01111111, 0b00101010, 0b00010010,0b00000000, 0b00000000, 0b00000000},  // $
    {0b00100011, 0b00010011, 0b00001000, 0b01100100, 0b01100010,0b00000000, 0b00000000, 0b00000000},  // %
    {0b00110110, 0b01001001, 0b01010101, 0b00100010, 0b01010000,0b00000000, 0b00000000, 0b00000000},  // &
    {0b00000000, 0b00000101, 0b00000011, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // '
    {0b00000000, 0b00011100, 0b00100010, 0b01000001, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // (
    {0b00000000, 0b01000001, 0b00100010, 0b00011100, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // )
    {0b00010100, 0b00001000, 0b00111110, 0b00001000, 0b00010100,0b00000000, 0b00000000, 0b00000000},  // *
    {0b00001000, 0b00001000, 0b00111110, 0b00001000, 0b00001000,0b00000000, 0b00000000, 0b00000000},  // +
    {0b00000000, 0b01010000, 0b00110000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // ,
    {0b00001000, 0b00001000, 0b00001000, 0b00001000, 0b00001000,0b00000000, 0b00000000, 0b00000000},  // -
    {0b00000000, 0b01100000, 0b01100000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // .
    {0b00100000, 0b00010000, 0b00001000, 0b00000100, 0b00000010,0b00000000, 0b00000000, 0b00000000},  // /
    {0b00111110, 0b01010001, 0b01001001, 0b01000101, 0b00111110,0b00000000, 0b00000000, 0b00000000},  // 0
    {0b00000000, 0b01000010, 0b01111111, 0b01000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // 1
    {0b01000010, 0b01100001, 0b01010001, 0b01001001, 0b01000110,0b00000000, 0b00000000, 0b00000000},  // 2
    {0b00100001, 0b01000001, 0b01000101, 0b01001011, 0b00110001,0b00000000, 0b00000000, 0b00000000},  // 3
    {0b00011000, 0b00010100, 0b00010010, 0b01111111, 0b00010000,0b00000000, 0b00000000, 0b00000000},  // 4
    {0b00100111, 0b01000101, 0b01000101, 0b01000101, 0b00111001,0b00000000, 0b00000000, 0b00000000},  // 5
    {0b00111100, 0b01001010, 0b01001001, 0b01001001, 0b00110000,0b00000000, 0b00000000, 0b00000000},  // 6
    {0b00000011, 0b00000001, 0b01110001, 0b00001001, 0b00000111,0b00000000, 0b00000000, 0b00000000},  // 7
    {0b00110110, 0b01001001, 0b01001001, 0b01001001, 0b00110110,0b00000000, 0b00000000, 0b00000000},  // 8
    {0b00000110, 0b01001001, 0b01001001, 0b00101001, 0b00011110,0b00000000, 0b00000000, 0b00000000},  // 9
    {0b00000000, 0b00110110, 0b00110110, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // :
    {0b00000000, 0b01010110, 0b00110110, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // ;
    {0b00001000, 0b00010100, 0b00100010, 0b01000001, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // <
    {0b00010100, 0b00010100, 0b00010100, 0b00010100, 0b00010100,0b00000000, 0b00000000, 0b00000000},  // =
    {0b00000000, 0b01000001, 0b00100010, 0b00010100, 0b00001000,0b00000000, 0b00000000, 0b00000000},  // >
    {0b00000010, 0b00000001, 0b01010001, 0b00001001, 0b00000110,0b00000000, 0b00000000, 0b00000000},  // ?
    {0b00110010, 0b01001001, 0b01111001, 0b01000001, 0b00111110,0b00000000, 0b00000000, 0b00000000},  // @
    {0b01111110, 0b00010001, 0b00010001, 0b00010001, 0b01111110,0b00000000, 0b00000000, 0b00000000},  // A
    {0b01111111, 0b01001001, 0b01001001, 0b01001001, 0b00110110,0b00000000, 0b00000000, 0b00000000},  // B
    {0b00111110, 0b01000001, 0b01000001, 0b01000001, 0b00100010,0b00000000, 0b00000000, 0b00000000},  // C
    {0b01111111, 0b01000001, 0b01000001, 0b00100010, 0b00011100,0b00000000, 0b00000000, 0b00000000},  // D
    {0b01111111, 0b01001001, 0b01001001, 0b01001001, 0b01000001,0b00000000, 0b00000000, 0b00000000},  // E
    {0b01111111, 0b00001001, 0b00001001, 0b00001001, 0b00000001,0b00000000, 0b00000000, 0b00000000},  // F
    {0b00111110, 0b01000001, 0b01001001, 0b01001001, 0b01111010,0b00000000, 0b00000000, 0b00000000},  // G
    {0b01111111, 0b00001000, 0b00001000, 0b00001000, 0b01111111,0b00000000, 0b00000000, 0b00000000},  // H
    {0b01000000, 0b01000001, 0b01111111, 0b01000001, 0b01000000,0b00000000, 0b00000000, 0b00000000},  // I
    {0b00100000, 0b01000000, 0b01000001, 0b00111111, 0b00000001,0b00000000, 0b00000000, 0b00000000},  // J
    {0b01111111, 0b00001000, 0b00010100, 0b00100010, 0b01000001,0b00000000, 0b00000000, 0b00000000},  // K
    {0b01111111, 0b01000000, 0b01000000, 0b01000000, 0b01000000,0b00000000, 0b00000000, 0b00000000},  // L
    {0b01111111, 0b00000010, 0b00001100, 0b00000010, 0b01111111,0b00000000, 0b00000000, 0b00000000},  // M
    {0b01111111, 0b00000100, 0b00001000, 0b00010000, 0b01111111,0b00000000, 0b00000000, 0b00000000},  // N
    {0b00111110, 0b01000001, 0b01000001, 0b01000001, 0b00111110,0b00000000, 0b00000000, 0b00000000},  // O
    {0b01111111, 0b00001001, 0b00001001, 0b00001001, 0b00000110,0b00000000, 0b00000000, 0b00000000},  // P
    {0b00111110, 0b01000001, 0b01010001, 0b00100001, 0b01011110,0b00000000, 0b00000000, 0b00000000},  // Q
    {0b01111111, 0b00001001, 0b00011001, 0b00101001, 0b01000110,0b00000000, 0b00000000, 0b00000000},  // R
    {0b01000110, 0b01001001, 0b01001001, 0b01001001, 0b00110001,0b00000000, 0b00000000, 0b00000000},  // S
    {0b00000001, 0b00000001, 0b01111111, 0b00000001, 0b00000001,0b00000000, 0b00000000, 0b00000000},  // T
    {0b00111111, 0b01000000, 0b01000000, 0b01000000, 0b00111111,0b00000000, 0b00000000, 0b00000000},  // U
    {0b00011111, 0b00100000, 0b01000000, 0b00100000, 0b00011111,0b00000000, 0b00000000, 0b00000000},  // V
    {0b00111111, 0b01000000, 0b00111000, 0b01000000, 0b00111111,0b00000000, 0b00000000, 0b00000000},  // W
    {0b01100011, 0b00010100, 0b00001000, 0b00010100, 0b01100011,0b00000000, 0b00000000, 0b00000000},  // X
    {0b00000111, 0b00001000, 0b01110000, 0b00001000, 0b00000111,0b00000000, 0b00000000, 0b00000000},  // Y
    {0b01100001, 0b01010001, 0b01001001, 0b01000101, 0b01000011,0b00000000, 0b00000000, 0b00000000},  // Z
    {0b01111111, 0b01000001, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // [
    {0b00010101, 0b00010110, 0b01111100, 0b00010110, 0b00010101,0b00000000, 0b00000000, 0b00000000},  // back slash
    {0b00000000, 0b00000000, 0b00000000, 0b01000001, 0b01111111,0b00000000, 0b00000000, 0b00000000},  // ]
    {0b00000100, 0b00000010, 0b00000001, 0b00000010, 0b00000100,0b00000000, 0b00000000, 0b00000000},  // ^
    {0b01000000, 0b01000000, 0b01000000, 0b01000000, 0b01000000,0b00000000, 0b00000000, 0b00000000},  // _
    {0b00000000, 0b00000001, 0b00000010, 0b00000100, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // `
    {0b00100000, 0b01010100, 0b01010100, 0b01010100, 0b01111000,0b00000000, 0b00000000, 0b00000000},  // a
    {0b01111111, 0b01001000, 0b01000100, 0b01000100, 0b00111000,0b00000000, 0b00000000, 0b00000000},  // b
    {0b00111000, 0b01000100, 0b01000100, 0b01000100, 0b00100000,0b00000000, 0b00000000, 0b00000000},  // c
    {0b00111000, 0b01000100, 0b01000100, 0b01001000, 0b01111111,0b00000000, 0b00000000, 0b00000000},  // d
    {0b00111000, 0b01010100, 0b01010100, 0b01010100, 0b00011000,0b00000000, 0b00000000, 0b00000000},  // e
    {0b00001000, 0b01111110, 0b00001001, 0b00000001, 0b00000010,0b00000000, 0b00000000, 0b00000000},  // f
    {0b00001100, 0b01010010, 0b01010010, 0b01010010, 0b00111110,0b00000000, 0b00000000, 0b00000000},  // g
    {0b01111111, 0b00001000, 0b00000100, 0b00000100, 0b01111000,0b00000000, 0b00000000, 0b00000000},  // h
    {0b00000000, 0b01000100, 0b01111101, 0b01000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // i
    {0b00100000, 0b01000000, 0b01000100, 0b00111101, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // j
    {0b01111111, 0b00010000, 0b00101000, 0b01000100, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // k
    {0b00000000, 0b01000001, 0b01111111, 0b01000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // l
    {0b01111100, 0b00000100, 0b00011000, 0b00000100, 0b01111000,0b00000000, 0b00000000, 0b00000000},  // m
    {0b01111100, 0b00001000, 0b00000100, 0b00000100, 0b01111000,0b00000000, 0b00000000, 0b00000000},  // n
    {0b00111000, 0b01000100, 0b01000100, 0b01000100, 0b00111000,0b00000000, 0b00000000, 0b00000000},  // o
    {0b01111100, 0b00010100, 0b00010100, 0b00010100, 0b00001000,0b00000000, 0b00000000, 0b00000000},  // p
    {0b00001000, 0b00010100, 0b00010100, 0b00011000, 0b01111100,0b00000000, 0b00000000, 0b00000000},  // q
    {0b01111100, 0b00001000, 0b00000100, 0b00000100, 0b00001000,0b00000000, 0b00000000, 0b00000000},  // r
    {0b01001000, 0b01010100, 0b01010100, 0b01010100, 0b00100000,0b00000000, 0b00000000, 0b00000000},  // s
    {0b00000100, 0b00111111, 0b01000100, 0b01000000, 0b00100000,0b00000000, 0b00000000, 0b00000000},  // t
    {0b00111100, 0b01000000, 0b01000000, 0b00100000, 0b01111100,0b00000000, 0b00000000, 0b00000000},  // u
    {0b00011100, 0b00100000, 0b01000000, 0b00100000, 0b00011100,0b00000000, 0b00000000, 0b00000000},  // v
    {0b00111100, 0b01000000, 0b00111000, 0b01000000, 0b00111100,0b00000000, 0b00000000, 0b00000000},  // w
    {0b01000100, 0b00101000, 0b00010000, 0b00101000, 0b01000100,0b00000000, 0b00000000, 0b00000000},  // x
    {0b00001100, 0b01010000, 0b01010000, 0b01010000, 0b00111100,0b00000000, 0b00000000, 0b00000000},  // y
    {0b01000100, 0b01100100, 0b01010100, 0b01001100, 0b01000100,0b00000000, 0b00000000, 0b00000000},  // z
    {0b00000000, 0b00001000, 0b00110110, 0b01000001, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // {
    {0b00000000, 0b00000000, 0b01111111, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // |
    {0b00000000, 0b01000001, 0b00110110, 0b00001000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // }
    {0b00001000, 0b00001000, 0b00101010, 0b00011100, 0b00001000,0b00000000, 0b00000000, 0b00000000},  // ~
    {0b00001000, 0b00011100, 0b00101010, 0b00001000, 0b00001000,0b00000000, 0b00000000, 0b00000000}   // <-
};


//function to set system clock speed to 48 MHz
void SystemClock48MHz( void )
{
    RCC->CR &= ~(RCC_CR_PLLON); // Disable the PLL

    while (( RCC->CR & RCC_CR_PLLRDY ) != 0 ); // Wait for the PLL to unlock

    RCC->CFGR = 0x00280000; // Configure the PLL for a 48MHz system clock

    RCC->CR |= RCC_CR_PLLON; // Enable the PLL

    while (( RCC->CR & RCC_CR_PLLRDY ) != RCC_CR_PLLRDY ); // Wait for the PLL to lock

    RCC->CFGR = ( RCC->CFGR & (~RCC_CFGR_SW_Msk)) | RCC_CFGR_SW_PLL; // Switch the processor to the PLL clock source

    SystemCoreClockUpdate(); // Update the system with the new clock frequency
}


//Main function
int main(int argc, char* argv[])
{

	SystemClock48MHz();

	myGPIOA_Init();		/* Initialize I/O port PA */
	myGPIOB_Init();		/* Initialize I/O port PB */
	myTIM2_Init();		/* Initialize timer TIM2 */
	myTIM3_Init();		/* Initialize timer TIM3 */
	myEXTI_Init();		/* Initialize EXTI */

    myADC_Init();       /* Initialize ADC*/
    myDAC_Init();       /* Initialize DAC*/

    mySPI_Init();       /* Initialize for SPI communications with OLED*/
    oled_config();      /*Reset OLED*/
    perma_print();      /*Print welcome message to OLED*/
    oled_config();      /*Reset OLEd again to begin printing res and freq*/

	while (1)
	{
		ADC_reader(); //continuously reading from the ADC to update DAC output
		refresh_OLED(); //continuously refreshing the OLED screen

	}
}


//Function to print to screen without being in refresh_oled which cycles in the main.
void perma_print( void )
{

    // Buffer size = at most 16 characters per PAGE + terminating '\0'
    unsigned char Buffer[17];
    unsigned char c; //to index through Buffer

    snprintf( Buffer, sizeof( Buffer ), "Hi Guoliang! :)");
    /* Buffer now contains your character ASCII codes for LED Display
       - select PAGE (LED Display line) and set starting SEG (column)
       - for each c = ASCII code = Buffer[0], Buffer[1], ...,
           send 8 bytes in Characters[c][0-7] to LED Display
    */
    oled_Write_Cmd(0xB0); //select the row on which we want to display this info
    oled_Write_Cmd(0x10); //select first segment
    oled_Write_Cmd(0x02); //select first segment

    bufferindex = 0;
    characterindex = 0;

    while(Buffer[bufferindex] != '\0') {
    	c = Buffer[bufferindex];
    	characterindex = 0;

        while(characterindex <= 7) {
        	oled_Write_Data(Characters[c][characterindex]); //Loads all values from the first index of the buffer then goes to next row
        	characterindex++;
        }
        bufferindex++;

    }
    wait(500);

     snprintf( Buffer, sizeof( Buffer ), "Presenting...");
     oled_Write_Cmd(0xB2); //select the row on which we want to display this info
     oled_Write_Cmd(0x10); //select first segment
     oled_Write_Cmd(0x02); //select first segment
     bufferindex = 0;
     characterindex = 0;

     while(Buffer[bufferindex] != '\0') {
     	c = Buffer[bufferindex];
     	characterindex = 0;

         while(characterindex <= 7) {
         	oled_Write_Data(Characters[c][characterindex]); //Loads all values from the first index of the buffer then goes to next row
         	characterindex++;
         }
         bufferindex++;

     }
     wait(500);

     snprintf( Buffer, sizeof( Buffer ), "ECE 355 Project");
      oled_Write_Cmd(0xB4); //select the row on which we want to display this info
      oled_Write_Cmd(0x10); //select first segment
      oled_Write_Cmd(0x02); //select first segment
      bufferindex = 0;
      characterindex = 0;

      while(Buffer[bufferindex] != '\0') {
      	c = Buffer[bufferindex];
      	characterindex = 0;

          while(characterindex <= 7) {
          	oled_Write_Data(Characters[c][characterindex]); //Loads all values from the first index of the buffer then goes to next row
          	characterindex++;
          }
          bufferindex++;

      }
      wait(500);

      snprintf( Buffer, sizeof( Buffer ), "Sophie & Menoa");
        oled_Write_Cmd(0xB6); //select the row on which we want to display this info
        oled_Write_Cmd(0x10); //select first segment
        oled_Write_Cmd(0x02); //select first segment
        bufferindex = 0;
        characterindex = 0;

        while(Buffer[bufferindex] != '\0') {
        	c = Buffer[bufferindex];
        	characterindex = 0;

            while(characterindex <= 7) {
            	oled_Write_Data(Characters[c][characterindex]); //Loads all values from the first index of the buffer then goes to next row
            	characterindex++;
            }
            bufferindex++;

        }

    wait(500);

}

//function that continuously is called in main to print current frequency and resistance values
void refresh_OLED( void )
{

    // Buffer size = at most 16 characters per PAGE + terminating '\0'
    unsigned char Buffer[17];
    unsigned char c; //to index through Buffer

    snprintf( Buffer, sizeof( Buffer ), "Res: %5u Ohms", Res );
    /* Buffer now contains your character ASCII codes for LED Display
       - select PAGE (LED Display line) and set starting SEG (column)
       - for each c = ASCII code = Buffer[0], Buffer[1], ...,
           send 8 bytes in Characters[c][0-7] to LED Display
    */
    oled_Write_Cmd(0xB2); //select the row on which we want to display this info
    oled_Write_Cmd(0x10); //select first segment
    oled_Write_Cmd(0x02); //select first segment

    bufferindex = 0;
    characterindex = 0;

    while(Buffer[bufferindex] != '\0') {
    	c = Buffer[bufferindex];
    	characterindex = 0;

        while(characterindex <= 7) {
        	oled_Write_Data(Characters[c][characterindex]); //Loads all values from the first index of the buffer then goes to next row
        	characterindex++;
        }
        bufferindex++;

    }

    snprintf( Buffer, sizeof( Buffer ), "Freq: %5u Hz", Freq ); //prints only when the buffer is full (sizeof(Buffer))

    oled_Write_Cmd(0xB4); //select the fourth page
    oled_Write_Cmd(0x10); //select first segment
    oled_Write_Cmd(0x02); //select first segment
    bufferindex = 0;
    characterindex = 0;

	while(Buffer[bufferindex] != '\0') {
		c = Buffer[bufferindex];
		characterindex = 0;

		while(characterindex <= 7) {
			oled_Write_Data(Characters[c][characterindex]); //Loads all values from the first index of the buffer then goes to next row
			characterindex++;
		}
		bufferindex++;
	}


    wait(100);


}

//Intialize general purpose input/output pins in port A
void myGPIOA_Init()
{
	/* Enable clock for GPIOA peripheral */
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

	/* Configure PA2 as input */
	GPIOA->MODER &= ~(GPIO_MODER_MODER2);
	GPIOA->MODER |= (GPIO_MODER_MODER5); //pin 5 as analog input

    /*Configure PA4 as analog output*/
    GPIOA->MODER |= GPIO_MODER_MODER4;

	/* Ensure no pull-up/pull-down for PA2, PA5, PA4 */
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR2);
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR5);//pin 5
    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR4);//pin 4
}

//Intialize general purpose input/output pins in port B
void myGPIOB_Init()
{
	/* Enable clock for GPIOB peripheral */
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

	/* Configure SPI ports for OLED */
	//Configure PB3 to connect to serial clock SCLK on OLED
    GPIOB->MODER &= ~(GPIO_MODER_MODER3); //clear the function mode bits for proper out/in selection
	GPIOB->MODER |= GPIO_MODER_MODER3_1; //set as alternate function, allowing SPI and screen control
    GPIOB->AFR[0] &= ~(GPIO_AFRL_AFSEL3);       // Reset AF selection bits
	GPIOB->AFR[0] |= (0x0 << GPIO_AFRL_AFSEL3_Pos); // Set AF0

	//Configure PB4 to connect to reset RES on OLED
    GPIOB->MODER &= ~(GPIO_MODER_MODER4); //clear the function mode bits for proper out/in selection
	GPIOB->MODER |= GPIO_MODER_MODER4_0; //general output mode

	//Configure PB5 for MOSI to OLED
    GPIOB->MODER &= ~(GPIO_MODER_MODER5); //clear the function mode bits for proper out/in selection
	GPIOB->MODER |= GPIO_MODER_MODER5_1; //set as alternate function, allowing SPI and screen control
    GPIOB->AFR[0] &= ~(GPIO_AFRL_AFSEL5);       // Reset AF selection bits
	GPIOB->AFR[0] |= (0x0 << GPIO_AFRL_AFSEL5_Pos); // Set AF0

	//Configure PB6 to connect to chip select CS on OLED
    GPIOB->MODER &= ~(GPIO_MODER_MODER6); //clear the function mode bits for proper out/in selection
	GPIOB->MODER |= GPIO_MODER_MODER6_0; //general output mode

	//Configure PB7 to connect to data command DC on OLED
    GPIOB->MODER &= ~(GPIO_MODER_MODER7); //clear the function mode bits for proper out/in selection
	GPIOB->MODER |= GPIO_MODER_MODER7_0; //general output mode

	/* Ensure no pull-up/pull-down*/
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR3);
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR4);
    GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR5);
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR6);
    GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR7);
}

//Initialization for timer 2
void myTIM2_Init()
{
	/* Enable clock for TIM2 peripheral */
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

	/* Configure TIM2: buffer auto-reload, count up, stop on overflow,
	 * enable update events, interrupt on overflow only */
	TIM2->CR1 = 0x008C;

	/* Set clock prescaler value */
	TIM2->PSC = myTIM2_PRESCALER;

	/* Set auto-reloaded delay */
	TIM2->ARR = myTIM2_PERIOD;

	/* Update timer registers */
	TIM2->EGR = 0x0001;

	/* Assign TIM2 interrupt priority = 0 in NVIC */
	NVIC_SetPriority(TIM2_IRQn, 0);

	/* Enable TIM2 interrupts in NVIC */
	NVIC_EnableIRQ(TIM2_IRQn);

	/* Enable update interrupt generation */
	TIM2->DIER |= TIM_DIER_UIE;

}

//Initialization for timer 3
void myTIM3_Init()
{
	/* Enable clock for TIM3 peripheral */
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

	/* Configure TIM3: buffer auto-reload, count up, stop on overflow,
	 * enable update events, interrupt on overflow only */
	TIM3->CR1 = 0x008C;

	/* Set clock prescaler value */
	TIM3->PSC = myTIM3_PRESCALER;
	/* Set auto-reloaded delay */
	TIM3->ARR = myTIM3_PERIOD;

	/* Update timer registers */
	TIM3->EGR = 0x0001;

	/* Assign TIM3 interrupt priority = 0 in NVIC */
	NVIC_SetPriority(TIM3_IRQn, 0);

	/* Enable TIM3 interrupts in NVIC */
	NVIC_EnableIRQ(TIM3_IRQn);

	/* Enable update interrupt generation */
	TIM3->DIER |= TIM_DIER_UIE;

}

//Initialization for external interrupts
void myEXTI_Init()
{
	/* Map EXTI2 line to PA2 */
	SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PA ; //To connect PA0 to EXTI0 (button)
	SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI1_PA ; //To connect PA1 to EXTI1
	SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI2_PA ; //Now connect PA2 (0x00) to EXTI2 (bits 8-11 of EXTICR)

	/* EXTI2 line interrupts: set rising-edge trigger */
	EXTI->RTSR |= EXTI_RTSR_TR0; //Set rising edge trigger for EXTI0
	EXTI->RTSR |= EXTI_RTSR_TR1; //Set rising edge trigger for EXTI1
	EXTI->RTSR |= EXTI_RTSR_TR2; //Set rising edge trigger for EXTI2

	/* Unmask interrupts from EXTI lines */
	EXTI->IMR |= EXTI_IMR_IM0;
	EXTI->IMR |= EXTI_IMR_IM1;
	EXTI->IMR |= EXTI_IMR_IM2;

	/* Assign EXTI2 interrupt priority = 0 in NVIC */
	NVIC_SetPriority(EXTI0_1_IRQn, 0); //Make sure EXTI0 is priority 0
	NVIC_SetPriority(EXTI2_3_IRQn, 1); //Make sure EXTI2 is priority 0

	/* Enable EXTI2 interrupts in NVIC */
	NVIC_EnableIRQ(EXTI0_1_IRQn); //enable EXTI0 interrupt line
	NVIC_EnableIRQ(EXTI2_3_IRQn); //enable EXTI2 interrupt line
}

//Initialization for internal ADC
void myADC_Init()
{
	RCC->APB2ENR |= RCC_APB2ENR_ADCEN; //Enable clock for the ADC1 on the board

	ADC1->CR = ADC_CR_ADCAL; // calibrate the ADC contol register

	while ((ADC1->CR & ADC_CR_ADCAL) != 0 ); //once done calibrating, the bit will be set back to 0. Ensure program waits for this

	ADC1->CR |= ADC_CR_ADEN; //enable the ADC

	ADC1->CHSELR |= ADC_CHSELR_CHSEL5; //set internal ADC to read from ADC channel 5 (PA5)

	ADC1->CFGR1 |= ADC_CFGR1_CONT; //set up the ADC for continuous sampling

	ADC1->CR |= ADC_CR_ADSTART; //Start group regular conversion

}

//Initialization for internal DAC
void myDAC_Init(){

    //Note that the built in DAC, when enabled is automatically connected to PA4.

    RCC->APB1ENR |= RCC_APB1ENR_DACEN;  //Enable the clock for the built in DAC

    DAC->CR |= DAC_CR_EN1; //Enable DAC channel

}

//Initialization for internal SPI (not including OLED configuration)
void mySPI_Init(void) {

	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN; //Enable the SPI clock

	//taken from SPI initialization example in interfacing slides

	SPI_Handle.Instance = SPI1;

	SPI_Handle.Init.Direction = SPI_DIRECTION_1LINE;

	SPI_Handle.Init.Mode = SPI_MODE_MASTER;

	SPI_Handle.Init.DataSize = SPI_DATASIZE_8BIT;

	SPI_Handle.Init.CLKPolarity = SPI_POLARITY_LOW;

	SPI_Handle.Init.CLKPhase = SPI_PHASE_1EDGE;

	SPI_Handle.Init.NSS = SPI_NSS_SOFT;

	SPI_Handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;

	SPI_Handle.Init.FirstBit = SPI_FIRSTBIT_MSB;

	SPI_Handle.Init.CRCPolynomial = 7;

	HAL_SPI_Init(&SPI_Handle); /* initialize SPI1 (CMSIS) */

	__HAL_SPI_ENABLE(&SPI_Handle); /* enable SPI1 (CMSIS) */

}

/* This handler is declared in system/src/cmsis/vectors_stm32f051x8.c */
void TIM2_IRQHandler()
{
	/* Check if update interrupt flag is indeed set */
	if ((TIM2->SR & TIM_SR_UIF) != 0)
	{
		trace_printf("\n*** Overflow2! ***\n");

		/* Clear update interrupt flag */
		// Relevant register: TIM2->SR
		TIM2->SR &= ~(TIM_SR_UIF);

		/* Restart stopped timer */
		// Relevant register: TIM2->CR1
		TIM2->CR1 |= TIM_CR1_CEN;
	}
}

/* This handler is declared in system/src/cmsis/vectors_stm32f051x8.c */
void TIM3_IRQHandler()
{
	/* Check if update interrupt flag is indeed set */
	if ((TIM3->SR & TIM_SR_UIF) != 0)
	{
		//trace_printf("\n*** Overflow3! ***\n");

		/* Clear update interrupt flag */
		// Relevant register: TIM3->SR
		TIM3->SR &= ~(TIM_SR_UIF);

		/* Restart stopped timer */
		// Relevant register: TIM3->CR1
		TIM3->CR1 |= TIM_CR1_CEN;
	}
}


/* This handler is declared in system/src/cmsis/vectors_stm32f051x8.c */
void EXTI2_3_IRQHandler()
{
	/* Check if EXTI2 interrupt pending flag is indeed set */
	if ((EXTI->PR & EXTI_PR_PR2) != 0)

	{
        //only want to manipulate values and measure if the input line is set to 2 currently (PA2 as input).
        if(input_line == 2){
            // 1. If this is the first edge:
            if (edge_count == 0) {
                //	- Clear count register (TIM2->CNT).
                TIM2->CNT &= 0b00000000;
                //	- Start timer (TIM2->CR1).
                TIM2->CR1 |= TIM_CR1_CEN;
                edge_count = 1;

            } else {
                //	- Stop timer (TIM2->CR1).
                TIM2->CR1 &= ~TIM_CR1_CEN;
                //	- Read out count register (TIM2->CNT).
                float periodd = (float)(TIM2->CNT) / (float)SystemCoreClock;
                //	- Calculate signal period and frequency.
                Freq = (unsigned int)((1)/(periodd)); //update frequency value
                edge_count = 0;

            }

        }
            // 2. Clear EXTI2 interrupt pending flag (EXTI->PR).
            // note we want to clear it no matter if the operations were performed or not
            EXTI->PR |= EXTI_PR_PR2;
	}
}

void EXTI0_1_IRQHandler()
{
	// Check if EXTI0 interrupt pending flag is indeed set (button pushed)
	if ((EXTI->PR & EXTI_PR_PR0) != 0) {

		if(input_line == 1) {
			input_line = 2;
            //clear the mask on EXTI2 line interrupts
        	EXTI->IMR |= EXTI_IMR_IM2;
		} else {
			input_line = 1;
            //mask the EXTI2 interrupt line
            EXTI->IMR &= ~(EXTI_IMR_IM2);
		}
		EXTI->PR |= EXTI_PR_PR0; //clear pending flag

	}
	// Check if EXTI1 interrupt pending flag is indeed set
	if ((EXTI->PR & EXTI_PR_PR1) != 0){

        //only want to manipulate values and measure if the input line is set to 1 currently (PA2 as input).
        if(input_line == 1){
            // 1. If this is the first edge:
            if (edge_count == 0) {
                //	- Clear count register (TIM2->CNT).
                TIM2->CNT &= 0b00000000;
                //	- Start timer (TIM2->CR1).
                TIM2->CR1 |= TIM_CR1_CEN;
                edge_count = 1;

            } else {
                //	- Stop timer (TIM2->CR1).
                TIM2->CR1 &= ~TIM_CR1_CEN;
                //	- Read out count register (TIM2->CNT).
                float periodd = (float)(TIM2->CNT) / (float)SystemCoreClock;
                //	- Calculate signal period and frequency.
                Freq = (unsigned int)((1)/(periodd)); //update frequency value
                edge_count = 0;

            }

        }

		EXTI->PR |= EXTI_PR_PR1; //clear pending flag even if measurement is not taken
	}
}

//function to read input values from potentiometer and set to output of DAC
void ADC_reader(){

	while ((ADC1->ISR & ADC_ISR_EOC) == 0){}; //wait until the end of conversion flag is set

	ADC1->ISR &= ~ADC_ISR_EOC; //Clear the end of conversion flag

    //We will want the potentiometer parameters to print to the screen, this processes and populated those variables

	POT_val = (ADC1->DR & ADC_DR_DATA);//read out the group conversion data to global variable ( & ADC_DR_DATA)

	Res = ((((float)POT_val)/((float)(0xFFF)))*5000); //position (resistance value)

    DAC->DHR12R1 = ADC1->DR; //write the ADC value to the DAC

}

//function to create a delay between commands
void wait(uint32_t wait_time){

    //	- Clear count register (TIM2->CNT).
    TIM3->CNT &= 0b00000000;

    TIM3->ARR = wait_time; //set timer counting period to the desired time

    TIM3->EGR |= TIM_EGR_UG; //reset the counter

    TIM3->CR1 |= TIM_CR1_CEN; //start timer

    while ((TIM3->SR & TIM_SR_UIF) == 0){}; //wait for update interrupt flag to be set (timer done)

    //	- Stop timer (TIM3->CR1).
    TIM3->CR1 &= ~TIM_CR1_CEN;
}

//Function to select area of screen currently writing to
void oled_Write_Cmd( unsigned char cmd )
{
    //... // make PB6 = CS# = 1
	GPIOB->BSRR = GPIO_BSRR_BS_6; //Chip select ensures that communication with the OLED is exclusive

    //... // make PB7 = D/C# = 0
	GPIOB->BSRR = GPIO_BSRR_BR_7; //Data/Command (0 command 1 data)

    //... // make PB6 = CS# = 0
	GPIOB->BSRR = GPIO_BSRR_BR_6;

    oled_Write( cmd );

    //... // make PB6 = CS# = 1
	GPIOB->BSRR = GPIO_BSRR_BS_6;
}

//Function to write data to currently selected section of screen
void oled_Write_Data( unsigned char data )
{
    //... // make PB6 = CS# = 1
	GPIOB->BSRR = GPIO_BSRR_BS_6;

    //... // make PB7 = D/C# = 1
	GPIOB->BSRR = GPIO_BSRR_BS_7;

    //... // make PB6 = CS# = 0
	GPIOB->BSRR = GPIO_BSRR_BR_6;

    oled_Write( data );

    //... // make PB6 = CS# = 1
	GPIOB->BSRR = GPIO_BSRR_BS_6;
}

//Function called to write to OLED
void oled_Write( unsigned char Value )
{

    /* Wait until SPI1 is ready for writing (TXE = 1 in SPI1_SR) */

	while ((SPI1->SR & SPI_SR_TXE) == 0){}; //wait until transmit buffer empty flag is set

    /* Send one 8-bit character:
       - This function also sets BIDIOE = 1 in SPI1_CR1
    */
	HAL_SPI_Transmit( &SPI_Handle, &Value, 1, HAL_MAX_DELAY ); //Header is there


    /* Wait until transmission is complete (TXE = 1 in SPI1_SR) */
	while ((SPI1->SR & SPI_SR_TXE) == 0){}; //wait until transmit buffer empty flag is set
}

//Function that resets OLEd screen and sets all of its values to 0 to prep for writing
void oled_config( void )
{

    // Reset LED Display (RES# = PB4):
    // make pin PB4 = 0, wait for a few ms
	GPIOB->BSRR = GPIO_BSRR_BR_4;
	wait(3);

    // make pin PB4 = 1, wait for a few ms
    GPIOB->BSRR = GPIO_BSRR_BS_4;
	wait(3);

    // Send initialization commands to LED Display
    for ( unsigned int i = 0; i < sizeof( oled_init_cmds ); i++ ) {
        oled_Write_Cmd( oled_init_cmds[i] );
    }


    /* Fill LED Display data memory (GDDRAM) with zeros:
       - for each PAGE = 0, 1, ..., 7
           set starting SEG = 0
           call oled_Write_Data( 0x00 ) 128 times
    */
    unsigned char page = 0xB0;
    oled_Write_Cmd(0x02);
    oled_Write_Cmd(0x10);

    for(int i = 0; i <= 7; i++){
    	oled_Write_Cmd(page);//select the page
    	oled_Write_Cmd(0x02); //select first segment
    	oled_Write_Cmd(0x10); //select first segment

    	for(int j = 0; j <=127; j++){
    		oled_Write_Data(0x00); //write 8-bit value to the display (clearing it)
    	}

    	page++; //increment to next page
    }

}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
