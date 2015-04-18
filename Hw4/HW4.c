/* 
 * File:   i2c_main.c
 * Author: Lingyu
 *
 * Created on April 12, 2015, 12:33 AM
 */

#include <stdio.h>
#include <stdlib.h>

#include "i2c_display.h"
//#include "i2c_master_int.h"
/*
 *
 */
// DEVCFGs here

// These are the DEVCFG bits for the NU32 in standalone mode:
//#pragma config DEBUG = OFF          // Background Debugger disabled
//#pragma config FPLLMUL = MUL_20     // PLL Multiplier: Multiply by 20
//#pragma config FPLLIDIV = DIV_2     // PLL Input Divider:  Divide by 2
//#pragma config FPLLODIV = DIV_1     // PLL Output Divider: Divide by 1
//#pragma config FWDTEN = OFF         // WD timer: OFF
//#pragma config POSCMOD = HS         // Primary Oscillator Mode: High Speed xtal
//#pragma config FNOSC = PRIPLL       // Oscillator Selection: Primary oscillator w/ PLL
//#pragma config FPBDIV = DIV_1       // Peripheral Bus Clock: Divide by 1
//#pragma config BWP = OFF            // Boot write protect: OFF
//#pragma config ICESEL = ICS_PGx2    // ICE pins configured on PGx2, Boot write protect OFF.
//#pragma config FSOSCEN = OFF        // Disable second osc to get pins back
//#pragma config FSRSSEL = PRIORITY_7 // Shadow Register Set for interrupt priority 7


//These are the available DEVCFG bits for the PIC32MX250F128B are listed in the documentation that comes with XC32, in microchip/xc32/v1.33/docs/config_docs/32mx250f128b.html

// DEVCFG0
#pragma config DEBUG = OFF // no debugging
#pragma config JTAGEN = OFF // no jtag
#pragma config ICESEL = ICS_PGx1 // use PGED1 and PGEC1
#pragma config PWP = OFF // no write protect
#pragma config BWP = OFF // not boot write protect
#pragma config CP = OFF // no code protect

// DEVCFG1
#pragma config FNOSC = 011 // use primary oscillator with pll
#pragma config FSOSCEN = 0 // turn off secondary oscillator
#pragma config IESO = 0 // no switching clocks
#pragma config POSCMOD = 10 // high speed crystal mode
#pragma config OSCIOFNC = 0 // free up secondary osc pins
#pragma config FPBDIV = 00 // divide CPU freq by 1 for peripheral bus clock
#pragma config FCKSM = 11 // do not enable clock switch
#pragma config WDTPS = 10100 // slowest wdt
#pragma config WINDIS = 1 // no wdt window
#pragma config FWDTEN = 0 // wdt off by default
#pragma config FWDTWINSZ = 11 // wdt window at 25%


// DEVCFG2 - get the CPU clock to 40MHz
#pragma config FPLLIDIV = DIV_2 // divide input clock to be in range 4-5MHz
#pragma config FPLLMUL = MUL_20 // multiply clock after FPLLIDIV
#pragma config FPLLODIV = DIV_2 // divide clock after FPLLMUL to get 40MHz
#pragma config UPLLIDIV = DIV_2 // divide 8MHz input clock, then multiply by 12 to get 48MHz for USB
#pragma config UPLLEN = ON // USB clock onin

// DEVCFG3
#pragma config USERID = 0 // some 16bit userid
#pragma config PMDL1WAY = 1 // not multiple reconfiguration, check this
#pragma config IOL1WAY = 1 // not multimple reconfiguration, check this
#pragma config FUSBIDIO = 1 // USB pins controlled by USB module
#pragma config FVBUSONIO = 1 // controlled by USB module

#include<xc.h> // processor SFR definitions
#include<sys/attribs.h> // __ISR macro



//#define SIZE WIDTH*HEIGHT/8  //size in bytes
//int readADC(void);
//int val;
//static unsigned char video_buffer[SIZE+1]={0};  //bufer corresponding to display
//static unsigned char * gddram = video_buffer+1;  //video buffer start, write over i2c

// lookup table for all of the ascii characters
static const char ASCII[96][5] = {
 {0x00, 0x00, 0x00, 0x00, 0x00} // 20  (space)
,{0x00, 0x00, 0x5f, 0x00, 0x00} // 21 !
,{0x00, 0x07, 0x00, 0x07, 0x00} // 22 "
,{0x14, 0x7f, 0x14, 0x7f, 0x14} // 23 #
,{0x24, 0x2a, 0x7f, 0x2a, 0x12} // 24 $
,{0x23, 0x13, 0x08, 0x64, 0x62} // 25 %
,{0x36, 0x49, 0x55, 0x22, 0x50} // 26 &
,{0x00, 0x05, 0x03, 0x00, 0x00} // 27 '
,{0x00, 0x1c, 0x22, 0x41, 0x00} // 28 (
,{0x00, 0x41, 0x22, 0x1c, 0x00} // 29 )
,{0x14, 0x08, 0x3e, 0x08, 0x14} // 2a *
,{0x08, 0x08, 0x3e, 0x08, 0x08} // 2b +
,{0x00, 0x50, 0x30, 0x00, 0x00} // 2c ,
,{0x08, 0x08, 0x08, 0x08, 0x08} // 2d -
,{0x00, 0x60, 0x60, 0x00, 0x00} // 2e .
,{0x20, 0x10, 0x08, 0x04, 0x02} // 2f /
,{0x3e, 0x51, 0x49, 0x45, 0x3e} // 30 0
,{0x00, 0x42, 0x7f, 0x40, 0x00} // 31 1
,{0x42, 0x61, 0x51, 0x49, 0x46} // 32 2
,{0x21, 0x41, 0x45, 0x4b, 0x31} // 33 3
,{0x18, 0x14, 0x12, 0x7f, 0x10} // 34 4
,{0x27, 0x45, 0x45, 0x45, 0x39} // 35 5
,{0x3c, 0x4a, 0x49, 0x49, 0x30} // 36 6
,{0x01, 0x71, 0x09, 0x05, 0x03} // 37 7
,{0x36, 0x49, 0x49, 0x49, 0x36} // 38 8
,{0x06, 0x49, 0x49, 0x29, 0x1e} // 39 9
,{0x00, 0x36, 0x36, 0x00, 0x00} // 3a :
,{0x00, 0x56, 0x36, 0x00, 0x00} // 3b ;
,{0x08, 0x14, 0x22, 0x41, 0x00} // 3c <
,{0x14, 0x14, 0x14, 0x14, 0x14} // 3d =
,{0x00, 0x41, 0x22, 0x14, 0x08} // 3e >
,{0x02, 0x01, 0x51, 0x09, 0x06} // 3f ?
,{0x32, 0x49, 0x79, 0x41, 0x3e} // 40 @
,{0x7e, 0x11, 0x11, 0x11, 0x7e} // 41 A
,{0x7f, 0x49, 0x49, 0x49, 0x36} // 42 B
,{0x3e, 0x41, 0x41, 0x41, 0x22} // 43 C
,{0x7f, 0x41, 0x41, 0x22, 0x1c} // 44 D
,{0x7f, 0x49, 0x49, 0x49, 0x41} // 45 E
,{0x7f, 0x09, 0x09, 0x09, 0x01} // 46 F
,{0x3e, 0x41, 0x49, 0x49, 0x7a} // 47 G
,{0x7f, 0x08, 0x08, 0x08, 0x7f} // 48 H
,{0x00, 0x41, 0x7f, 0x41, 0x00} // 49 I
,{0x20, 0x40, 0x41, 0x3f, 0x01} // 4a J
,{0x7f, 0x08, 0x14, 0x22, 0x41} // 4b K
,{0x7f, 0x40, 0x40, 0x40, 0x40} // 4c L
,{0x7f, 0x02, 0x0c, 0x02, 0x7f} // 4d M
,{0x7f, 0x04, 0x08, 0x10, 0x7f} // 4e N
,{0x3e, 0x41, 0x41, 0x41, 0x3e} // 4f O
,{0x7f, 0x09, 0x09, 0x09, 0x06} // 50 P
,{0x3e, 0x41, 0x51, 0x21, 0x5e} // 51 Q
,{0x7f, 0x09, 0x19, 0x29, 0x46} // 52 R
,{0x46, 0x49, 0x49, 0x49, 0x31} // 53 S
,{0x01, 0x01, 0x7f, 0x01, 0x01} // 54 T
,{0x3f, 0x40, 0x40, 0x40, 0x3f} // 55 U
,{0x1f, 0x20, 0x40, 0x20, 0x1f} // 56 V
,{0x3f, 0x40, 0x38, 0x40, 0x3f} // 57 W
,{0x63, 0x14, 0x08, 0x14, 0x63} // 58 X
,{0x07, 0x08, 0x70, 0x08, 0x07} // 59 Y
,{0x61, 0x51, 0x49, 0x45, 0x43} // 5a Z
,{0x00, 0x7f, 0x41, 0x41, 0x00} // 5b [
,{0x02, 0x04, 0x08, 0x10, 0x20} // 5c ¥
,{0x00, 0x41, 0x41, 0x7f, 0x00} // 5d ]
,{0x04, 0x02, 0x01, 0x02, 0x04} // 5e ^
,{0x40, 0x40, 0x40, 0x40, 0x40} // 5f _
,{0x00, 0x01, 0x02, 0x04, 0x00} // 60 `
,{0x20, 0x54, 0x54, 0x54, 0x78} // 61 a
,{0x7f, 0x48, 0x44, 0x44, 0x38} // 62 b
,{0x38, 0x44, 0x44, 0x44, 0x20} // 63 c
,{0x38, 0x44, 0x44, 0x48, 0x7f} // 64 d
,{0x38, 0x54, 0x54, 0x54, 0x18} // 65 e
,{0x08, 0x7e, 0x09, 0x01, 0x02} // 66 f
,{0x0c, 0x52, 0x52, 0x52, 0x3e} // 67 g
,{0x7f, 0x08, 0x04, 0x04, 0x78} // 68 h
,{0x00, 0x44, 0x7d, 0x40, 0x00} // 69 i
,{0x20, 0x40, 0x44, 0x3d, 0x00} // 6a j
,{0x7f, 0x10, 0x28, 0x44, 0x00} // 6b k
,{0x00, 0x41, 0x7f, 0x40, 0x00} // 6c l
,{0x7c, 0x04, 0x18, 0x04, 0x78} // 6d m
,{0x7c, 0x08, 0x04, 0x04, 0x78} // 6e n
,{0x38, 0x44, 0x44, 0x44, 0x38} // 6f o
,{0x7c, 0x14, 0x14, 0x14, 0x08} // 70 p
,{0x08, 0x14, 0x14, 0x18, 0x7c} // 71 q
,{0x7c, 0x08, 0x04, 0x04, 0x08} // 72 r
,{0x48, 0x54, 0x54, 0x54, 0x20} // 73 s
,{0x04, 0x3f, 0x44, 0x40, 0x20} // 74 t
,{0x3c, 0x40, 0x40, 0x20, 0x7c} // 75 u
,{0x1c, 0x20, 0x40, 0x20, 0x1c} // 76 v
,{0x3c, 0x40, 0x30, 0x40, 0x3c} // 77 w
,{0x44, 0x28, 0x10, 0x28, 0x44} // 78 x
,{0x0c, 0x50, 0x50, 0x50, 0x3c} // 79 y
,{0x44, 0x64, 0x54, 0x4c, 0x44} // 7a z
,{0x00, 0x08, 0x36, 0x41, 0x00} // 7b {
,{0x00, 0x00, 0x7f, 0x00, 0x00} // 7c |
,{0x00, 0x41, 0x36, 0x08, 0x00} // 7d }
,{0x10, 0x08, 0x08, 0x10, 0x08} // 7e ?
,{0x00, 0x06, 0x09, 0x09, 0x06} // 7f ?
}; // end char ASCII[96][5]



int main() {

    // startup
    __builtin_disable_interrupts();
    
   // i2c_master_setup();

    // set the CP0 CONFIG register to indicate that
    // kseg0 is cacheable (0x3) or uncacheable (0x2)
    // see Chapter 2 "CPU for Devices with M4K Core"
    // of the PIC32 reference manual
    __builtin_mtc0(_CP0_CONFIG, _CP0_CONFIG_SELECT, 0xa4210583);

    // no cache on this chip!

    // 0 data RAM access wait states
    BMXCONbits.BMXWSDRM = 0x0;

    // enable multi vector interrupts
    INTCONbits.MVEC = 0x1;

    // disable JTAG to be able to use TDI, TDO, TCK, TMS as digital
    DDPCONbits.JTAGEN = 0;

    __builtin_enable_interrupts();

  

 
     TRISBbits.TRISB7 = 0;
     LATBbits.LATB7 = 1;
     
    display_init();
       
    

     int colstart=32;
     int rowstart=28;
     int coltracker=colstart;
     int rowtracker=rowstart;
     char message[20];
     sprintf(message,"Hello World 1337!");
     int length=strlen(message);
   int i;
   int j;
   int k=0;
   for (i=0; i<17; ++i){
       
   for (j=0; j<5; j++){
   //    if (i==5){
   //        display_draw();
   //        rowtracker=40;
   //        coltracker=28;

   //    }
   //    if (i==11){
   //        display_draw();
   //        rowtracker=50;
   //        coltracker=28;

   //    }
   //for (i=coltracker; i<coltracker+5; ++i){
   //    if ((message[i])==0){
   //        coltracker=colstart;
   //        rowtracker=rowtracker+9;
   //        
       
       if ((ASCII[message[i]-0x20][j] & 0b00000001)==0b00000001){
           display_pixel_set(rowtracker-7,coltracker,1);
       }
       if ((((ASCII[message[i]-0x20][j])>>1) & 0b00000001)==0b00000001){
           display_pixel_set(rowtracker-6,coltracker,1);
       }
       if ((((ASCII[message[i]-0x20][j])>>2) & 0b00000001)==0b00000001){
           display_pixel_set(rowtracker-5,coltracker,1);
       }
       if ((((ASCII[message[i]-0x20][j])>>3) & 0b00000001)==0b00000001){
           display_pixel_set(rowtracker-4,coltracker,1);
       }
       if ((((ASCII[message[i]-0x20][j])>>4) & 0b00000001)==0b00000001){
           display_pixel_set(rowtracker-3,coltracker,1);
       }
       if ((((ASCII[message[i]-0x20][j])>>5) & 0b00000001)==0b00000001){
           display_pixel_set(rowtracker-2,coltracker,1);
       }
       if ((((ASCII[message[i]-0x20][j])>>6) & 0b00000001)==0b00000001){
           display_pixel_set(rowtracker-1,coltracker,1);
       }
       if ((((ASCII[message[i]-0x20][j])>>7) & 0b00000001)==0b00000001){
           display_pixel_set(rowtracker,coltracker,1);
       }
       coltracker=coltracker+1;
      // }
   }
   }
   display_draw();
   /*
   coltracker=28;
   rowtracker=40;
   for (i=6; i<11; ++i){

   for (j=0; j<5; j++){
   //for (i=coltracker; i<coltracker+5; ++i){
   //    if ((message[i])==0){
   //        coltracker=colstart;
   //        rowtracker=rowtracker+9;
   //

       if ((ASCII[message[i]-0x20][j] & 0b00000001)==0b00000001){
           display_pixel_set(coltracker,rowtracker-7,1);
       }
       if ((((ASCII[message[i]-0x20][j])>>1) & 0b00000001)==0b00000001){
           display_pixel_set(coltracker,rowtracker-6,1);
       }
       if ((((ASCII[message[i]-0x20][j])>>2) & 0b00000001)==0b00000001){
           display_pixel_set(coltracker,rowtracker-5,1);
       }
       if ((((ASCII[message[i]-0x20][j])>>3) & 0b00000001)==0b00000001){
           display_pixel_set(coltracker,rowtracker-4,1);
       }
       if ((((ASCII[message[i]-0x20][j])>>4) & 0b00000001)==0b00000001){
           display_pixel_set(coltracker,rowtracker-3,1);
       }
       if ((((ASCII[message[i]-0x20][j])>>5) & 0b00000001)==0b00000001){
           display_pixel_set(coltracker,rowtracker-2,1);
       }
       if ((((ASCII[message[i]-0x20][j])>>6) & 0b00000001)==0b00000001){
           display_pixel_set(coltracker,rowtracker-1,1);
       }
       if ((((ASCII[message[i]-0x20][j])>>7) & 0b00000001)==0b00000001){
           display_pixel_set(coltracker,rowtracker,1);
       }
       coltracker=coltracker-1;
      // }
   }
   }
*/
   TRISBbits.TRISB7 = 0;
   LATBbits.LATB7 = 1;

   //display_draw();
   //display_pixel_set(16,30,)
   while (1){
       
   };

}

