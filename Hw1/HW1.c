
/* 
 * File:   hw1main.c
 * Author: Lingyu
 *
 * Created on April 1, 2015, 11:30 PM
 */

#include <stdio.h>
#include <stdlib.h>

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



int readADC(void);
int val;
int main() {

    // startup
    __builtin_disable_interrupts();

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
    
    // set up USER pin as input
     ANSELBbits.ANSB13 = 0; // 0 for digital, 1 for analog
     TRISBbits.TRISB13 = 1; // 1 for input, 0 for output
     
    // set up LED1 pin as a digital output
     TRISBbits.TRISB7 = 0;
     LATBbits.LATB7 = 1;


    // set up LED2 as OC1 using Timer2 at 1kHz
     RPB15Rbits.RPB15R = 0b0101;
          ANSELBbits.ANSB15 = 0;


        PR2 = 39999;	//40mhz/PR2 equals frequency
        TMR2 = 0;
    	T2CONbits.TCKPS = 0;  //prescaler equals 256 at 0b111 or 1 at 0b000
    	T2CONbits.TGATE = 0;
    	T2CONbits.TCS = 0;

        OC1CONbits.OCTSEL = 0; //set to timer 2
        OC1CONbits.OCM = 0b110;  // PWM mode without fault pin; other OC1CON bits are defaults
        OC1RS = 20000;             // duty cycle = OC1RS/(PR2+1) = 25%
        OC1R = 20000;              // initialize before turning OC1 on; then it is read-only
        T2CONbits.ON = 1;        // turn on Timer2
        OC1CONbits.ON = 1;       // turn on OC1


    // set up A0 as AN0
    ANSELAbits.ANSA0 = 1;
    AD1CON3bits.ADCS = 3;
    AD1CHSbits.CH0SA = 0;
    AD1CON1bits.ADON = 1;

   LATBbits.LATB15 = 1;

    while (1) {

        _CP0_SET_COUNT(0); // set core timer to 0, remember it counts at half the CPU clock
            LATBINV = 0b10000000; // invert a pin
            // wait for half a second, setting LED brightness to pot angle while waiting
        while (_CP0_GET_COUNT() < 10000000) {
            val = readADC();
            OC1RS = val * 40000/1023;
            if (PORTBbits.RB13 == 1) {
            // nothing
            } else {
            LATBINV = 0b10000000;
        }
        }
        // invert pin every 0.5s, set PWM duty cycle % to the pot voltage output %
        
    }
}

int readADC(void) {
    int elapsed = 0;
    int finishtime = 0;
    int sampletime = 20;
    int a = 0;

    AD1CON1bits.SAMP = 1;
    elapsed = _CP0_GET_COUNT();
    finishtime = elapsed + sampletime;
    while (_CP0_GET_COUNT() < finishtime) {
    }
    AD1CON1bits.SAMP = 0;
    while (!AD1CON1bits.DONE) {
    }
    a = ADC1BUF0;
    return a;
}