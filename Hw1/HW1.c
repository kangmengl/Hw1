
// DEVCFGs here

// These are the DEVCFG bits for the NU32 in standalone mode:
#pragma config DEBUG = OFF          // Background Debugger disabled
#pragma config FPLLMUL = MUL_20     // PLL Multiplier: Multiply by 20
#pragma config FPLLIDIV = DIV_2     // PLL Input Divider:  Divide by 2
#pragma config FPLLODIV = DIV_1     // PLL Output Divider: Divide by 1
#pragma config FWDTEN = OFF         // WD timer: OFF
#pragma config POSCMOD = HS         // Primary Oscillator Mode: High Speed xtal
#pragma config FNOSC = PRIPLL       // Oscillator Selection: Primary oscillator w/ PLL
#pragma config FPBDIV = DIV_1       // Peripheral Bus Clock: Divide by 1
#pragma config BWP = OFF            // Boot write protect: OFF
#pragma config ICESEL = ICS_PGx2    // ICE pins configured on PGx2, Boot write protect OFF.
#pragma config FSOSCEN = OFF        // Disable second osc to get pins back
#pragma config FSRSSEL = PRIORITY_7 // Shadow Register Set for interrupt priority 7


These are the available DEVCFG bits for the PIC32MX250F128B are listed in the documentation that comes with XC32, in microchip/xc32/v1.33/docs/config_docs/32mx250f128b.html

// DEVCFG0
#pragma config DEBUG = x // no debugging
#pragma config JTAGEN = x // no jtag
#pragma config ICESEL = x // use PGED1 and PGEC1
#pragma config PWP = x // no write protect
#pragma config BWP = x // not boot write protect
#pragma config CP = x // no code protect

// DEVCFG1
#pragma config FNOSC = x // use primary oscillator with pll
#pragma config FSOSCEN = x // turn off secondary oscillator
#pragma config IESO = x // no switching clocks
#pragma config POSCMOD = x // high speed crystal mode
#pragma config OSCIOFNC = x // free up secondary osc pins
#pragma config FPBDIV = x // divide CPU freq by 1 for peripheral bus clock
#pragma config FCKSM = x // do not enable clock switch
#pragma config WDTPS = x // slowest wdt
#pragma config WINDIS = x // no wdt window
#pragma config FWDTEN = x // wdt off by default
#pragma config FWDTWINSZ = x // wdt window at 25%

// DEVCFG2 - get the CPU clock to 40MHz
#pragma config FPLLIDIV = x // divide input clock to be in range 4-5MHz
#pragma config FPLLMUL = x // multiply clock after FPLLIDIV
#pragma config UPLLIDIV = x // divide clock after FPLLMUL
#pragma config UPLLEN = x // USB clock on
#pragma config FPLLODIV = x // divide clock by 1 to output on pin

// DEVCFG3
#pragma config USERID = 0 // some 16bit userid
#pragma config PMDL1WAY = x // not multiple reconfiguration, check this
#pragma config IOL1WAY = x // not multimple reconfiguration, check this
#pragma config FUSBIDIO = x // USB pins controlled by USB module
#pragma config FVBUSONIO = x // controlled by USB module

#include<xc.h> // processor SFR definitions
#include<sys/attribs.h> // __ISR macro



int readADC(void);

int main() {

    // startup

    // set up USER pin as input

    // set up LED1 pin as a digital output

    // set up LED2 as OC1 using Timer2 at 1kHz

    // set up A0 as AN0
    ANSELAbits.ANSA0 = 1;
    AD1CON3bits.ADCS = 3;
    AD1CHSbits.CH0SA = 0;
    AD1CON1bits.ADON = 1;

    while (1) {
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