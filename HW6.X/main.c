#include <xc.h>           // processor SFR definitions
#include <sys/attribs.h>  // __ISR macro
#include <math.h>    //include math library
#include <stdio.h>  //include standard io library
#include "ILI9163C.h"   //include LCD library

// DEVCFG0
#pragma config DEBUG = OFF // no debugging
#pragma config JTAGEN = OFF // no jtag
#pragma config ICESEL = ICS_PGx1 // use PGED1 and PGEC1
#pragma config PWP = OFF // no write protect
#pragma config BWP = OFF // no boot write protect
#pragma config CP = OFF // no code protect

// DEVCFG1
#pragma config FNOSC = PRIPLL // use primary oscillator with pll
#pragma config FSOSCEN = OFF // turn off secondary oscillator
#pragma config IESO = OFF // no switching clocks
#pragma config POSCMOD = HS // high speed crystal mode
#pragma config OSCIOFNC = OFF // free up secondary osc pins
#pragma config FPBDIV = DIV_1 // divide CPU freq by 1 for peripheral bus clock
#pragma config FCKSM = CSDCMD // do not enable clock switch
#pragma config WDTPS = PS1048576 // slowest wdt
#pragma config WINDIS = OFF // no wdt window
#pragma config FWDTEN = OFF // wdt off by default
#pragma config FWDTWINSZ = WINSZ_25 // wdt window at 25%

// DEVCFG2 - get the CPU clock to 48MHz
#pragma config FPLLIDIV = DIV_2 // divide input clock to be in range 4-5MHz (8MHz/2=4MHz)
#pragma config FPLLMUL = MUL_24 // multiply clock after FPLLIDIV (4MHz*24=96MHz)
#pragma config FPLLODIV = DIV_2 // divide clock after FPLLMUL to get 48MHz (96MHz/2=48MHz)
#pragma config UPLLIDIV = DIV_2 // divider for the 8MHz input clock, then multiply by 12 to get 48MHz for USB
#pragma config UPLLEN = ON // USB clock on

// DEVCFG3
#pragma config USERID = 0 // some 16bit userid, doesn't matter what
#pragma config PMDL1WAY = OFF // allow multiple reconfigurations
#pragma config IOL1WAY = OFF // allow multiple reconfigurations
#pragma config FUSBIDIO = ON // USB pins controlled by USB module
#pragma config FVBUSONIO = ON // USB BUSON controlled by USB module

#define CS LATBbits.LATB7  //set chip select pin as B7

void display_char(unsigned short x, unsigned short y, unsigned short color1, unsigned short color2, char c);

int main() {
    int counter = 0;
    __builtin_disable_interrupts();

    // set the CP0 CONFIG register to indicate that kseg0 is cacheable (0x3)
    __builtin_mtc0(_CP0_CONFIG, _CP0_CONFIG_SELECT, 0xa4210583);

    // 0 data RAM access wait states
    BMXCONbits.BMXWSDRM = 0x0;

    // enable multi vector interrupts
    INTCONbits.MVEC = 0x1;

    // disable JTAG to get pins back
    DDPCONbits.JTAGEN = 0;

    // do your TRIS and LAT commands here
    TRISBbits.TRISB4 = 1;   //Set pin RB4 as an input pin
    
    TRISAbits.TRISA4 = 0;   //Set pin RA4 as an output pin
    LATAbits.LATA4 = 1;     //Initialise LED to be HIGH
    
    __builtin_enable_interrupts();
    
    SPI1_init();
    LCD_init();
    LCD_clearScreen(ORANGERED);
    
    //int i=0, j=0;
    
    display_char(50, 50, RED, GREEN, 0x48);
    
    while(1) {
        /*for (i=0; i<128; i++){
            for (j=0; j<128; j++){
                LCD_drawPixel(i,j,BLUE);
                _CP0_SET_COUNT(0);
                while (_CP0_GET_COUNT()<240000){  // 10ms delay = 10ms*24MHz
                ;   //delay for 1ms
                }
            }
        }*/
    }
}

void display_char(unsigned short x, unsigned short y, unsigned short color1, unsigned short color2, char c){
    int i=0, j=0;
    unsigned short dot=0;
    for (i=0; i<5; i++){
        for (j=0; j<8; j++){
            dot = (ASCII[c-0x20][i] >> (7-j))&0x1;
            if (dot==1){
                LCD_drawPixel(x+i,y+j, color1);
            }
            else if (dot==0){
                LCD_drawPixel(x+i,y+j, color2);
            }
        }
    }
}