#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include<math.h>    //include math library

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

#define CS LATAbits.LATA0  //set chip select pin as A0
#define NUMPTS 200  //set number of points to plot per cycle
#define PI 3.14159265   //set PI to use for sine function
#define MAXLVL 256  //define maximum level of 2^8

static volatile char sineFunc[NUMPTS];
static volatile char triangleFunc[NUMPTS];

void setVoltage(char channel, char voltage);
void initSPI1();
char spi1_io(char write);
void makeSineFunction();
void makeTriangleFunction();

int main() {

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
    
    initSPI1();
    makeSineFunction();
    makeTriangleFunction();
    
    while(1) {
        while(PORTBbits.RB4){
            _CP0_SET_COUNT(0);
            while (_CP0_GET_COUNT()<12000){  // 0.5ms delay = 0.5ms*24MHz
            // use _CP0_SET_COUNT(0) and _CP0_GET_COUNT() to test the PIC timing
              // remember the core timer runs at half the CPU speed
            ;   //delay for 0.5ms
            }
            LATAbits.LATA4 = !LATAbits.LATA4;   //Invert pin RA4
        }
    }
}

void setVoltage(char channel, char voltage){
    if (channel==0){    //set VoutA
        CS = 0;
        spi1_io(0x70|((voltage & 0xF0)>>4));
        spi1_io((voltage & 0x0F)<<4);
        CS = 1;
    }
    else if (channel==1){   //set VoutB
        CS = 0;
        spi1_io(0xF0|((voltage & 0xF0)>>4));
        spi1_io((voltage & 0x0F)<<4);
        CS = 1;
    }
    else 
        ;
}

void initSPI1(){
    RPA0Rbits.RPA0R = 0b0000;   //set A0 as SS1 pin
    RPB8Rbits.RPB8R = 0b0011;   //set B8 as SDO pin
    
    TRISAbits.TRISA0 = 0;   //set pin A0 as output
    TRISBbits.TRISB8 = 0;   //set pin B8 as output
    
    CS = 1; //when command is beginning, clear CS to low and at ending, CS high.
    
    SPI1CON = 0;              // turn off the spi module and reset it
    SPI1BUF;                  // clear the rx buffer by reading from it
    SPI1BRG = 0x1;            // baud rate to 12 MHz [SPI4BRG = (48000000/(2*desired))-1]
    SPI1STATbits.SPIROV = 0;  // clear the overflow bit
    SPI1CONbits.CKE = 1;      // data changes when clock goes from hi to lo (since CKP is 0)
    SPI1CONbits.MSTEN = 1;    // master operation
    SPI1CONbits.ON = 1;       // turn on spi 1
}

//send a byte via SPI and return the response
char spi1_io(char write){
    SPI1BUF = write;    //write the byte to SPI1
    while(!SPI1STATbits.SPIRBF){
        ;   //do nothing while receiving the byte
    }
    return SPI1BUF;
}

void makeSineFunction(){
    int i=0;
    for (i-0; i<NUMPTS;i++){
        sineFunc[i] = MAXLVL*sin(2*PI*i/MAXLVL);
    }
}

void makeTriangleFunction(){
    int i=0;
    for (i-0; i<NUMPTS;i++){
        triangleFunc[i] = i*MAXLVL/NUMPTS;
    }
}