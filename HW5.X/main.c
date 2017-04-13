#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include"i2c_master_noint.h"    //include i2c header file

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

#define SLAVE_ADDR 0x27 //define slave address
#define GPIO_ADDR 0x09 //define address of GPIO register
#define PU_ADDR 0x06    //define address of GPIO pull up resistor register
#define IO_ADDR 0x00    //define address of IODIR

void initExpander();
void setExpander(char pin, char level);
char getExpander();

int main() {
    
    char stat = 0;
    
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
    
    initExpander();
    //IOCON.SEQOP = 0;
    
    __builtin_enable_interrupts();
    
    
    while(1) {
        
        stat = (getExpander() & 0x80) >> 7;  //stat=0 when button is pushed
        setExpander(0,stat);   //set pin GP0 with level of stat to set high when button is high
        
    }
}

void initExpander(){
    ANSELBbits.ANSB2 = 0;   //turn off B2 pin as analog input
    ANSELBbits.ANSB3 = 0;   //turn off B3 pin as analog input
    
    i2c_master_setup(); //set up i2c
    
    //Set GP0-3 as outputs, initially off and GP4-7 as inputs
    i2c_master_start(); //begin the start sequence
    i2c_master_send(SLAVE_ADDR <<1);    //send slave address and left shift by 1 to indicate write with LSB 0
    i2c_master_send(IO_ADDR);    //send register address to IODIR register
    i2c_master_send(0xF0);  //send data to set IO
    i2c_master_stop();
    
    //Set GP0-3 initially off and GP4-7 as pull-up high
    i2c_master_start(); //begin the start sequence
    i2c_master_send(SLAVE_ADDR <<1);    //send slave address and left shift by 1 to indicate write with LSB 0
    i2c_master_send(PU_ADDR);    //send register address to pull up resistor
    i2c_master_send(0xF0);  //send data
    i2c_master_stop();
}

void setExpander(char pin, char level){ //to set GP0 on expander high if GP7 is high
    
    unsigned char master_write = (level & 0x01) << pin; //put the level value where the pin is
    
    i2c_master_start(); //begin the start sequence
    i2c_master_send(SLAVE_ADDR << 1);    //send slave address and left shift by 1 to indicate write with LSB 0
    i2c_master_send(GPIO_ADDR);    //send register address
    i2c_master_send(master_write);
    i2c_master_stop();
    
}   

char getExpander(){ //to read GPIO address
    
    char master_read = 0x00;
    
    i2c_master_start(); //begin the start sequence
    i2c_master_send(SLAVE_ADDR << 1);    //send slave address and left shift by 1 to indicate write with LSB 0
    i2c_master_send(GPIO_ADDR);    //send register address
    i2c_master_restart();
    i2c_master_send(SLAVE_ADDR <<1 | 0x01);  //send slave address and left shift by 1 and or 1 to indicate read with LSB 1
    master_read = i2c_master_recv();
    i2c_master_ack(1);
    i2c_master_stop();
    
    return master_read;
}