#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include"i2c_master_noint.h"    //include i2c header file
#include"ILI9163C.h"    //include LCD header file

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

#define SLAVE_ADDR 0x6B //define slave address
#define CTRL1_ADDR 0x10 //define address of CTRL_1 register
#define OUT_TEMP_L_ADDR 0x20   //define address of OUT_TEMP_L register
#define OUT_TEMP_L_ADDR 0x28    //define address of OUTX_L_XL register
#define WHO_AM_I_ADDR 0x0F  //define address of WHO_AM_I register

void initLMS();
void setExpander(char pin, char level);
void I2C_read_multiple(unsigned char slave_address, unsigned char register, unsigned char * data, int length);


void display_char(unsigned short x, unsigned short y, unsigned short color1, unsigned short color2, char c);    //display character
void display_string(unsigned short xpos, unsigned short ypos, char* msg);   //display string
void draw_bar(unsigned short x1, unsigned short y1, unsigned short w, unsigned short colorA, unsigned short colorB, unsigned short len);    //update progress bar

int main() {
    
    unsigned char data[14];
    signed short data_array[7];
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
    
    initLMS();
    //IOCON.SEQOP = 0;
    
    __builtin_enable_interrupts();
    
    
    while(1) {
        I2C_read_multiple(SLAVE_ADDR, OUT_TEMP_L_ADDR, data, 14);
        for (i=0; i<7; i++){
            data_array[i]=(data[2*i]<<8)|data[2*i+1];
        }
    }
}

void initLMS(){
    ANSELBbits.ANSB2 = 0;   //turn off B2 pin as analog input
    ANSELBbits.ANSB3 = 0;   //turn off B3 pin as analog input
    
    i2c_master_setup(); //set up i2c
    
    //Set CTRL values by sequential write
    i2c_master_start(); //begin the start sequence
    i2c_master_send(SLAVE_ADDR <<1);    //send slave address and left shift by 1 to indicate write with LSB 0
    i2c_master_send(CTRL1_ADDR);    //send register address to CTRL register
    i2c_master_send(0x82);  //send data to set CTRL1
    i2c_master_send(0x88);  //send data to set CTRL2
    i2c_master_send(0x04);  //send data to set CTRL3
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

void I2C_read_multiple(unsigned char slave_address, unsigned char reg, unsigned char * data, int length){ //to read values from accelerometer address
    
    i2c_master_start(); //begin the start sequence
    i2c_master_send(slave_address << 1);    //send slave address and left shift by 1 to indicate write with LSB 0
    i2c_master_send(reg);    //send register address
    i2c_master_restart();
    i2c_master_send(slave_address <<1 | 0x01);  //send slave address and left shift by 1 and or 1 to indicate read with LSB 1
    for (i=0;i<length;i++){
    data[i] = i2c_master_recv();
    i2c_master_ack(0);
    }
    i2c_master_ack(1);
    i2c_master_stop();

}

void display_char(unsigned short x, unsigned short y, unsigned short color1, unsigned short color2, char c){
    int i=0, j=0;
    char text[50];
    float fps = 0;
    unsigned short dot=0;
    for (i=0; i<5; i++){
        for (j=0; j<8; j++){
            if (x+i<128 && y+j<128){
                dot = (ASCII[c-0x20][i] >> j)&0x1;
                if (dot==1){
                    LCD_drawPixel(x+i,y+j, color1);
                }
                else if (dot==0){
                    LCD_drawPixel(x+i,y+j, color2);
                }
                /*fps = 24000000/((i+1)*(j+1)*_CP0_GET_COUNT());
                sprintf(text, "FPS = %4.2f   ", fps);
                display_string(28, 70, text);*/
            }
        }
    }
}

void display_string(unsigned short xpos, unsigned short ypos, char* msg){
    int counter = 0;
    while (msg[counter]){
        display_char(xpos+5*counter, ypos, TEXT, BACKGROUND, msg[counter]);
        counter++;
    }
}

void draw_bar(unsigned short x1, unsigned short y1, unsigned short w, unsigned short colorA, unsigned short colorB, unsigned short len){
    int xdir = 0, ydir=0;
    for (xdir=0; xdir<len; xdir++){
        for (ydir=0; ydir<w; ydir++){
            LCD_drawPixel(x1+xdir, y1+ydir, colorA);
        }
    }
    for (xdir=len; xdir<BAR_LENGTH; xdir++){
        for (ydir=0; ydir<w; ydir++){
            LCD_drawPixel(x1+xdir, y1+ydir, colorB);
        }
    }
}