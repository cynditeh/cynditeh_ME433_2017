/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "app.h"
#include<stdio.h>   //standard input output library
#include"i2c_master_noint.h"    //include i2c header file
#include"ILI9163C.h"    //include LCD header file


// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
 */

APP_DATA appData;
#define SLAVE_ADDR 0x6B //define slave address
#define CTRL1_ADDR 0x10 //define address of CTRL_1 register
#define CTRL2_ADDR 0x11 //define address of CTRL_1 register
#define CTRL3_ADDR 0x12 //define address of CTRL_1 register
#define OUT_TEMP_L_ADDR 0x20   //define address of OUT_TEMP_L register
#define OUTX_L_XL_ADDR 0x28    //define address of OUTX_L_XL register
#define WHO_AM_I_ADDR 0x0F  //define address of WHO_AM_I register

#define CS LATBbits.LATB7  //set chip select pin as B7
#define BAR_LENGTH 50  //set length of progress bar
#define CAL 500 //set scaling of value
#define BAR_WIDTH 2     //set width of progress bar
#define BACKGROUND ORANGE
#define TEXT BLACK
// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
 */

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

void initIMU() {
    ANSELBbits.ANSB2 = 0; //turn off B2 pin as analog input
    ANSELBbits.ANSB3 = 0; //turn off B3 pin as analog input

    i2c_master_setup(); //set up i2c

    //Set CTRL values by sequential write
    i2c_master_start(); //begin the start sequence
    i2c_master_send(SLAVE_ADDR << 1); //send slave address and left shift by 1 to indicate write with LSB 0
    i2c_master_send(CTRL1_ADDR); //send register address to CTRL1 register
    i2c_master_send(0x82); //send data to set CTRL1
    i2c_master_send(0x88); //send data to set CTRL2
    i2c_master_send(0x04); //send data to set CTRL3
    i2c_master_stop();
}

char whoAmI() {
    char who = 0;
    i2c_master_start(); //begin the start sequence
    i2c_master_send(SLAVE_ADDR << 1); //send slave address and left shift by 1 to indicate write with LSB 0
    i2c_master_send(WHO_AM_I_ADDR); //send WHO_AM_I register address
    i2c_master_restart();
    i2c_master_send(SLAVE_ADDR << 1 | 0x01); //send slave address and left shift by 1 and or 1 to indicate read with LSB 1
    who = i2c_master_recv(); //receive WHO_AM_I value
    i2c_master_ack(1);
    i2c_master_stop();

    return who;
}

void I2C_read_multiple(unsigned char slave_address, unsigned char reg, unsigned char * data, int length) { //to read values from accelerometer address
    int i = 0;
    i2c_master_start(); //begin the start sequence
    i2c_master_send(slave_address << 1); //send slave address and left shift by 1 to indicate write with LSB 0
    i2c_master_send(reg); //send register address
    i2c_master_restart();
    i2c_master_send(slave_address << 1 | 0x01); //send slave address and left shift by 1 and or 1 to indicate read with LSB 1
    for (i = 0; i < length - 1; i++) { //loop for number of data to read
        data[i] = i2c_master_recv();
        i2c_master_ack(0);
    }
    data[length - 1] = i2c_master_recv();
    i2c_master_ack(1);
    i2c_master_stop();
}

void display_char(unsigned short x, unsigned short y, unsigned short color1, unsigned short color2, char c) {
    int i = 0, j = 0;
    unsigned short dot = 0;
    for (i = 0; i < 5; i++) {
        for (j = 0; j < 8; j++) {
            if (x + i < 128 && y + j < 128) {
                dot = (ASCII[c - 0x20][i] >> j)&0x1;
                if (dot == 1) {
                    LCD_drawPixel(x + i, y + j, color1);
                } else if (dot == 0) {
                    LCD_drawPixel(x + i, y + j, color2);
                }
            }
        }
    }
}

void display_string(unsigned short xpos, unsigned short ypos, char* msg) {
    int counter = 0;
    while (msg[counter]) {
        display_char(xpos + 5 * counter, ypos, TEXT, BACKGROUND, msg[counter]);
        counter++;
    }
}

void draw_barX(unsigned short x1, unsigned short y1, unsigned short w, unsigned short colorA, unsigned short colorB, signed short len) {
    signed short xdirX = 0, ydirX = 0;
    if (len >= 0) { //check if direction is positive
        if ((len / CAL) < BAR_LENGTH) {
            for (xdirX = 0; xdirX < (len / CAL); xdirX++) { //print colorA for len
                for (ydirX = 0; ydirX < w; ydirX++) {
                    LCD_drawPixel(x1 - xdirX, y1 - ydirX, colorA);
                }
            }
            for (xdirX = (len / CAL); xdirX < BAR_LENGTH; xdirX++) { //print colorB for len to BAR_LENGTH
                for (ydirX = 0; ydirX < w; ydirX++) {
                    LCD_drawPixel(x1 - xdirX, y1 - ydirX, colorB);
                }
            }
            for (xdirX = 0; xdirX > (-BAR_LENGTH); xdirX--) { //print colorB for opposite direction
                for (ydirX = 0; ydirX < w; ydirX++) {
                    LCD_drawPixel(x1 - xdirX, y1 - ydirX, colorB);
                }
            }
        } else {
            for (xdirX = 0; xdirX < BAR_LENGTH; xdirX++) { //print full bar if out of range
                for (ydirX = 0; ydirX < w; ydirX++) {
                    LCD_drawPixel(x1 - xdirX, y1 - ydirX, colorA);
                }
            }
        }
    } else if (len < 0) { //case if negative length
        if ((len / CAL)>(-BAR_LENGTH)) { //check if range within BAR_LENGTH
            for (xdirX = 0; xdirX > (len / CAL); xdirX--) { //print colorA for len
                for (ydirX = 0; ydirX < w; ydirX++) {
                    LCD_drawPixel(x1 - xdirX, y1 - ydirX, colorA);
                }
            }
            for (xdirX = len / CAL; xdirX > (-BAR_LENGTH); xdirX--) { //print colorB for len to BAR_LENGTH
                for (ydirX = 0; ydirX < w; ydirX++) {
                    LCD_drawPixel(x1 - xdirX, y1 - ydirX, colorB);
                }
            }
            for (xdirX = 0; xdirX < BAR_LENGTH; xdirX++) { //print colorB for opposite direction
                for (ydirX = 0; ydirX < w; ydirX++) {
                    LCD_drawPixel(x1 - xdirX, y1 - ydirX, colorB);
                }
            }
        } else {
            for (xdirX = 0; xdirX > (-BAR_LENGTH); xdirX--) { //print full bar if out of range
                for (ydirX = 0; ydirX < w; ydirX++) {
                    LCD_drawPixel(x1 - xdirX, y1 - ydirX, colorA);
                }
            }
        }
    }
}

void draw_barY(unsigned short x2, unsigned short y2, unsigned short w, unsigned short colorA, unsigned short colorB, signed short len) {
    signed short xdirY = 0, ydirY = 0;
    if (len >= 0) {
        if ((len / CAL) < BAR_LENGTH) {
            for (ydirY = 0; ydirY < (len / CAL); ydirY++) {
                for (xdirY = 0; xdirY < w; xdirY++) {
                    LCD_drawPixel(x2 - xdirY, y2 - ydirY, colorA);
                }
            }
            for (ydirY = (len / CAL); ydirY < BAR_LENGTH; ydirY++) {
                for (xdirY = 0; xdirY < w; xdirY++) {
                    LCD_drawPixel(x2 - xdirY, y2 - ydirY, colorB);
                }
            }
            for (ydirY = 0; ydirY > (-BAR_LENGTH); ydirY--) {
                for (xdirY = 0; xdirY < w; xdirY++) {
                    LCD_drawPixel(x2 - xdirY, y2 - ydirY, colorB);
                }
            }
        } else {
            for (ydirY = 0; ydirY < BAR_LENGTH; ydirY++) {
                for (xdirY = 0; xdirY < w; xdirY++) {
                    LCD_drawPixel(x2 - xdirY, y2 - ydirY, colorA);
                }
            }
        }
    } else if (len < 0) {
        if ((len / CAL)>(-BAR_LENGTH)) {
            for (ydirY = 0; ydirY > (len / CAL); ydirY--) {
                for (xdirY = 0; xdirY < w; xdirY++) {
                    LCD_drawPixel(x2 - xdirY, y2 - ydirY, colorA);
                }
            }
            for (ydirY = len / CAL; ydirY > (-BAR_LENGTH); ydirY--) {
                for (xdirY = 0; xdirY < w; xdirY++) {
                    LCD_drawPixel(x2 - xdirY, y2 - ydirY, colorB);
                }
            }
            for (ydirY = 0; ydirY < BAR_LENGTH; ydirY++) {
                for (xdirY = 0; xdirY < w; xdirY++) {
                    LCD_drawPixel(x2 - xdirY, y2 - ydirY, colorB);
                }
            }
        } else {
            for (ydirY = 0; ydirY > (-BAR_LENGTH); ydirY--) {
                for (xdirY = 0; xdirY < w; xdirY++) {
                    LCD_drawPixel(x2 - xdirY, y2 - ydirY, colorA);
                }
            }
        }
    }
}

/* TODO:  Add any necessary local functions.
 */


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Initialize(void) {
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;


    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
}

/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks(void) {

    /* Check the application's current state. */
    switch (appData.state) {
            /* Application's initial state. */
        case APP_STATE_INIT:
        {
            bool appInitialized = true;


            if (appInitialized) {
                // do your TRIS and LAT commands here
                TRISBbits.TRISB4 = 1; //Set pin RB4 as an input pin

                TRISAbits.TRISA4 = 0; //Set pin RA4 as an output pin
                LATAbits.LATA4 = 1; //Initialise LED to be HIGH
                SPI1_init();
                LCD_init();
                LCD_clearScreen(BACKGROUND);
                initIMU();
                appData.state = APP_STATE_SERVICE_TASKS;
            }
            break;
        }

        case APP_STATE_SERVICE_TASKS:
        {
            unsigned char data[14];
            char message[20];
            signed short data_array[7];
            int i = 0, k = 0;

            while (1) {
                /*while (PORTBbits.RB4) {
                    _CP0_SET_COUNT(0);
                    while (_CP0_GET_COUNT() < 12000) { // 0.5ms delay = 0.5ms*24MHz
                        // use _CP0_SET_COUNT(0) and _CP0_GET_COUNT() to test the PIC timing
                        // remember the core timer runs at half the CPU speed
                        ; //delay for 0.5ms
                    }
                    LATAbits.LATA4 = !LATAbits.LATA4; //Invert pin RA4
                }*/

                //sprintf(message, "WHO AM I: %d!   ", whoAmI());
                //display_string(28, 10, message);
                _CP0_SET_COUNT(0);
                I2C_read_multiple(SLAVE_ADDR, OUTX_L_XL_ADDR, data, 6);
                for (i = 0; i < 3; i++) {
                    data_array[i] = (data[2 * i + 1] << 8) | data[2 * i];
                }
                //        
                //        for (k=0; k<2; k++){
                //        sprintf(message, "Data %d = %d   ", k ,data_array[k]);
                //        display_string(28, 2+9*k, message);
                //        }
                sprintf(message, "X_ACCEL = %d   ", data_array[0]);
                display_string(28, 2, message); //display x direction acceleration
                sprintf(message, "Y_ACCEL = %d   ", data_array[1]);
                display_string(28, 12, message); //display y direction acceleration

                draw_barX(63, 73, BAR_WIDTH, BLUE, BACKGROUND, data_array[0]); //display x direction bar
                draw_barY(64, 73, BAR_WIDTH, BLUE, BACKGROUND, data_array[1]); //display y direction bar

                /*I2C_read_multiple(SLAVE_ADDR, OUT_TEMP_L_ADDR, data, 14); //code to read all 14 registers
                for (i=0; i<7; i++){
                    data_array[i]=(data[2*i+1]<<8)|data[2*i];
                }
        
                for (k=0; k<7; k++){    //transfer all into shorts
                sprintf(message, "Data %d = %d   ", k ,data_array[k]);
                display_string(28, 25+10*k, message);
                }*/
                while (_CP0_GET_COUNT() < 4800000) { // 200ms delay = 200ms*24MHz
                    ; //delay for 200ms
                }
            }
            break;
        }

            /* TODO: implement your application state machine.*/


            /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}



/*******************************************************************************
 End of File
 */
