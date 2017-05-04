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
#include <stdio.h>
#include <xc.h>
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

#define FACTOR 1000
/* Mouse Report */
MOUSE_REPORT mouseReport APP_MAKE_BUFFER_DMA_READY;
MOUSE_REPORT mouseReportPrevious APP_MAKE_BUFFER_DMA_READY;


// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

void APP_USBDeviceHIDEventHandler(USB_DEVICE_HID_INDEX hidInstance,
        USB_DEVICE_HID_EVENT event, void * eventData, uintptr_t userData) {
    APP_DATA * appData = (APP_DATA *) userData;

    switch (event) {
        case USB_DEVICE_HID_EVENT_REPORT_SENT:

            /* This means the mouse report was sent.
             We are free to send another report */

            appData->isMouseReportSendBusy = false;
            break;

        case USB_DEVICE_HID_EVENT_REPORT_RECEIVED:

            /* Dont care for other event in this demo */
            break;

        case USB_DEVICE_HID_EVENT_SET_IDLE:

            /* Acknowledge the Control Write Transfer */
            USB_DEVICE_ControlStatus(appData->deviceHandle, USB_DEVICE_CONTROL_STATUS_OK);

            /* save Idle rate received from Host */
            appData->idleRate = ((USB_DEVICE_HID_EVENT_DATA_SET_IDLE*) eventData)->duration;
            break;

        case USB_DEVICE_HID_EVENT_GET_IDLE:

            /* Host is requesting for Idle rate. Now send the Idle rate */
            USB_DEVICE_ControlSend(appData->deviceHandle, &(appData->idleRate), 1);

            /* On successfully receiving Idle rate, the Host would acknowledge back with a
               Zero Length packet. The HID function driver returns an event
               USB_DEVICE_HID_EVENT_CONTROL_TRANSFER_DATA_SENT to the application upon
               receiving this Zero Length packet from Host.
               USB_DEVICE_HID_EVENT_CONTROL_TRANSFER_DATA_SENT event indicates this control transfer
               event is complete */

            break;

        case USB_DEVICE_HID_EVENT_SET_PROTOCOL:
            /* Host is trying set protocol. Now receive the protocol and save */
            appData->activeProtocol = *(USB_HID_PROTOCOL_CODE *) eventData;

            /* Acknowledge the Control Write Transfer */
            USB_DEVICE_ControlStatus(appData->deviceHandle, USB_DEVICE_CONTROL_STATUS_OK);
            break;

        case USB_DEVICE_HID_EVENT_GET_PROTOCOL:

            /* Host is requesting for Current Protocol. Now send the Idle rate */
            USB_DEVICE_ControlSend(appData->deviceHandle, &(appData->activeProtocol), 1);

            /* On successfully receiving Idle rate, the Host would acknowledge
              back with a Zero Length packet. The HID function driver returns
              an event USB_DEVICE_HID_EVENT_CONTROL_TRANSFER_DATA_SENT to the
              application upon receiving this Zero Length packet from Host.
              USB_DEVICE_HID_EVENT_CONTROL_TRANSFER_DATA_SENT event indicates
              this control transfer event is complete */
            break;

        case USB_DEVICE_HID_EVENT_CONTROL_TRANSFER_DATA_SENT:
            break;

        default:
            break;
    }
}

/*******************************************************************************
  Function:
    void APP_USBDeviceEventHandler (USB_DEVICE_EVENT event,
        USB_DEVICE_EVENT_DATA * eventData)
  Summary:
    Event callback generated by USB device layer.
  Description:
    This event handler will handle all device layer events.
  Parameters:
    None.
  Returns:
    None.
 */

void APP_USBDeviceEventHandler(USB_DEVICE_EVENT event, void * eventData, uintptr_t context) {
    USB_DEVICE_EVENT_DATA_CONFIGURED * configurationValue;
    switch (event) {
        case USB_DEVICE_EVENT_SOF:
            /* This event is used for switch debounce. This flag is reset
             * by the switch process routine. */
            appData.sofEventHasOccurred = true;
            appData.setIdleTimer++;
            break;
        case USB_DEVICE_EVENT_RESET:
        case USB_DEVICE_EVENT_DECONFIGURED:

            /* Device got deconfigured */

            appData.isConfigured = false;
            appData.isMouseReportSendBusy = false;
            appData.state = APP_STATE_WAIT_FOR_CONFIGURATION;
            //appData.emulateMouse = true;
            //BSP_LEDOn ( APP_USB_LED_1 );
            //BSP_LEDOn ( APP_USB_LED_2 );
            //BSP_LEDOff ( APP_USB_LED_3 );

            break;

        case USB_DEVICE_EVENT_CONFIGURED:

            /* Device is configured */
            configurationValue = (USB_DEVICE_EVENT_DATA_CONFIGURED *) eventData;
            if (configurationValue->configurationValue == 1) {
                appData.isConfigured = true;

                //BSP_LEDOff ( APP_USB_LED_1 );
                //BSP_LEDOff ( APP_USB_LED_2 );
                //BSP_LEDOn ( APP_USB_LED_3 );

                /* Register the Application HID Event Handler. */

                USB_DEVICE_HID_EventHandlerSet(appData.hidInstance,
                        APP_USBDeviceHIDEventHandler, (uintptr_t) & appData);
            }
            break;

        case USB_DEVICE_EVENT_POWER_DETECTED:

            /* VBUS was detected. We can attach the device */
            USB_DEVICE_Attach(appData.deviceHandle);
            break;

        case USB_DEVICE_EVENT_POWER_REMOVED:

            /* VBUS is not available any more. Detach the device. */
            USB_DEVICE_Detach(appData.deviceHandle);
            break;

        case USB_DEVICE_EVENT_SUSPENDED:
            //BSP_LEDOff ( APP_USB_LED_1 );
            //BSP_LEDOn ( APP_USB_LED_2 );
            //BSP_LEDOn ( APP_USB_LED_3 );
            break;

        case USB_DEVICE_EVENT_RESUMED:
        case USB_DEVICE_EVENT_ERROR:
        default:
            break;

    }
}


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
    appData.deviceHandle = USB_DEVICE_HANDLE_INVALID;
    appData.isConfigured = false;
    //appData.emulateMouse = true;
    appData.hidInstance = 0;
    appData.isMouseReportSendBusy = false;


    /*Initialise IMU*/
    TRISBbits.TRISB4 = 1; //Set pin RB4 as an input pin
    TRISAbits.TRISA4 = 0; //Set pin RA4 as an output pin
    LATAbits.LATA4 = 1; //Initialise LED to be HIGH
    SPI1_init();
    LCD_init();
    LCD_clearScreen(GREEN);
    initIMU();

}

/******************************************************************************
  Function:
    void APP_Tasks ( void )
  Remarks:
    See prototype in app.h.
 */

void APP_Tasks(void) {
    //static int8_t vector = 0;
    //static uint8_t movement_length = 0;
    int8_t dir_table[] = {-4, -4, -4, 0, 4, 4, 4, 0};
    static uint8_t inc = 0;

    unsigned char data[14];
    char message[20];
    signed short data_array[7];
    int i = 0, k = 0;

    /* Check the application's current state. */
    switch (appData.state) {
            /* Application's initial state. */
        case APP_STATE_INIT:
        {
            /* Open the device layer */
            appData.deviceHandle = USB_DEVICE_Open(USB_DEVICE_INDEX_0,
                    DRV_IO_INTENT_READWRITE);

            if (appData.deviceHandle != USB_DEVICE_HANDLE_INVALID) {
                /* Register a callback with device layer to get event notification (for end point 0) */
                USB_DEVICE_EventHandlerSet(appData.deviceHandle,
                        APP_USBDeviceEventHandler, 0);

                appData.state = APP_STATE_WAIT_FOR_CONFIGURATION;
            } else {
                /* The Device Layer is not ready to be opened. We should try
                 * again later. */
            }
            break;
        }

        case APP_STATE_WAIT_FOR_CONFIGURATION:

            /* Check if the device is configured. The 
             * isConfigured flag is updated in the
             * Device Event Handler */

            if (appData.isConfigured) {
                appData.state = APP_STATE_MOUSE_EMULATE;
            }
            break;

        case APP_STATE_MOUSE_EMULATE:

            // every 50th loop, or 20 times per second
            /*if (movement_length > 50) {
                appData.mouseButton[0] = MOUSE_BUTTON_STATE_RELEASED;
                appData.mouseButton[1] = MOUSE_BUTTON_STATE_RELEASED;
                appData.xCoordinate = (int8_t) dir_table[vector & 0x07];
                appData.yCoordinate = (int8_t) dir_table[(vector + 2) & 0x07];
                vector++;
                movement_length = 0;
            }*/
            
            
            I2C_read_multiple(SLAVE_ADDR, OUTX_L_XL_ADDR, data, 6);
            for (i = 0; i < 3; i++) {
                data_array[i] = (data[2 * i + 1] << 8) | data[2 * i];
            }

            if (inc == 5) {
                appData.mouseButton[0] = MOUSE_BUTTON_STATE_RELEASED;
                appData.mouseButton[1] = MOUSE_BUTTON_STATE_RELEASED;
                appData.xCoordinate = data_array[0]/FACTOR;
                appData.yCoordinate = data_array[1]/FACTOR;
                inc = 0;
            } else {
                appData.mouseButton[0] = MOUSE_BUTTON_STATE_RELEASED;
                appData.mouseButton[1] = MOUSE_BUTTON_STATE_RELEASED;
                appData.xCoordinate = 0;
                appData.yCoordinate = 0;
                inc++;
            }




            if (!appData.isMouseReportSendBusy) {
                /* This means we can send the mouse report. The
                   isMouseReportBusy flag is updated in the HID Event Handler. */

                appData.isMouseReportSendBusy = true;

                /* Create the mouse report */

                MOUSE_ReportCreate(appData.xCoordinate, appData.yCoordinate,
                        appData.mouseButton, &mouseReport);

                if (memcmp((const void *) &mouseReportPrevious, (const void *) &mouseReport,
                        (size_t)sizeof (mouseReport)) == 0) {
                    /* Reports are same as previous report. However mouse reports
                     * can be same as previous report as the coordinate positions are relative.
                     * In that case it needs to be sent */
                    if ((appData.xCoordinate == 0) && (appData.yCoordinate == 0)) {
                        /* If the coordinate positions are 0, that means there
                         * is no relative change */
                        if (appData.idleRate == 0) {
                            appData.isMouseReportSendBusy = false;
                        } else {
                            /* Check the idle rate here. If idle rate time elapsed
                             * then the data will be sent. Idle rate resolution is
                             * 4 msec as per HID specification; possible range is
                             * between 4msec >= idlerate <= 1020 msec.
                             */
                            if (appData.setIdleTimer
                                    >= appData.idleRate * 4) {
                                /* Send REPORT as idle time has elapsed */
                                appData.isMouseReportSendBusy = true;
                            } else {
                                /* Do not send REPORT as idle time has not elapsed */
                                appData.isMouseReportSendBusy = false;
                            }
                        }
                    }

                }
                if (appData.isMouseReportSendBusy == true) {
                    /* Copy the report sent to previous */
                    memcpy((void *) &mouseReportPrevious, (const void *) &mouseReport,
                            (size_t)sizeof (mouseReport));

                    /* Send the mouse report. */
                    USB_DEVICE_HID_ReportSend(appData.hidInstance,
                            &appData.reportTransferHandle, (uint8_t*) & mouseReport,
                            sizeof (MOUSE_REPORT));
                    appData.setIdleTimer = 0;
                }
                //movement_length++;
            }

            break;

        case APP_STATE_ERROR:

            break;

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