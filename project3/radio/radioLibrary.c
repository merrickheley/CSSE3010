 /* -----------------------------------------------------------------------------
# Company name: CSSE3010 - Embedded Systems Design & Interfacing
# Project name: Project 3
# Module name : radioLibrary
# Functionality: Support functions for radio usage
# Hardware platform: Netduino (AT91SAM7)
#
# Author name: Merrick Heley
# Creation date: 2013-05-13

------------------------------------------------------------------------------*/

#include "radioLibrary.h"
#include "nrf24l01plus.h"
#include "../sdcard/sdcard.h"

#include <string.h>
#include <debug_printf.h>
#include <pio/pio.h>

#define radiolibPACKET_DEFAULT {RF_CHANNEL, 0, 0x7F, 0x7F, 0x7F, 0, 0}

#define radiolibCMD_PASSKEY         0xC2
#define radiolibCMD_MAG_X           0xC3
#define radiolibCMD_IR_RAW          0xC4
#define radiolibCMD_TURN_RIGHT      0xC7
#define radiolibCMD_TURN_LEFT       0xC8

unsigned char ucAckPassKey;
unsigned char ucAckMagX;
unsigned char ucAckTurnLeft;

unsigned char ucEnableMotors;
unsigned char ucForwardMotors;
unsigned char ucDirection;
unsigned char ucAltitude;
unsigned char ucTurnLeft;

// Radio pins
static const Pin pinsRF[] = {nrf24l01plus_MISO, nrf24l01plus_MOSI, nrf24l01plus_SCK, nrf24l01plus_CE, nrf24l01plus_CSN};

/* Transmit a string over radio */
short sRadioTransmit(unsigned char *pucTransmit) {

    // Check if string is transmit-able
    if (strlen(pucTransmit) < 32) {
        xQueueSendToBack(xRadioTX_Queue, pucTransmit, 0);
    } else {
        return 1;
    }

    return 0;
}

/* Send a give packet */
short sSendPacket(xTransmitPacket *pxSend) {
    unsigned char pucTransmit[32] = {0};

    pucTransmit[0] = pxSend->ucBlimpId;
    pucTransmit[1] = ucEnableMotors;
    pucTransmit[2] = ucForwardMotors;
    pucTransmit[3] = ucDirection;
    pucTransmit[4] = ucAltitude;
    pucTransmit[5] = pxSend->ucSpecialCmd;
    pucTransmit[6] = (pxSend->usSpecialData & 0xFF00)/0xFF;
    pucTransmit[7] = pxSend->usSpecialData & 0x00FF;

    return sRadioTransmit(pucTransmit);
}

/* Send a default packet */
short sSendDefault() {
    xTransmitPacket xSend = radiolibPACKET_DEFAULT;
    return sSendPacket(&xSend);
}

/* Request the pass key */
short sRequestPassKey(short sId) {
    xTransmitPacket xSend = radiolibPACKET_DEFAULT;

    xSend.ucSpecialCmd = radiolibCMD_PASSKEY;
    xSend.usSpecialData = sId;
    ucAckPassKey = 1;

    return sSendPacket(&xSend);
}

/* Request the mag X */
short sRequestMagX() {
    xTransmitPacket xSend = radiolibPACKET_DEFAULT;

    xSend.ucSpecialCmd = radiolibCMD_MAG_X;
    ucAckMagX = 1;

    return sSendPacket(&xSend);
}

/* Turn the blimp left */
short sRequestTurnLeft() {
    xTransmitPacket xSend = radiolibPACKET_DEFAULT;

    xSend.ucSpecialCmd = radiolibCMD_TURN_LEFT;
    ucAckTurnLeft= 1;

    return sSendPacket(&xSend);
}

/* Send a command to the blimp preserving previous values */
short sCommandBlimp() {
    xTransmitPacket xSend = radiolibPACKET_DEFAULT;

    return sSendPacket(&xSend);
}

/* Enable or disable the motors */
short sToggleMotors() {
    ucEnableMotors = !ucEnableMotors;

    return sCommandBlimp();
}

/* Check if the motors are enabled */
short sMotorsEnabled() {
    return ucEnableMotors;
}

/* Set the speed of the blimp */
short sSetSpeed(unsigned char ucStatus) {
    ucForwardMotors = ucStatus;
}

/* Set the direction of the blimp */
short sSetDirection(unsigned char ucStatus) {
    ucDirection = ucStatus;
}

/* Set the altitude of the blimp */
short sSetAltitude(unsigned char ucStatus) {
    ucAltitude = ucStatus;
}

/* Process a packet */
void vProcessPacket(unsigned char *pucReceived) {
    xBlimpPacket xTemp;

    xTemp.ucBlimpId                 = pucReceived[0];
    xTemp.ucAckEnableMotors         = pucReceived[1];
    xTemp.ucAckForwardMotors        = pucReceived[2];
    xTemp.ucAckDirectionCommand     = pucReceived[3];
    xTemp.ucAckAltitude             = pucReceived[4];
    xTemp.ucAckSpecial              = pucReceived[5];
    xTemp.usRequested               = pucReceived[7]  << 8 | pucReceived[6];
    xTemp.usIRValues                = pucReceived[9]  << 8 | pucReceived[8];
    xTemp.usMagnometer              = pucReceived[11] << 8 | pucReceived[10];

    // Resend if values do not match
    if (ucEnableMotors &&
            (xTemp.ucAckEnableMotors != ucEnableMotors
            || xTemp.ucAckForwardMotors != ucForwardMotors
            || xTemp.ucAckDirectionCommand != ucDirection
            || xTemp.ucAckAltitude != ucAltitude)) {

        sCommandBlimp();
    }

    // Check if waiting for a pass key response
    if (ucAckPassKey) {
        if (xTemp.ucAckSpecial == radiolibCMD_PASSKEY) {
                debug_printf("Passkey: %04X\r\n", xTemp.usRequested);
                ucAckPassKey = 0;
                sSendDefault();
        } else {
            sRequestPassKey(radiolibPASS_ID);
        }
    }

    // Check if waiting for a magnetometer response
    if (ucAckMagX) {
        if (xTemp.ucAckSpecial == radiolibCMD_MAG_X) {
                debug_printf("MagX: %04X\r\n", xTemp.usRequested);
                ucAckMagX = 0;
                sSendDefault();
        } else {
            sRequestMagX();
        }
    }

    if (ucAckTurnLeft) {
        if (xTemp.ucAckSpecial == radiolibCMD_MAG_X) {
                ucAckTurnLeft = 0;
                sSendDefault();
        } else {
            sRequestTurnLeft();
        }
    }

    // Log the data to the SD card
    vSdCard_LogData( sFileNo,
                    (unsigned long) xTaskGetTickCount(),
                     xTemp.ucBlimpId,
                     xTemp.usIRValues,
                     xTemp.usMagnometer,
                     xTemp.ucAckForwardMotors,
                     xTemp.ucAckDirectionCommand,
                     xTemp.ucAckAltitude);


    xPrintPacket = xTemp;
}

/* Configure the radio */
void vRadioConfigure() {

    // Configure default packet
    ucEnableMotors    = 0;
    ucForwardMotors   = 0x7F;
    ucDirection       = 0x7F;
    ucAltitude        = 0x7F;

    // Configure acknowledges
    ucAckPassKey = 0;
    ucAckMagX = 0;

    // Load the sd card
    vSdCard_Load();
    sFileNo = sSdCard_GetLogNo();
    vSdCard_CreateLogFile(sFileNo);

    // Configure RF pins
    PIO_Configure(pinsRF, PIO_LISTSIZE(pinsRF));

    // Runs init
    nrf24l01plus_init();
}
