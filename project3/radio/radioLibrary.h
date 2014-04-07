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

#ifndef __RADIOLIBRARY_H__
#define __RADIOLIBRARY_H__

#define radiolibPASS_ID         0x9915
#define radiolibENABLE_MOTORS   1
#define radiolibDISABLE_MOTORS  0

typedef struct {
    unsigned char ucBlimpId;
    unsigned char ucAckEnableMotors;
    unsigned char ucAckForwardMotors;
    unsigned char ucAckDirectionCommand;
    unsigned char ucAckAltitude;
    unsigned char ucAckSpecial;
    unsigned short usRequested;
    unsigned short usIRValues;
    unsigned short usMagnometer;
} xBlimpPacket;

typedef struct {
    unsigned char ucBlimpId;
    unsigned char ucEnableMotors;
    unsigned char ucFowardMotors;
    unsigned char ucDirection;
    unsigned char ucAltitude;
    unsigned char ucSpecialCmd;
    unsigned short usSpecialData;
} xTransmitPacket __attribute__((packed));

xBlimpPacket xPrintPacket;

short sFileNo;

/* ------------------ Functions         ------------------ */

/* Configures the radio for use */
void vRadioConfigure();

/* Processes a received packet */
void vProcessPacket();

/* Send raw data over radio */
short sRadioTransmit(unsigned char *pucTransmit);

/* Send the ID for a passkey */
short sRequestPassKey(short sId);

/* Send the magX request */
short sRequestMagX();

/* Send a command to the blimp preserving previous values */
short sCommandBlimp();

/* Check if the motors are enabled */
short sMotorsEnabled();

/* Enable the motors */
short sSetMotors(unsigned char ucStatus);

/* Set the speed of the blimp */
short sSetSpeed(unsigned char ucStatus);

/* Set the direction of the blimp */
short sSetDirection(unsigned char ucStatus);

/* Set the altitude of the blimp */
short sSetAltitude(unsigned char ucStatus);

#endif /* __RADIOLIBRARY_H__ */
