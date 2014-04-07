 /* -----------------------------------------------------------------------------
# Company name: CSSE3010 - Embedded Systems Design & Interfacing
# Project name: Project 2
# Module name : sdcard
# Functionality: SD Card Controller
# Hardware platform: Netduino (AT91SAM7)
#
# Author name: Merrick Heley
# Creation date: 2013-04-22
------------------------------------------------------------------------------*/

#ifndef __SDCARD_H__
#define __SDCARD_H__

#include "FreeRTOS.h"
#include "FreeRTOS_CLI.h"
#include "ff.h"

void die (FRESULT rc);

// Convert string to short
short   sStringToShort(unsigned char* pucStr);

// Load the sd card
void    vSdCard_Load           ();

// Get the current log number
short   sSdCard_GetLogNo       ();

// Create a log file
void    vSdCard_CreateLogFile  (short sFileNo);

// Log data to the logfile
void    vSdCard_LogData        (short sFileNo,
                                unsigned long  ulTimeStamp,
                                short sPositionX,
                                short sPositionY,
                                short sDegreesPan,
                                short sDegreesTilt);

// Sync to the sd card
void    vSdCard_Sync           ();

// Close the Sd card
void    vSdCard_CloseLogFile   ();

#endif
