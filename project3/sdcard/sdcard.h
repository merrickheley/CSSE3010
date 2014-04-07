 /* -----------------------------------------------------------------------------
# Company name: CSSE3010 - Embedded Systems Design & Interfacing
# Project name: Project 3
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

// Unmounts the sd card
void    vSdCard_UnLoad         ();

// Get the current log number
short   sSdCard_GetLogNo       ();

// Create a log file
void    vSdCard_CreateLogFile  (short sFileNo);

// Log data to the logfile
void vSdCard_LogData(short sFileNo,
                     unsigned long  ulTimeStamp,
                     unsigned char blimpId,
                     short heightAGL,
                     short magHeading,
                     unsigned char fwdCmd,
                     unsigned char dirCmd,
                     unsigned char altCmd);

// Sync to the sd card
void    vSdCard_Sync           ();

// Close the Sd card
void    vSdCard_CloseLogFile   ();

#endif
