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

#include <string.h>
#include <stdio.h>
#include "remote.h"

#include "sdcard.h"

FATFS Fatfs;                                    // File system object
FIL Fil;                                        // File object
BYTE Buff[128];                                 // File read buffer
short sFileOpen = 0;                            // Is a file open?


#define sdcardFILE_PREFIX "9915_V"
#define sdcardFILE_SUFFIX ".TXT"

/* Stop with dying message */
void die (FRESULT rc) {    /* FatFs return value */
    sprintf(ucRemoteBuffer, "Failed with rc=%u.\n", rc);
    remote_printf(ucRemoteBuffer);
    for (;;) ;
}

// Convert a string to a short
short sStringToShort(unsigned char* pucStr) {
    short sValue = 0;
    // if pucStr is non-zero
    if (pucStr) {
        // Check if it is not end of string and is between 0 and 9
        // Multiple current value by 10, add new integer
        // and increment pucStr pointer
        while (*pucStr != '\0' && *pucStr >= '0' && *pucStr <= '9') {
            sValue = (sValue * 10) + (*pucStr - '0');
            pucStr++;
        }
    }

    return sValue;
}

// Convert milliseconds to a timestamp
void vMillisecondsToTime(unsigned long ulMSec, short *sHr, short *sMin, short *sSec) {
    *sSec = (ulMSec/1000)%60;
    *sMin = ((ulMSec/1000)/60)%60;
    *sHr  = ((ulMSec/1000)/60)/60;
}

/* Register volume work area (never fails) */
void vSdCard_Load() {
    f_mount(0, &Fatfs);
}

// Find the highest numbered log file currently on the SD card
short sSdCard_GetLogNo() {
    FRESULT rc;                 /* Result code */
    DIR dir;                    /* Directory object */
    FILINFO fno;                /* File information object */
    UINT bw, br, i;
    short sMaxNo = 0;

    // Open root directory
    rc = f_opendir(&dir, "");
    if (rc) die(rc);

    // Iterate over directory
    for (;;) {
        rc = f_readdir(&dir, &fno);        /* Read a directory item */
        if (rc || !fno.fname[0]) break;    /* Error or end of dir */
        if (!(fno.fattrib & AM_DIR)
                && !strncmp(fno.fname, sdcardFILE_PREFIX, strlen(sdcardFILE_PREFIX))
                && sMaxNo < sStringToShort(&fno.fname[strlen(sdcardFILE_PREFIX)])) {
            sMaxNo = sStringToShort(&fno.fname[strlen(sdcardFILE_PREFIX)]);
        }
    }
    if (rc) die(rc);

    return sMaxNo+1;
}

// Create the new log file for this session
void vSdCard_CreateLogFile(short sFileNo) {
    FRESULT rc;
    unsigned char pucFileName[20];

    sprintf(pucFileName, "%s%d%s", sdcardFILE_PREFIX, sFileNo, sdcardFILE_SUFFIX);

    // Create log file
    rc = f_open(&Fil, pucFileName, FA_WRITE | FA_OPEN_ALWAYS);
    if (rc) die(rc);

    rc = f_lseek(&Fil, f_size(&Fil));
    if (rc) die(rc);

    sFileOpen = 1;
}

// Close the log file
void vSdCard_CloseLogFile() {
    if (sFileOpen == 1) {
        f_close(&Fil);
        sFileOpen = 0;
    }
}

// Sync the opened file to the SD card
void vSdCard_SyncData() {
    if (sFileOpen == 1) {
        f_sync(&Fil);
    }
}

// Log the data to the file with a timestamp
void vSdCard_LogData(short sFileNo,
                     unsigned long  ulTimeStamp,
                     short sPositionX,
                     short sPositionY,
                     short sDegreesPan,
                     short sDegreesTilt) {
    FRESULT rc;                         /* Result code */
    unsigned char *pucWriteBuffer[128];
    short sHours, sMinutes, sSeconds;
    UINT xRet;
    vMillisecondsToTime(ulTimeStamp, &sHours, &sMinutes, &sSeconds);

    if (!sFileOpen) {
        vSdCard_CreateLogFile(sFileNo);
    }

    // write to the file
    sprintf(pucWriteBuffer, "%02d-%02d-%02d-%03d: %02d, %02d, %03d, %03d\r\n",
                    sHours,
                    sMinutes,
                    sSeconds,
                    ulTimeStamp%1000,
                    sPositionX,
                    sPositionY,
                    sDegreesPan,
                    sDegreesTilt);
    sprintf(ucRemoteBuffer, "%s", pucWriteBuffer);
    remote_printf(ucRemoteBuffer);

    f_write(&Fil, pucWriteBuffer, strlen(pucWriteBuffer), &xRet);
}

void vSdCard_Ls(int8_t *Buf, size_t *psLen) {
    FRESULT rc;                 /* Result code */
    DIR dir;                    /* Directory object */
    FILINFO fno;                /* File information object */

    // Close the currently open file


    // Open root directory
    rc = f_opendir(&dir, "");
    if (rc) die(rc);

    // Print directories
    for (;;) {
        rc = f_readdir(&dir, &fno);     /* Read a directory item */
        if (rc || !fno.fname[0]) break; /* Error or end of dir */
        if (fno.fattrib & AM_DIR)
            *psLen += sprintf(Buf+*psLen, "   <dir>  %s\r\n", fno.fname);
        else
            *psLen += sprintf(Buf+*psLen, "%8lu  %s\r\n", fno.fsize, fno.fname);
    }
    if (rc) die(rc);
}

/*---------------------------------------------------------*/
/* User Provided Timer Function for FatFs module           */
/*---------------------------------------------------------*/

DWORD get_fattime (void)
{
    return      ((DWORD)(2012 - 1980) << 25)    /* Year = 2012 */
            | ((DWORD)1 << 21)                /* Month = 1 */
            | ((DWORD)1 << 16)                /* Day_m = 1*/
            | ((DWORD)0 << 11)                /* Hour = 0 */
            | ((DWORD)0 << 5)                /* Min = 0 */
            | ((DWORD)0 >> 1);                /* Sec = 0 */
}
