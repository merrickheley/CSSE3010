 /* -----------------------------------------------------------------------------
# Company name: CSSE3010 - Embedded Systems Design & Interfacing
# Project name: Project 3
# Module name : cliCallbacks
# Functionality: Creates the CLI command callbacks
# Hardware platform: Netduino (AT91SAM7)
#
# Author name: Merrick Heley
# Creation date: 2013-05-13
------------------------------------------------------------------------------*/

#include <FreeRTOS.h>
#include <pio/pio.h>
#include <string.h>
#include <debug_printf.h>

#include "cliCallbacks.h"
#include "../radio/radioLibrary.h"
#include "../servo/servoLibrary.h"
#include "../sdcard/sdcard.h"
#include "../conv/convolution.h"
#include "../i2c/mag/magLibrary.h"

// Convert a string of hex chars to a number
short sHexStringToNum(char *pucInput, short sLen, long *lOutput) {
    *lOutput = 0;
    short sCountLen = 0;

    // Iterate over the chars
    // Multiply the current number by 16 and add the new number
    while (sCountLen < sLen) {
        if ('0' <= *pucInput && *pucInput <= '9') {
            *lOutput = *lOutput*16 + (*pucInput-'0');
        } else if ('a' <= *pucInput && *pucInput <= 'f') {
            *lOutput = *lOutput*16 + (*pucInput-'a' + 10);
        } else if ('A' <= *pucInput && *pucInput <= 'F') {
            *lOutput = *lOutput*16 + (*pucInput-'A' + 10);
        } else {
            return 1;
        }

        pucInput++;
        sCountLen++;
    }

    return 0;
}

/*-----------------------------------------------------------*/
/* CLI Echo Function */
portBASE_TYPE prvEchoCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString ) {

    long lParam_len;
    char *cCmd_string;

    //Get parameters from command string
    cCmd_string = FreeRTOS_CLIGetParameter(pcCommandString, 1, &lParam_len);

    //sprintf(ucRemoteBuffer, "\r\necho %s\r\n", cCmd_string);
    //remote_printf(ucRemoteBuffer);

    //Write command echo output string to write buffer.
    xWriteBufferLen = sprintf(pcWriteBuffer, "\r\n>> %s\r\n", cCmd_string);

    return pdTRUE;
}

/*-----------------------------------------------------------*/
/* Radio transmit command */
portBASE_TYPE prvSendCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString ) {

    long lParam_len;
    char *cCmd_string;

    //Get parameters from command string
    cCmd_string = FreeRTOS_CLIGetParameter(pcCommandString, 1, &lParam_len);

    // If command is within length, transmit it
    if (!sRadioTransmit(cCmd_string)) {
        xWriteBufferLen = sprintf(pcWriteBuffer, "\r\n>> Sent: %s\r\n", cCmd_string);
    } else {
        xWriteBufferLen = sprintf(pcWriteBuffer, "\r\n>> Send failed.\r\n", cCmd_string);
    }

    return pdTRUE;
}

/*-----------------------------------------------------------*/
/* Stop period printing */
portBASE_TYPE prvPerioidCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString ) {

    ucPeriodicPrinting = !ucPeriodicPrinting;
    xWriteBufferLen = sprintf(pcWriteBuffer, "\r\nPeriodic display toggled to %d\r\n", ucPeriodicPrinting);

    return pdTRUE;
}

/*-----------------------------------------------------------*/
/* Stop period printing */
portBASE_TYPE prvDebugCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString ) {

    ucDebugPrinting = (ucDebugPrinting+1)%3;
    xWriteBufferLen = sprintf(pcWriteBuffer, "\r\nDebugging toggled to %d\r\n", ucDebugPrinting);

    return pdTRUE;
}

/*-----------------------------------------------------------*/
/* Request the pass key */
portBASE_TYPE prvPassKeyCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString ) {

    sRequestPassKey(radiolibPASS_ID); // or 0x1599?
    xWriteBufferLen = sprintf(pcWriteBuffer, "\r\nPass key requested.\r\n");

    return pdTRUE;
}

/*-----------------------------------------------------------*/
/* Request the pass key */
portBASE_TYPE prvMagXCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString ) {
    sRequestMagX();
    xWriteBufferLen = sprintf(pcWriteBuffer, "\r\nMagnetometer X value requested.\r\n");

    return pdTRUE;
}

/*-----------------------------------------------------------*/
/* Prepare the SD card for ejection */
portBASE_TYPE prvEjectCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString ) {
    vSdCard_SyncData();
    xWriteBufferLen = sprintf(pcWriteBuffer, "\r\n>> SD Card: Files synchronised and ready to be ejected\r\n");

    return pdTRUE;
}

/*-----------------------------------------------------------*/
/* Enable or disable the motors */
portBASE_TYPE prvMotorsCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString ) {

    sToggleMotors();
    xWriteBufferLen = sprintf(pcWriteBuffer, "\r\n>> Motors toggled to %d\r\n", sMotorsEnabled());

    return pdTRUE;
}

/*-----------------------------------------------------------*/
/* Set the height above ground level */
portBASE_TYPE prvHeightCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString ) {

    long lParam_len,lTemp;
    char *cCmd_string;

    //Get parameters from command string
    cCmd_string = FreeRTOS_CLIGetParameter(pcCommandString, 1, &lParam_len);

    if (!sHexStringToNum(cCmd_string, 2, &lTemp)) {
        if (sMotorsEnabled()) {
            sSetAltitude((unsigned char) lTemp);
            xWriteBufferLen = sprintf(pcWriteBuffer, "\r\n>> Height set to 0x%02X\r\n", (unsigned char) lTemp);
            sCommandBlimp();
        } else {
            xWriteBufferLen = sprintf(pcWriteBuffer, "\r\n>> Motors not on\r\n", (unsigned char) lTemp);
        }
    } else {
        xWriteBufferLen = sprintf(pcWriteBuffer, "\r\n>> Bad data, height not changed\r\n");
    }
    return pdTRUE;
}

/* Turn on task5 */
portBASE_TYPE prvTask5Command(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString ) {

    sTask5 = !sTask5;
    xWriteBufferLen = sprintf(pcWriteBuffer, "\r\nTask5 toggled to %d\r\n", sTask5);
    return pdTRUE;
}

portBASE_TYPE prvMagCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString ) {

    sMagEnabled = !sMagEnabled;
    xWriteBufferLen = sprintf(pcWriteBuffer, "\r\nMag toggled to %d\r\n", sMagEnabled);
    return pdTRUE;
}

// Task for replaying from a log file
portBASE_TYPE prvReplayCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString ) {
    FIL Fil;                                // File object
    BYTE RawBuff[121];                      // File read buffer, number of chars in a line
    BYTE Buff[21];                          // Processed line

    FRESULT rc;                             // Result code
    DIR dir;                                // Directory object
    FILINFO fno;                            // File information object
    long lParam_len;                        // Length of arg
    char *cCmd_string;                      // Return arg
    UINT bw, br, i;                         // File counters

    short sFirstRead = 1;                   // Is this the first read from the file
    unsigned char pucNewString[8]     = {0};
    unsigned char pucTimeString[8] = {0};   // Previous timestring
    long j;
    //Get parameters from command string
    cCmd_string = FreeRTOS_CLIGetParameter(pcCommandString, 1, &lParam_len);

    rc = f_open(&Fil, cCmd_string, FA_OPEN_EXISTING | FA_READ);
    if (rc) {
        xWriteBufferLen = sprintf(pcWriteBuffer, "\r\n>> No such file\r\n");
        return pdTRUE;
    }
    if (rc) die(rc);

    // Read data
    for (;;) {
        rc = f_read(&Fil, RawBuff, 120, &br);                 /* Read a chunk of file */
        if (rc || !br) break;                                 /* Error or end of file */

        // Get the converted data
        for (i=0; i<=121; i+=6) {
           Buff[i/6] = ucDecoder(&RawBuff[i]);
        }
        debug_printf("%s", Buff);

        strncpy(pucNewString, Buff, 8);

        rc = f_read(&Fil, RawBuff, 120, &br);                 /* Read a chunk of file */
        if (rc || !br) break;                                 /* Error or end of file */

        // Get the converted data
        for (i=0; i<=121; i+=6) {
           Buff[i/6] = ucDecoder(&RawBuff[i]);
        }
        Buff[19] = '\0';
        debug_printf("%s\r\n", Buff);

        sHexStringToNum(Buff+06, 3, &j);
        sSetSpeed((char) j);
        debug_printf("%02x", (char) j);

        sHexStringToNum(Buff+11, 3, &j);
        sSetDirection((char) j);
        debug_printf(" %02x", (char) j);

        sHexStringToNum(Buff+16, 3, &j);
        sSetAltitude((char) j);
        debug_printf(" %02x", (char) j);

        sCommandBlimp();
        debug_printf("\r\n");

        // If the timestamp is different, save it
        if (strncmp(pucTimeString, pucNewString, 8)) {

            strncpy(pucTimeString, pucNewString, 8);

            vTaskDelay(1000);
        }
    }
    if (rc) die(rc);

    xWriteBufferLen = sprintf(pcWriteBuffer, "\r\nReplay complete\r\n\r\n");
    return pdTRUE;
}
