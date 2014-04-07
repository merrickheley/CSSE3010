 /* -----------------------------------------------------------------------------
# Company name: CSSE3010 - Embedded Systems Design & Interfacing
# Project name: Project 2
# Module name : cli
# Functionality: Creates the CLI commands
# Hardware platform: Netduino (AT91SAM7)
#
# Author name: Merrick Heley
# Creation date: 2013-04-25
------------------------------------------------------------------------------*/

#include "cli.h"

#include "sdcard.h"
#include "servo.h"
#include "convolution.h"
#include "usart.h"
#include "tuioclient.h"

#include "remote.h"
#include <pio/pio.h>
#include <string.h>

static const Pin pins[] = {
    PIN_LASER,
    PIN_USART0_RXD,
    PIN_USART0_TXD
};

// Convert a timestamp of format "HH-MM-SS" to seconds
long lTimestampDifference(unsigned char *pucTimestamp1, unsigned char *pucTimestamp2) {
    return ((long) sStringToShort(pucTimestamp1+0)-(long) sStringToShort(pucTimestamp2+0))*60*60*1000 +
           ((long) sStringToShort(pucTimestamp1+3)-(long) sStringToShort(pucTimestamp2+3))*60*1000 +
           ((long) sStringToShort(pucTimestamp1+6)-(long) sStringToShort(pucTimestamp2+6))*1000 +
           ((long) sStringToShort(pucTimestamp1+9)-(long) sStringToShort(pucTimestamp2+9));
}

/* Structure that defines the "echo" command line command. */
static const xCommandLineInput xEcho =
{
    ( char * ) "echo",
    ( char * ) "\r\necho: Echo the input.\r\n",
    prvEchoCommand,
    1
};

/*-----------------------------------------------------------*/
/* CLI Echo Function */
static portBASE_TYPE prvEchoCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString ) {

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

/* Structure that defines the "test" command line command. */
static const xCommandLineInput xTest =
{
    ( char * ) "test",
    ( char * ) "\r\ntest: Test output.\r\n",
    prvTestCommand,
    0

};

/*-----------------------------------------------------------*/
/* CLI Test Function */
static portBASE_TYPE prvTestCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString ) {

    xWriteBufferLen = sprintf(pcWriteBuffer, "\r\n>> Tested..\r\n");

    return pdTRUE;
}

// Structure for the eject command
static const xCommandLineInput xEject =
{
    ( char * ) "eject",
    ( char * ) "\r\neject: Safely eject the SD card\r\n",
    prvEjectCommand,
    0
};

// Prepare the SD card for ejection
static portBASE_TYPE prvEjectCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString ) {
    vSdCard_SyncData();
    xWriteBufferLen = sprintf(pcWriteBuffer, "\r\n>> SD Card: Files synchronised and ready to be ejected\r\n");

    sCloseLog = 1;

    return pdTRUE;
}

static const xCommandLineInput xLaser =
{
    ( char * ) "laser",
    ( char * ) "\r\nlaser: turn the laser <on> or <off>\r\n",
    prvLaserCommand,
    1
};

// Turn the laser on or off
static portBASE_TYPE prvLaserCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString ) {
    long lParam_len;
    char *cCmd_string;

    //Get parameters from command string
    cCmd_string = FreeRTOS_CLIGetParameter(pcCommandString, 1, &lParam_len);

    if (!strcmp(cCmd_string, "on")) {
        PIO_Set(&pins[0]);
        xWriteBufferLen = sprintf(pcWriteBuffer, "\r\n>> Laser <on>\r\n");
    } else if (!strcmp(cCmd_string, "off")) {
        PIO_Clear(&pins[0]);
        xWriteBufferLen = sprintf(pcWriteBuffer, "\r\n>> Laser <off>\r\n");
    } else {
        xWriteBufferLen = sprintf(pcWriteBuffer, "\r\n>> Invalid argument to laser\r\n");
    }

    return pdTRUE;
}

static const xCommandLineInput xTail =
{
    ( char * ) "tail",
    ( char * ) "\r\ntail: display the last 10 lines of a file\r\n",
    prvTailCommand,
    1
};

// Tail command, display the last 10 lines of a file
static portBASE_TYPE prvTailCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString ) {
    static FIL Fil;                             // File object
    BYTE xBuff[20];                              // File read buffer

    static short sFlag = 0;                     // Flag, indicates stage in code
    static FRESULT rc;                          // Result code
    static FILINFO fno;                         // File information object
    portBASE_TYPE xReturn;                      // Return for function
    long lParam_len;                            // Length of arg
    char *cCmd_string;                          // Return arg
    static UINT bw, br, i;                      // File counters

    DWORD xNBuffer[cliTAIL+1];                  // Newline buffer, circular array
    unsigned short usNBufferCounter;            // Newline buffer counter
    unsigned short usReadCounter;               // Read counter

    // Close the currently open file and navigate to the root dir
    if (sFlag == 0) {
        vSdCard_CloseLogFile();

        // Clear counters
        usNBufferCounter = 0;
        usReadCounter = 0;
        memset(xNBuffer, 0, (cliTAIL+1)*sizeof(DWORD));

        //Get parameters from command string
        cCmd_string = FreeRTOS_CLIGetParameter(pcCommandString, 1, &lParam_len);

        // Open the file up
        rc = f_open(&Fil, cCmd_string, FA_OPEN_EXISTING | FA_READ);
        if (rc) {
            xWriteBufferLen = sprintf(pcWriteBuffer, "\r\n>> No such file\r\n");
            return pdTRUE;
        }

        // Add every newline to a circular array
        for (;;) {
            rc = f_read(&Fil, xBuff, sizeof(xBuff), &br);     /* Read a chunk of file */
            if (rc || !br) break;                           /* Error or end of file */
            for (i = 0; i < br; i++) {                      /* Add newlines to the array */
                if (xBuff[i] == '\n') {
                    xNBuffer[usNBufferCounter] = sizeof(xBuff)*usReadCounter + i + 1;
                    usNBufferCounter = (usNBufferCounter+1)%(cliTAIL+1);
                }
            }
            usReadCounter++;
        }
        if (rc) die(rc);

        // Seek to character
        rc = f_lseek(&Fil, xNBuffer[usNBufferCounter]);
        if (rc) die(rc);

        xWriteBufferLen = sprintf(pcWriteBuffer, "\r\n");

        sFlag++;
        xReturn = cliREPEAT;
    } else {

        // Print from the 11th newline (10 lines left)
        rc = f_read(&Fil, xBuff, sizeof(xBuff)-1, &br);     /* Read a chunk of file */
        if (rc || !br) {
            xWriteBufferLen = sprintf(pcWriteBuffer, "");
            f_close(&Fil);

            sFlag = 0;
            return pdTRUE;                              /* Error or end of file */
        }
        xBuff[br] = '\0';
        xWriteBufferLen = sprintf(pcWriteBuffer, "%s", xBuff);
        xReturn = cliREPEAT;
    }

    return xReturn;
}

static const xCommandLineInput xLs =
{
    ( char * ) "ls",
    ( char * ) "\r\nls: list files on the SD card\r\n",
    prvLsCommand,
    0
};

// Ls command, list files on the SD card
static portBASE_TYPE prvLsCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString ) {
    static short sFlag = 0;
    static FRESULT rc;                 /* Result code */
    static DIR dir;                    /* Directory object */
    static FILINFO fno;                /* File information object */
    portBASE_TYPE xReturn;

    // Close the currently open file and navigate to the root dir
    if (sFlag == 0) {
        vSdCard_CloseLogFile();

        // Open root directory
        rc = f_opendir(&dir, "");
        if (rc) die(rc);

        xWriteBufferLen = sprintf(pcWriteBuffer, "\r\n");

        sFlag++;
        xReturn = cliREPEAT;
    } else {
        rc = f_readdir(&dir, &fno);     /* Read a directory item */
        if (rc || !fno.fname[0]) {      /* Error or end of dir */
            xWriteBufferLen = sprintf(pcWriteBuffer, "");
            sFlag = 0;
            xReturn = pdTRUE;
        } else {
            if (fno.fattrib & AM_DIR)
                xWriteBufferLen = sprintf(pcWriteBuffer, ">>    <dir>  %s\r\n", fno.fname);
            else
                xWriteBufferLen = sprintf(pcWriteBuffer, ">> %8lu  %s\r\n", fno.fsize, fno.fname);
            xReturn = cliREPEAT;
        }
        if (rc) die(rc);
    }

    return xReturn;
}

static const xCommandLineInput xTaskList=
{
    ( char * ) "task_usage",
    ( char * ) "\r\ntask_usage: Show the task usage and statuses\r\n",
    prvTaskListCommand,
    0
};

// Get the task list and print it
static portBASE_TYPE prvTaskListCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString ) {
    portCHAR pcTaskBuffer[cliNUM_TASKS*50];

    vTaskList(pcTaskBuffer);
    xWriteBufferLen = sprintf(pcWriteBuffer, "%s", pcTaskBuffer);

    return pdTRUE;
}

static const xCommandLineInput xReplay=
{
    ( char * ) "replay",
    ( char * ) "\r\nreplay: Do a positional replay of the task\r\n",
    prvReplayCommand,
    1
};

// Task for replaying from a log file
static portBASE_TYPE prvReplayCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString ) {
    FIL Fil;                                // File object
    BYTE Buff[32];                          // File read buffer, number of chars in a line

    FRESULT rc;                             // Result code
    DIR dir;                                // Directory object
    FILINFO fno;                            // File information object
    long lParam_len;                        // Length of arg
    char *cCmd_string;                      // Return arg
    UINT bw, br, i;                         // File counters

    short sFirstRead = 1;                   // Is this the first read from the file
    unsigned char pucTimeString[13] = {0};  // Previous timestring
    short sDegreesX, sDegreesY;             // Degrees X
    long lTimeDifference = 0;               // Degrees Y
    long j;

    if (xSemaphoreTake(xControlServo, 10) == pdFALSE) {
        xWriteBufferLen = sprintf(pcWriteBuffer, "\r\n");
        return pdTRUE;
    }

    // Close the currently open file and navigate to the root dir
    vSdCard_CloseLogFile();

    //Get parameters from command string
    cCmd_string = FreeRTOS_CLIGetParameter(pcCommandString, 1, &lParam_len);

    // Open the file up
    rc = f_open(&Fil, cCmd_string, FA_OPEN_EXISTING | FA_READ);
    if (rc) {
        xWriteBufferLen = sprintf(pcWriteBuffer, "\r\n>> No such file\r\n");
        return pdTRUE;
    }

    // Read data
    for (;;) {
        rc = f_read(&Fil, Buff, sizeof(Buff), &br);     /* Read a chunk of file */
        if (rc || !br) break;                           /* Error or end of file */

        // If the timestamp is different, move the servo
        if (strncmp(pucTimeString, Buff, 12)) {
            if (sFirstRead) {
                lTimeDifference = 0;
                sFirstRead = 0;
            } else {
                lTimeDifference = lTimestampDifference(Buff, pucTimeString);
            }

            strncpy(pucTimeString, Buff, 12);

            // Extract the degrees from the string
            sDegreesX = sStringToShort(Buff+22);
            sDegreesY = sStringToShort(Buff+27);

            sprintf(ucRemoteBuffer, "%s %03d %03d %d\r\n", pucTimeString, sDegreesX, sDegreesY, lTimeDifference);
            remote_printf(ucRemoteBuffer);

            // Delay for the time difference while maintaining the pan/tilt angles
            for (j = 0; j < lTimeDifference/10; j++) {
                vServo_SetPan(sDegreesX);
                vServo_SetTilt(sDegreesY);

                vTaskDelay(10);
            }
        }
    }
    if (rc) die(rc);

    xSemaphoreGive(xControlServo);
    xWriteBufferLen = sprintf(pcWriteBuffer, "\r\nReplay complete\r\n");
    return pdTRUE;
}

static const xCommandLineInput xEncode=
{
    ( char * ) "encode",
    ( char * ) "\r\nencode: Convolution encode a byte (char or hex)\r\n",
    prvEncodeCommand,
    1
};

// Task for encoding a byte
static portBASE_TYPE prvEncodeCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString ) {
    long lParam_len;                            // Length of arg
    char *cCmd_string;                          // Return arg
    unsigned char ucByte = 0;                   // Byte to convert
    unsigned char pucOutput[3] = {0,0,0};       // Output array
    short psState[3] = {0};                       // Initial state to 0
    short i = 0;

    //Get parameters from command string
    cCmd_string = FreeRTOS_CLIGetParameter(pcCommandString, 1, &lParam_len);

    // Convert data string to a number (takes single char or hex)
    if (lParam_len == 1) {
        ucByte = (unsigned char) cCmd_string[0];
    } else if (lParam_len == 4 && cCmd_string[0] == '0' && cCmd_string[1] == 'x'){
        // Convert chars to a string
        for (i = 0; i < 2; i++) {
            // Check what range the char falls in, and compute the output
            // If the first char, multiply by 16
            if ('0' <= cCmd_string[i+2] && cCmd_string[i+2] <= '9') {
                ucByte += (cCmd_string[i+2]-'0')*(i ? 1 : 16);
            } else if ('a' <= cCmd_string[i+2] && cCmd_string[i+2] <= 'f') {
                ucByte += (cCmd_string[i+2]-'a'+10)*(i ? 1 : 16);
            } else if ('A' <= cCmd_string[i+2] && cCmd_string[i+2] <= 'F') {
                ucByte += (cCmd_string[i+2]-'A'+10)*(i ? 1 : 16);
            } else {
                xWriteBufferLen = sprintf(pcWriteBuffer, "\r\n>> Error: bad input\r\n");
                return pdTRUE;
            }
        }
    } else {
        xWriteBufferLen = sprintf(pcWriteBuffer, "\r\n>> Error: bad input\r\n");
        return pdTRUE;
    }

    vEncoder(ucByte, pucOutput, psState);

    xWriteBufferLen = sprintf(pcWriteBuffer, "\r\n>> 0x%02x%02x%02x\r\n", pucOutput[0], pucOutput[1], pucOutput[2]);

    return pdTRUE;
}

static const xCommandLineInput xDecode=
{
    ( char * ) "decode",
    ( char * ) "\r\ndecode: Convolution decode a 6 char hex string\r\n",
    prvDecodeCommand,
    1
};

// Task for decoding a byte
static portBASE_TYPE prvDecodeCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString ) {
    long lParam_len;                            // Length of arg
    char *cCmd_string;                          // Return arg
    long lNum;

    //Get parameters from command string
    cCmd_string = FreeRTOS_CLIGetParameter(pcCommandString, 1, &lParam_len);

    if (cCmd_string[0] == '0' && cCmd_string[1] == 'x' && !sHexStringToNum(cCmd_string+2, 6, &lNum)){
       xWriteBufferLen = sprintf(pcWriteBuffer, "\r\n>> Decoded: %c\r\n", ucDecoder((unsigned char *) cCmd_string+2));
   } else {
       xWriteBufferLen = sprintf(pcWriteBuffer, "\r\n>> Bad Input\r\n");
   }
    return pdTRUE;
}

static const xCommandLineInput xDistance=
{
    ( char * ) "dist",
    ( char * ) "\r\ndist: distance in mm between fiducials\r\n",
    prvDistanceCommand,
    1
};

// Set the distance between fiducial markers in mm
static portBASE_TYPE prvDistanceCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString ) {
    long lParam_len;                            // Length of arg
    char *cCmd_string;                          // Return arg
    long lNum;

    //Get parameters from command string
    cCmd_string = FreeRTOS_CLIGetParameter(pcCommandString, 1, &lParam_len);

    sFiducialDistance = sStringToShort((unsigned char *) cCmd_string);
    //xWriteBufferLen = sprintf(pcWriteBuffer, "\r\n>> Fiducial distancing set to %dmm\r\n", sFiducialDistance);

    if (sFiducialDistance) {
        xWriteBufferLen = sprintf(pcWriteBuffer, "\r\n>> Fiducial distancing on\r\n");
    } else {
        xWriteBufferLen = sprintf(pcWriteBuffer, "\r\n>> Fiducial distancing off\r\n");
    }

    return pdTRUE;
}

static const xCommandLineInput xTransmit=
{
    ( char * ) "trans",
    ( char * ) "\r\ntrans: transmit a file via laser\r\n",
    prvTransmitCommand,
    1
};

// Transmit a file over USART
static portBASE_TYPE prvTransmitCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString ) {
    FIL Fil;                                // File object
    BYTE xBuff[20];                         // File buffer

    FRESULT rc;                             // Result code
    DIR dir;                                // Directory object
    FILINFO fno;                            // File information object
    long lParam_len;                        // Length of arg
    char *cCmd_string;                      // Return arg
    UINT bw, br, i;                         // File counters
    unsigned char ucBuffChar;

    // Close the currently open file
    vSdCard_CloseLogFile();

    //Get parameters from command string
    cCmd_string = FreeRTOS_CLIGetParameter(pcCommandString, 1, &lParam_len);

    // Open the file up
    rc = f_open(&Fil, cCmd_string, FA_OPEN_EXISTING | FA_READ);
    if (rc) {
        xWriteBufferLen = sprintf(pcWriteBuffer, "\r\n>> No such file: %s\r\n", cCmd_string);
        return pdTRUE;
    }

    //debug_printf("\r\nTransmitting...\r\n");

    // Header transmit
    sUsartTransmit(1);
    vTaskDelay(50);

    // Filename transmit
    for (i=0; i<lParam_len; i++) {    //i=0, lParam_len+2
        //debug_printf("%c", xBuff[i]);
        sUsartTransmit(cCmd_string[i]);
        vTaskDelay(50);
    }

    sUsartTransmit(2);
    vTaskDelay(2000);

    //debug_printf("\r\n");

    // Read data and send
    for (;;) {
        rc = f_read(&Fil, &ucBuffChar, sizeof(ucBuffChar), &br);     /* Read a chunk of file */
        if (rc || !br) break;                                       /* Error or end of file */
        //debug_printf("%c", ucBuffChar);
        sUsartTransmit(ucBuffChar);
        vTaskDelay(50);
    }
    if (rc) die(rc);

    //debug_printf("\r\n");

    // Iterate over the footer
    sprintf(xBuff, "\x03\x04", cCmd_string);
    for (i=0; i<2; i++) {
        //debug_printf("%c", xBuff[i]);
        sUsartTransmit(xBuff[i]);
        vTaskDelay(50);
    }

    xWriteBufferLen = sprintf(pcWriteBuffer, "\r\nTransmission complete.\r\n");

    return pdTRUE;
}

// Configure the CLI
void vCli_Configure() {

    // Create semaphores
    vSemaphoreCreateBinary(xControlServo);

    // Set the fiducial distance
    sFiducialDistance = 0;

    // Allow logging
    sCloseLog = 0;

    // Configure pins the CLI uses
    PIO_Configure(pins, PIO_LISTSIZE(pins));
    vConfigureUsart();

    // Add in the commands
    FreeRTOS_CLIRegisterCommand(&xEcho);
    FreeRTOS_CLIRegisterCommand(&xTest);
    FreeRTOS_CLIRegisterCommand(&xEject);
    FreeRTOS_CLIRegisterCommand(&xLaser);
    FreeRTOS_CLIRegisterCommand(&xTail);
    FreeRTOS_CLIRegisterCommand(&xLs);
    FreeRTOS_CLIRegisterCommand(&xTaskList);
    FreeRTOS_CLIRegisterCommand(&xReplay);
    FreeRTOS_CLIRegisterCommand(&xEncode);
    FreeRTOS_CLIRegisterCommand(&xDecode);
    FreeRTOS_CLIRegisterCommand(&xDistance);
    FreeRTOS_CLIRegisterCommand(&xTransmit);
}
