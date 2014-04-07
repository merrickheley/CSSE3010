/* -----------------------------------------------------------------------------
# Company name: CSSE3010 - Embedded Systems Design & Interfacing
# Project name: Project 2
# Module name : remote
# Functionality: remote logging functions
# Hardware platform: Netduino (AT91SAM7)
#
# Author name: Merrick Heley
# Creation date: 2013-05-05
------------------------------------------------------------------------------*/

#include "remote.h"

#include <FreeRTOS.h>
#include <board.h>
#include <stdarg.h>
#include <string.h>
#include <ctype.h>
#include <stdio.h>
#include <stdio.h>

#include "queue.h"

#include "uip.h"
#include "USB-CDC.h"

#include "convolution.h"
#include "debug_printf.h";

// Send data over the remote logger
void vRemoteSend(unsigned char *pucInput, short sLen) {
    unsigned char pucOutput[3] = {0,0,0};           // Output array
    static short psState[3] = {0};                  // Initial state to 0
    xLogStruct xTemp = {{0}, 0};
    int i;

    // Encode the data
    for (i=0; i<sLen; i++) {
            //debug_printf("%c", pucInput[i]);
            vEncoder(pucInput[i], pucOutput, psState);
            xTemp.pucLine[i*3 + 0] = pucOutput[2];
            xTemp.pucLine[i*3 + 1] = pucOutput[1];
            xTemp.pucLine[i*3 + 2] = pucOutput[0];
    }
    // Set the len and add to queue
    xTemp.sLen = sLen*3;
    //debug_printf("\r\n***%d***%s***", xTemp.pucLine, xTemp.sLen);
    xQueueSendToBack(xLogQueue, &xTemp, 0);
}

// Print and send to remote and debug_printf
// Not to RTOS conventions, matches debug_printf
void remote_printf (unsigned char *pucInput) {
    debug_printf(pucInput);
    vRemoteSend(pucInput, strlen(pucInput));
}
