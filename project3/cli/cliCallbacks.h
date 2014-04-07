 /* -----------------------------------------------------------------------------
# Company name: CSSE3010 - Embedded Systems Design & Interfacing
# Project name: Project 3
# Module name : cli
# Functionality: Creates the CLI command callbacks
# Hardware platform: Netduino (AT91SAM7)
#
# Author name: Merrick Heley
# Creation date: 2013-05-13
------------------------------------------------------------------------------*/

#ifndef __CLICALLBACKS_H__
#define __CLICALLBACKS_H__

/* ------------------ Local includes    ------------------ */
#include "FreeRTOS.h"
#include "FreeRTOS_CLI.h"

/* ------------------ Public Variables  ------------------ */
unsigned char ucPeriodicPrinting;
unsigned char ucDebugPrinting;

/* ------------------ Public Functions  ------------------ */

/* Echo whatever argument is given */
portBASE_TYPE prvEchoCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );

/* Transmit the argument over radio */
portBASE_TYPE prvSendCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );

/* Toggle data periodic printing */
portBASE_TYPE prvPerioidCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );

/* Toggle data debug printing */
portBASE_TYPE prvDebugCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );

/* Request the pass key */
portBASE_TYPE prvPassKeyCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );

/* Request the pass key */
portBASE_TYPE prvMagXCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );

/* Eject the SD card safely, no arguments */
portBASE_TYPE prvEjectCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );

/* Enable or disable the motors */
portBASE_TYPE prvMotorsCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );

/* Set the height above ground level */
portBASE_TYPE prvHeightCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );

/* Change to task 5 */
portBASE_TYPE prvTask5Command(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );

/* Turn on mag */
portBASE_TYPE prvMagCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );

/* Replay a file */
portBASE_TYPE prvReplayCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );

#endif
