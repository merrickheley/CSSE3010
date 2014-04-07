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

#ifndef __CLI_H__
#define __CLI_H__

#include "FreeRTOS.h"
#include "FreeRTOS_CLI.h"
#include "semphr.h"

#define cliREPEAT 2             // Return this to call code repeatedly
#define cliTAIL 10              // Length of tail
#define cliNUM_TASKS 6          // Number of tasks running
#define cliTEMP_BUFFER_SIZE 20  // Size of the temp buffer

// Replay will take this when it controls the servo
xSemaphoreHandle xControlServo;

short sFiducialDistance;

/* Command functions used for CLI commands */
// Echo whatever argument is given
static portBASE_TYPE prvEchoCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
// Output test data, no arguments
static portBASE_TYPE prvTestCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
// Eject the SD card safely, no arguments
static portBASE_TYPE prvEjectCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
// List the files on the SD card
static portBASE_TYPE prvLsCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
// Turn the laser on or off
static portBASE_TYPE prvLaserCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
// List the last 10 lines of a file
static portBASE_TYPE prvTailCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
// Show the task list
static portBASE_TYPE prvTaskListCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
// Replay the positions from a log file
static portBASE_TYPE prvReplayCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
// Convolution encode a byte
static portBASE_TYPE prvEncodeCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
// Convolution decode a string of hex chars
static portBASE_TYPE prvDecodeCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
// Set the distance between fiducial markers
static portBASE_TYPE prvDistanceCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
// Transmit a file over USART
static portBASE_TYPE prvTransmitCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );

void vCli_Configure();

#endif
