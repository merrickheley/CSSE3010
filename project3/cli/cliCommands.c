 /* -----------------------------------------------------------------------------
# Company name: CSSE3010 - Embedded Systems Design & Interfacing
# Project name: Project 3
# Module name : cliCommands
# Functionality: Creates the CLI commands
# Hardware platform: Netduino (AT91SAM7)
#
# Author name: Merrick Heley
# Creation date: 2013-05-13
------------------------------------------------------------------------------*/

#include "cliCommands.h"

/* Structure that defines the "echo" command line command. */
static const xCommandLineInput xEcho =
{
    ( char * ) "echo",
    ( char * ) "\r\necho: Echo the input.\r\n",
    prvEchoCommand,
    1
};

/* Structure that defines the "send" CLI command */
static const xCommandLineInput xSend =
{
    ( char * ) "send",
    ( char * ) "\r\nsend: Send data over radio.\r\n",
    prvSendCommand,
    1
};

/* Structure that defines the periodic printing CLI command */
static const xCommandLineInput xPeriodic =
{
    ( char * ) "pd",
    ( char * ) "\r\npd: Toggle periodic display.\r\n",
    prvPerioidCommand,
    0
};

/* Structure that defines the debugprinting CLI command */
static const xCommandLineInput xDebug =
{
    ( char * ) "dbg",
    ( char * ) "\r\dbg: Toggle debug display.\r\n",
    prvDebugCommand,
    0
};

/* Structure that defines the get pass key CLI command */
static const xCommandLineInput xGetPassKey =
{
    ( char * ) "getPassKey",
    ( char * ) "\r\getPassKey: Get the pass key unique to the student ID.\r\n",
    prvPassKeyCommand,
    0
};

/* Structure that defines the magX command */
static const xCommandLineInput xGetMagX =
{
    ( char * ) "getMagX",
    ( char * ) "\r\getMagX: Get the magnetometer X value\r\n",
    prvMagXCommand,
    0
};

/* Structure for the eject command */
static const xCommandLineInput xEject =
{
    ( char * ) "eject",
    ( char * ) "\r\neject: Safely eject the SD card\r\n",
    prvEjectCommand,
    0
};

/* Structure that defines the motors command */
static const xCommandLineInput xMotors =
{
    ( char * ) "motors",
    ( char * ) "\r\motors: toggle the motors on/off\r\n",
    prvMotorsCommand,
    0
};

/* Structure for the height command */
static const xCommandLineInput xHeight =
{
    ( char * ) "height",
    ( char * ) "\r\nheight: set the height AGL (2 hex chars)\r\n",
    prvHeightCommand,
    1
};

/* Structure for the task 5 command */
static const xCommandLineInput xTask5 =
{
    ( char * ) "task5",
    ( char * ) "\r\ntask5: Change to task 5\r\n",
    prvTask5Command,
    0
};

/* Structure for the mag command */
static const xCommandLineInput xMag =
{
    ( char * ) "mag",
    ( char * ) "\r\nmag: Toggle the magnetometer\r\n",
    prvMagCommand,
    0
};

/* Structure for the task 5 command */
static const xCommandLineInput xReplay =
{
    ( char * ) "replay",
    ( char * ) "\r\nreplay a file\r\n",
    prvReplayCommand,
    1
};

void vConfigureCli() {

    // Add in the commands
    FreeRTOS_CLIRegisterCommand(&xEcho);
    FreeRTOS_CLIRegisterCommand(&xSend);
    FreeRTOS_CLIRegisterCommand(&xPeriodic);
    FreeRTOS_CLIRegisterCommand(&xDebug);
    FreeRTOS_CLIRegisterCommand(&xGetPassKey);
    FreeRTOS_CLIRegisterCommand(&xGetMagX);
    FreeRTOS_CLIRegisterCommand(&xEject);
    FreeRTOS_CLIRegisterCommand(&xMotors);
    FreeRTOS_CLIRegisterCommand(&xHeight);
    FreeRTOS_CLIRegisterCommand(&xTask5);
    FreeRTOS_CLIRegisterCommand(&xMag);
    FreeRTOS_CLIRegisterCommand(&xReplay);
}
