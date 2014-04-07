 /* -----------------------------------------------------------------------------
# Company name: CSSE3010 - Embedded Systems Design & Interfacing
# Project name: Project 3
# Module name : cliCallbacks
# Functionality: Creates the CLI commands
# Hardware platform: Netduino (AT91SAM7)
#
# Author name: Merrick Heley
# Creation date: 2013-05-13
------------------------------------------------------------------------------*/

#ifndef __CLICOMMANDS_H__
#define __CLICOMMANDS_H__

#include "cliCallbacks.h"

/* Return this to call code repeatedly */
#define clicommandREPEAT 2

/*
 * Configure CLI for use
 */
void vConfigureCli();

#endif /* __CLICOMMANDS_H__ */
