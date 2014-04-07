 /* -----------------------------------------------------------------------------
# Company name: CSSE3010 - Embedded Systems Design & Interfacing
# Project name: Project 3
# Module name : accelLibrary
# Functionality: Support functions for radio usage
# Hardware platform: Netduino (AT91SAM7)
#
# Author name: Merrick Heley
# Creation date: 2013-05-18

------------------------------------------------------------------------------*/

#include <board.h>
#include <pio/pio.h>

#include "../i2cLibrary.h"
#include "accelLibrary.h"
#include "mma8452.h"

void vAccelConfigure(void) {
    TWI_Config();
    mma8452_init();
}
