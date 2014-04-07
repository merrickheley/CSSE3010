 /* -----------------------------------------------------------------------------
# Company name: CSSE3010 - Embedded Systems Design & Interfacing
# Project name: Project 3
# Module name : magLibrary
# Functionality: Support functions for magnetometer usage
# Hardware platform: Netduino (AT91SAM7)
#
# Author name: Merrick Heley
# Creation date: 2013-05-30

------------------------------------------------------------------------------*/

#include <board.h>
#include <pio/pio.h>

#include "magLibrary.h"
#include "hmc5883l.h"
#include "../i2cLibrary.h"

// COnfigure magnetometer
void vMagConfigure(void) {
    sMagEnabled = 0;

    TWI_Config();
    hmc5883l_init();
}
