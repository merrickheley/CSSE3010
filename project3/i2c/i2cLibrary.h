 /* -----------------------------------------------------------------------------
# Company name: CSSE3010 - Embedded Systems Design & Interfacing
# Project name: Project 3
# Module name : i2c
# Functionality: i2c configuration
# Hardware platform: Netduino (AT91SAM7)
#
# Author name: Merrick Heley
# Creation date: 2013-05-30

------------------------------------------------------------------------------*/

#ifndef __I2CLIBRARY_H__
#define __I2CLIBRARY_H__

#include <twi/twi.h>
#include <drivers/twi/twid.h>

// TWI clock frequency in Hz.
#define TWCK            400000

Twid twid;

/* ------------------ Public Functions  ------------------ */
void ISR_Twi(void);
void TWI_Config();



#endif /* __I2CLIBRARY_H__ */
