 /* -----------------------------------------------------------------------------
# Company name: CSSE3010 - Embedded Systems Design & Interfacing
# Project name: Project 3
# Module name : hmc5883l
# Functionality: hmc5883l library
# Hardware platform: Netduino (AT91SAM7)
#
# Author name: Merrick Heley
# Creation date: 2013-05-30
------------------------------------------------------------------------------*/

#include <twi/twi.h>
#include <drivers/twi/twid.h>
#include "FreeRTOS.h"
#include "queue.h"


// TWI clock frequency in Hz.
#define TWCK            400000

// Slave address.
#define hmc5883l_ADDRESS              0x1E

xQueueHandle xMagnetometer_queue;

extern const unsigned char SCALE;

extern const unsigned char dataRate;

extern Twid twid;

extern void ISR_Twi(void);

extern int hmc5883l_isconnect(void);

extern void hmc5883l_readData(void);

extern void hmc5883l_init();
