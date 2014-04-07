 /* -----------------------------------------------------------------------------
# Company name: CSSE3010 - Embedded Systems Design & Interfacing
# Project name: Project 2
# Module name : usart
# Functionality: gives the usart encoder commands
# Hardware platform: Netduino (AT91SAM7)
#
# Author name: Merrick Heley
# Creation date: 2013-05-04
------------------------------------------------------------------------------*/

#ifndef USART_H_
#define USART_H_

#include <usart/usart.h>
#include <pmc/pmc.h>

// Configure usart
void vConfigureUsart(void);
// Transmit a char over usart
short sUsartTransmit(unsigned char ucSend);

#endif /* USART_H_ */
