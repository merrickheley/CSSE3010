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

#include "usart.h"

// Transmit a char over usart
short sUsartTransmit(unsigned char ucSend) {
    return USART_WriteBuffer(AT91C_BASE_US0, &ucSend, 1);
}


// Configures USART0 in hardware handshaking mode, asynchronous, 8 bits, 1 stop
// bit, no parity, 2400 bauds and enables its transmitter and receiver.
void vConfigureUsart(void) {
    unsigned int mode = AT91C_US_USMODE_NORMAL
                        | AT91C_US_CLKS_CLOCK
                        | AT91C_US_CHRL_8_BITS
                        | AT91C_US_PAR_NONE
                        | AT91C_US_NBSTOP_1_BIT
                        | AT91C_US_CHMODE_NORMAL;

    // Enable the peripheral clock in the PMC
    PMC_EnablePeripheral(AT91C_ID_US0);

    // Configure the USART in the desired mode @2400 bauds
    USART_Configure(AT91C_BASE_US0, mode, 2400, BOARD_MCK);

    // Enable receiver & transmitter
    USART_SetTransmitterEnabled(AT91C_BASE_US0, 1);
    USART_SetReceiverEnabled(AT91C_BASE_US0, 1);
}
