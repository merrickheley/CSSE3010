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

#include "i2cLibrary.h"
#include <pio/pio.h>

static short sTWIConfigured = 0;

// TWI interrupt handler. Forwards the interrupt to the TWI driver handler.
void ISR_Twi(void)
{
    TWID_Handler(&twid);
}

// Configure TWI
void TWI_Config() {
    if (!sTWIConfigured) {

        static const Pin pinsMux[] = {PIN_MUX1, PIN_MUX2};
        static const Pin pins[] = { PINS_TWI };

        PIO_Configure(pins, PIO_LISTSIZE(pins));

        // Enable MUX -- Turns Analog Pins 4&5 into TWI.
        PIO_Configure(pinsMux, PIO_LISTSIZE(pinsMux));
        PIO_Clear(&pinsMux[0]);
        PIO_Clear(&pinsMux[1]);

        AT91C_BASE_PMC->PMC_PCER = 1 << AT91C_ID_TWI;
        TWI_ConfigureMaster(AT91C_BASE_TWI, TWCK, BOARD_MCK);
        TWID_Initialize(&twid, AT91C_BASE_TWI);
        AIC_ConfigureIT(AT91C_ID_TWI, 0, ISR_Twi);
        AIC_EnableIT(AT91C_ID_TWI);

        sTWIConfigured = 1;
    }
}
