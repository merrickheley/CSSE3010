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
#include <debug_printf.h>

#include "hmc5883l.h"
#include "../i2cLibrary.h"

#define hmc5883lREG_CONFIG_A          0x00
#define hmc5883lREG_CONFIG_B          0x01
#define hmc5883lREG_MODE              0x02
#define hmc5883lREG_X_MSB             0x03
#define hmc5883lREG_X_LSB             0x04
#define hmc5883lREG_Z_MSB             0x05
#define hmc5883lREG_Z_LSB             0x06
#define hmc5883lREG_Y_MSB             0x07
#define hmc5883lREG_Y_LSB             0x08
#define hmc5883lREG_STATUS            0x09
#define hmc5883lREG_ID_A              0x0A
#define hmc5883lREG_ID_B              0x0B
#define hmc5883lREG_ID_C              0x0C

#define hmc5883lSHIFT_MODE_AVERAGE    0x05
#define hmc5883lMODE_AVERAGE_1        0x00
#define hmc5883lMODE_AVERAGE_2        0x01
#define hmc5883lMODE_AVERAGE_4        0x02
#define hmc5883lMODE_AVERAGE_8        0x03

#define hmc5883lSHIFT_MODE_FREQ       0x02
#define hmc5883lMODE_FREQ_0P75        0x00
#define hmc5883lMODE_FREQ_1P5         0x01
#define hmc5883lMODE_FREQ_3           0x02
#define hmc5883lMODE_FREQ_7P5         0x03
#define hmc5883lMODE_FREQ_15          0x04
#define hmc5883lMODE_FREQ_30          0x05
#define hmc5883lMODE_FREQ_75          0x06

#define hmc5883lREG_GAIN_1370         0x00
#define hmc5883lREG_GAIN_1090         0x20
#define hmc5883lREG_GAIN_820          0x40
#define hmc5883lREG_GAIN_660          0x60
#define hmc5883lREG_GAIN_440          0x80
#define hmc5883lREG_GAIN_390          0xA0
#define hmc5883lREG_GAIN_330          0xC0
#define hmc5883lREG_GAIN_220          0xE0

#define hmc5883lREG_MODE_CONTINUOUS   0x00
#define hmc5883lREG_MODE_SINGLE       0x01
#define hmc5883lREG_MODE_IDLE         0x02

// TWI interrupt handler. Forwards the interrupt to the TWI driver handler.
//void ISR_Twi(void)
//{
//   TWID_Handler(&twid);
//}

// Read the WHO_AM_I register, this is a good test of communication
int hmc5883l_isconnect(void)
{
    unsigned char c;
    TWID_Read(&twid, hmc5883l_ADDRESS, hmc5883lREG_ID_C, 1, &c, 1, 0);
    return c == 0x33;
}

// Read accelerometer data from the hmc5883l and put them in accelerometer queue (xAccelerometer_queue)
void hmc5883l_readData(void)
{
    // I have no idea why this makes it work :/
    //hmc5883l_init();

    int i;
    unsigned char rawData[6] = {0};  // x/z/y mag register data stored here
    short mag_value[3] = {0};

    TWID_Read( &twid, hmc5883l_ADDRESS, hmc5883lREG_X_MSB, 1, &rawData[0], 6, 0);

    //debug_printf("raw: %d %d %d %d %d %d\r\n", rawData[0], rawData[1], rawData[2], rawData[3], rawData[4], rawData[5]);

    /* loop to calculate 16-bit ADC value for each axis */
    for (i=0; i<6; i+=2)
    {
        // We first construct the 16 bit two's complement number in the most
        // significant bytes of the short int (destination).
        mag_value[i/2] = ((rawData[i] << 8) | (rawData[i+1] & 0xFF));
    }

    //Put Accelerometer value into xAccelerometer_queue.
    xQueueSendToBack(xMagnetometer_queue, (void *) mag_value, 0);
}

// Initialise the hmc5883l
void hmc5883l_init(void)
{
    unsigned char temp;

    // Config register A (averaging and read speed)
    TWID_Write(&twid, hmc5883l_ADDRESS, hmc5883lREG_CONFIG_A, 1, (hmc5883lMODE_AVERAGE_8 << hmc5883lSHIFT_MODE_AVERAGE) |
                                                                 (hmc5883lMODE_FREQ_15   << hmc5883lSHIFT_MODE_FREQ), 1, 0);

    // Config register B (gain level)
    TWID_Write(&twid, hmc5883l_ADDRESS, hmc5883lREG_CONFIG_B, 1, hmc5883lREG_GAIN_1090, 1, 0);

    // Config register C (continuous mode)
    TWID_Write(&twid, hmc5883l_ADDRESS, hmc5883lREG_MODE,     1, hmc5883lREG_MODE_CONTINUOUS, 1, 0);
}
