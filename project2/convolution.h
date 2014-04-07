 /* -----------------------------------------------------------------------------
# Company name: CSSE3010 - Embedded Systems Design & Interfacing
# Project name: Project 2
# Module name : convolution
# Functionality: gives the convolution encoder commands
# Hardware platform: Netduino (AT91SAM7)
#
# Author name: Merrick Heley
# Creation date: 2013-04-28
------------------------------------------------------------------------------*/

#ifndef __CONV_H__
#define __CONV_H__

// Convert a hex string to a num, given in lOuput
short sHexStringToNum(char *pucInput, short sLen, long *lOutput);

// Function for encoding a byte, allows for initial state to be set
void vEncoder(unsigned char ucInputByte, unsigned char *pucOutput, short *psState);

// Function for decoding to a byte
unsigned char ucDecoder(unsigned char *pucEncoded);

#endif
