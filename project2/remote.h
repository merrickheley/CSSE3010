/* -----------------------------------------------------------------------------
# Company name: CSSE3010 - Embedded Systems Design & Interfacing
# Project name: Project 2
# Module name : remote
# Functionality: remote logging functions
# Hardware platform: Netduino (AT91SAM7)
#
# Author name: Merrick Heley
# Creation date: 2013-05-05
------------------------------------------------------------------------------*/

#ifndef __REMOTE_H__
#define __REMOTE_H__

unsigned char ucRemoteBuffer[150];

// Send via remote (no printing)
void vRemoteSend ( unsigned char *pucInput, short sLen );
// Print and send a string over remote
void remote_printf (unsigned char *pucInput);


#endif /* REMOTE_H_ */
