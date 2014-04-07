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

#ifndef __MAGLIBRARY_H__
#define __MAGLIBRARY_H__

#include "hmc5883l.h"

short sMagEnabled;

void vMagConfigure();

#endif  /* __MAGLIBRARY_H__ */
