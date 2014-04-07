 /* -----------------------------------------------------------------------------
# Company name: CSSE3010 - Embedded Systems Design & Interfacing
# Project name: Project 2
# Module name : Servo 
# Functionality: PWM servo control 
# Hardware platform: Netduino (AT91SAM7)
#
# Author name: Merrick Heley
# Creation date: 2013-04-21
------------------------------------------------------------------------------*/

#ifndef __SERVO_H__
#define __SERVO_H__

//------------------------------------------------------------------------------
//         Definitions and variables
//------------------------------------------------------------------------------

#define servoMAX_DEGREES   245       // Servo limits
#define servoMIN_DEGREES   55        // Changed from 255 and 50

//------------------------------------------------------------------------------
//         Public function definitions
//------------------------------------------------------------------------------

void vServo_ConfigurePwm(void);           // Configure the PWM
void vServo_SetPan(short sDegrees);       // Set servo1 to a specified degree
void vServo_SetTilt(short sDegrees);      // Set servo2 to a specified degree

#endif
