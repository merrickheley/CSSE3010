 /* -----------------------------------------------------------------------------
# Company name: CSSE3010 - Embedded Systems Design & Interfacing
# Project name: Project 3
# Module name : servoLibrary
# Functionality: PWM servo control 
# Hardware platform: Netduino (AT91SAM7)
#
# Author name: Merrick Heley
# Creation date: 2013-04-21
------------------------------------------------------------------------------*/

#ifndef __SERVO_H__
#define __SERVO_H__

#include <FreeRTOS.h>
#include <semphr.h>

//------------------------------------------------------------------------------
//         Definitions and variables
//------------------------------------------------------------------------------

#define servoMAX_DEGREES   240       // Servo limits
#define servoMIN_DEGREES   60        // Changed from 255 and 50

// Replay will take this when it controls the servo
xSemaphoreHandle xControlServo;

// Task 5 variable
short sTask5;

//------------------------------------------------------------------------------
//         Public function definitions
//------------------------------------------------------------------------------

// Configure the PWM
void vServo_ConfigurePwm(unsigned short usServo1Degrees, unsigned short usServo2Degrees);
void vServo_SetPan(short sDegrees);       // Set servo1 to a specified degree
void vServo_SetTilt(short sDegrees);      // Set servo2 to a specified degree

#endif
