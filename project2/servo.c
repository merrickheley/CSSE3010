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

//------------------------------------------------------------------------------
//         Headers
//------------------------------------------------------------------------------
#include <board.h>
#include <pio/pio.h>
#include <pwmc/pwmc.h>

#include "servo.h"

//------------------------------------------------------------------------------
//         Definitions and variables
//------------------------------------------------------------------------------

// PWM
#define servoCHANNEL_SERVO1              2
#define servoCHANNEL_SERVO2              3

#define servoPWM_FREQUENCY               50
#define servoDUTY_CYCLE                  2000

// Pio pins to configure.
static const Pin pwmPins[] = {
    PIN_PWMC_PWM2,
    PIN_PWMC_PWM3
};

//------------------------------------------------------------------------------
//         Local functions
//------------------------------------------------------------------------------


// Check that the servo is within its range
short sServoDegrees(short deg, int min, int max) {

    if (deg > max) {
        deg = max;
    }
    
    if (deg < min) {
        deg = min;
    }

    return deg;
}

//------------------------------------------------------------------------------
//         Public functions
//------------------------------------------------------------------------------

// Initialise PWM
void vServo_ConfigurePwm(void) {
    PIO_Configure(pwmPins, PIO_LISTSIZE(pwmPins));

    // Enable PWMC peripheral clock
    AT91C_BASE_PMC->PMC_PCER = 1 << AT91C_ID_PWMC;

    // Set clock A for servo's at 50hz * Duty cycle (samples).
    PWMC_ConfigureClocks(2 * servoPWM_FREQUENCY * servoDUTY_CYCLE, 0, BOARD_MCK);

    // Configure PWMC channel for SERVO (center-aligned, inverted polarity)
    PWMC_ConfigureChannel(servoCHANNEL_SERVO1, AT91C_PWMC_CPRE_MCKA, AT91C_PWMC_CALG, AT91C_PWMC_CPOL);
    PWMC_SetPeriod(servoCHANNEL_SERVO1, servoDUTY_CYCLE);
    PWMC_SetDutyCycle(servoCHANNEL_SERVO1, (servoMIN_DEGREES+servoMAX_DEGREES)/2);
    
    PWMC_ConfigureChannel(servoCHANNEL_SERVO2, AT91C_PWMC_CPRE_MCKA, AT91C_PWMC_CALG, AT91C_PWMC_CPOL);
    PWMC_SetPeriod(servoCHANNEL_SERVO2, servoDUTY_CYCLE);
    PWMC_SetDutyCycle(servoCHANNEL_SERVO2, servoMAX_DEGREES);
    
    // Enable channel #1 and #2
    PWMC_EnableChannel(servoCHANNEL_SERVO1);
    PWMC_EnableChannel(servoCHANNEL_SERVO2);
}

// Set the degrees on servo 1
void vServo_SetPan(short sDegrees) {
    PWMC_SetDutyCycle(servoCHANNEL_SERVO1, sServoDegrees(sDegrees, servoMIN_DEGREES, servoMAX_DEGREES));
}

// Set the degrees on servo 2
void vServo_SetTilt(short sDegrees) {
    PWMC_SetDutyCycle(servoCHANNEL_SERVO2, sServoDegrees(sDegrees, servoMIN_DEGREES, servoMAX_DEGREES));
}
