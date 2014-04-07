 /* -----------------------------------------------------------------------------
# Company name: CSSE3010 - Embedded Systems Design & Interfacing
# Project name: Project 3
# Module name : Main
# Functionality: Main file
# Hardware platform: Netduino (AT91SAM7)
#
# Author name: Merrick Heley
# Creation date: 2013-05-12

------------------------------------------------------------------------------*/

/*
	NOTE : Tasks run in System mode and the scheduler runs in Supervisor mode.
	The processor MUST bedemo application in supervisor mode when vTaskStartScheduler is
	called.  The demo applications included in the FreeRTOS.org download switch
	to supervisor mode prior to main being called.  If you are not using one of
	these demo application projects then ensure Supervisor mode is used.
*/

/* Standard includes. */
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "board.h"

/* application includes. */
#include <uip.h>
#include <uIP_Task.h>
#include <USB-CDC.h>
#include <dynamic.h>
#include <debug_printf.h>
#include "BlockQ.h"
#include "blocktim.h"
#include "flash.h"
#include "QPeek.h"
#include "dynamic.h"
#include "semphr.h"

#include <board.h>
#include <pio/pio.h>
#include <adc/adc.h>
#include <usart/usart.h>
#include <pmc/pmc.h>
#include <pwmc/pwmc.h>

#include <utility/led.h>
#include <utility/trace.h>


/* Local includes */
#include "./radio/radioTask.h"
#include "./cli/cliTask.h"
#include "./i2c/accel/accelTask.h"
#include "./i2c/mag/magTask.h"
#include "./servo/servoTask.h"
#include "./uip/tuioclient.h"

/* Priorities for the tasks. */
#define mainUSB_PRIORITY					( tskIDLE_PRIORITY + 3 )
#define mainUIP_PRIORITY                    ( tskIDLE_PRIORITY + 3 )
/* Task Stack Allocations */
#define mainUSB_TASK_STACK				    ( 200 )
#define mainUIP_TASK_STACK                  ( configMINIMAL_STACK_SIZE * 3 )

/*-----------------------------------------------------------*/

/*
 * Configure the processor for use with the NetduinoPlus board.  Setup is minimal
 * as the low level init function (called from the startup asm file) takes care
 * of most things.
 */
static void prvSetupHardware( void );

/*
 * The idle hook is to blink the blue LED.
 */
void vApplicationIdleHook( void );

/*
 * Starts all the other tasks, then starts the scheduler.
 */
int main( void )
{
	/* Setup any hardware that has not already been configured by the low
	level init routines. */
	prvSetupHardware();
	
	/* Create the USB CDC Serial task - used for debug-printf. */
	xTaskCreate( vUSBCDCTask,   (    signed char * ) "USB",         mainUSB_TASK_STACK,              NULL, mainUSB_PRIORITY, NULL );
	
	/* Create the CLI task */
	xTaskCreate( vCliTask,          (signed char * ) "CLI",         clitaskCLI_TASK_STACK,           NULL, clitaskCLI_PRIORITY, NULL );

	/* Create the Radio task */
	xTaskCreate( vRadioTask,        ( signed char * ) "RADIO",      radiotaskRADIO_TASK_STACK,       NULL, radiotaskRADIO_PRIORITY, NULL );

	/* Create the radio print task */
    xTaskCreate( vRadioPrintTask,   ( signed char * ) "RADIODISP",  radiotaskRADIO_PRINT_TASK_STACK, NULL, radiotaskRADIO_PRINT_PRIORITY, NULL );

    /* Create the accelerometer task */
    xTaskCreate( vAccelTask,        ( signed char * ) "ACCEL",      acceltaskACCEL_TASK_STACK,       NULL, acceltaskACCEL_PRIORITY, NULL );

    /* Create the magnetometer task */
    xTaskCreate( vMagTask,          ( signed char * ) "MAG",        magtaskACCEL_TASK_STACK,         NULL, magtaskACCEL_PRIORITY, NULL );

    /* Create the vision control and tracking task */
    xTaskCreate( vServoTask,        ( signed char * ) "SERVO",      servotaskSERVO_TASK_STACK,       NULL, servotaskSERVO_PRIORITY, NULL );
    xTaskCreate( vServoClearTask,   ( signed char * ) "SERVOCL",    servotaskSERVO_TASK_STACK,       NULL, servotaskSERVO_PRIORITY, NULL );

    /* Start the uIP task */
    xTaskCreate( vuIP_Task,         ( signed char * ) "UIP",        mainUIP_TASK_STACK,              NULL, mainUIP_PRIORITY, NULL );

	/* Start the scheduler.

	NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
	The processor MUST be in supervisor mode when vTaskStartScheduler is
	called.  The demo applications included in the FreeRTOS.org download switch
	to supervisor mode prior to main being called.  If you are not using one of
	these demo application projects then ensure Supervisor mode is used here. */

	vTaskStartScheduler();

	/* We should never get here as control is now taken by the scheduler. */
  	return 0;
}

/*-----------------------------------------------------------*/
/* Hardware Initialisation */
/* DO NOT MODIFY */
static void prvSetupHardware( void )
{
	portDISABLE_INTERRUPTS();
	
	/* When using the JTAG debugger the hardware is not always initialised to
	the correct default state.  This line just ensures that this does not
	cause all interrupts to be masked at the start. */
	AT91C_BASE_AIC->AIC_EOICR = 0;
	
	/* Most setup is performed by the low level init function called from the
	startup asm file. */

	/* Enable the peripheral clock. */
    AT91C_BASE_PMC->PMC_PCER = 1 << AT91C_ID_PIOA;
    AT91C_BASE_PMC->PMC_PCER = 1 << AT91C_ID_PIOB;
	AT91C_BASE_PMC->PMC_PCER = 1 << AT91C_ID_EMAC;

	/* Initialise the LED outputs for use by the demo application tasks. */
	LED_Configure(0);
}

/*-----------------------------------------------------------*/
/* Idle Application Task */
/* DO NOT MODIFY */
void vApplicationIdleHook( void )
{
	static portTickType xLastTx = 0;

	/* The idle hook simply prints the idle tick count */
	if( ( xTaskGetTickCount() - xLastTx ) > (1000 / portTICK_RATE_MS))
	{
		xLastTx = xTaskGetTickCount();
		
		LED_Toggle(0);		
	}
}
