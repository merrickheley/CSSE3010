 /* -----------------------------------------------------------------------------
# Company name: CSSE3010 - Embedded Systems Design & Interfacing
# Project name: Project 3
# Module name : accelTask
# Functionality: Support functions for radio usage
# Hardware platform: Netduino (AT91SAM7)
#
# Author name: Merrick Heley
# Creation date: 2013-05-18

------------------------------------------------------------------------------*/

#ifndef __ACCELTASK_H__
#define __ACCELTASK_H__

/* ------------------ Library includes  ------------------ */
#include <FreeRTOS.h>
#include <task.h>

/* ------------------ Defines           ------------------ */
#define acceltaskACCEL_PRIORITY         ( tskIDLE_PRIORITY + 1 )
#define acceltaskACCEL_TASK_STACK       ( configMINIMAL_STACK_SIZE * 3)

short *psLastTempData;

/* ------------------ Public Functions  ------------------ */

/* Task for reading data from the accelerometer */
void vAccelTask(void *pvParameters);

#endif /* __ACCELTASK_H__ */
