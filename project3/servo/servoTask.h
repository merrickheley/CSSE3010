 /* -----------------------------------------------------------------------------
# Company name: CSSE3010 - Embedded Systems Design & Interfacing
# Project name: Project 3
# Module name : servoTask
# Functionality: freeRTOS servo task
# Hardware platform: Netduino (AT91SAM7)
#
# Author name: Merrick Heley
# Creation date: 2013-05-19

------------------------------------------------------------------------------*/

#ifndef __SERVOTASK_H__
#define __SERVOTASK_H__

/* ------------------ Library includes  ------------------ */
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

/* ------------------ Local includes    ------------------ */
#include "servoLibrary.h"

/* ------------------ Defines           ------------------ */
#define servotaskSERVO_PRIORITY             ( tskIDLE_PRIORITY + 1 )
#define servotaskSERVO_TASK_STACK           ( configMINIMAL_STACK_SIZE * 3 )

/* Fist number of the set of 3 to track */
#define servoFIDUCIAL_SET      0

/* ------------------ Public Functions  ------------------ */

/* Servo task for freeRTOS */
void vServoTask();

/* Clear saved fiducial values */
void vServoClearTask();


#endif /* __SERVOTASK_H__ */
