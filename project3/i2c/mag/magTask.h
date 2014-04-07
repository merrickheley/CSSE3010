 /* -----------------------------------------------------------------------------
# Company name: CSSE3010 - Embedded Systems Design & Interfacing
# Project name: Project 3
# Module name : magTask
# Functionality: magnetometer task
# Hardware platform: Netduino (AT91SAM7)
#
# Author name: Merrick Heley
# Creation date: 2013-05-30

------------------------------------------------------------------------------*/

#ifndef __MAGTASK_H__
#define __MAGTASK_H__

/* ------------------ Library includes  ------------------ */
#include <FreeRTOS.h>
#include <task.h>

/* ------------------ Defines           ------------------ */
#define magtaskACCEL_PRIORITY         ( tskIDLE_PRIORITY + 1 )
#define magtaskACCEL_TASK_STACK       ( configMINIMAL_STACK_SIZE * 3)

/* ------------------ Public Functions  ------------------ */

/* Task for reading data from the accelerometer */
void vMagTask(void *pvParameters);

#endif /* __MAGTASK_H__ */
