 /* -----------------------------------------------------------------------------
# Company name: CSSE3010 - Embedded Systems Design & Interfacing
# Project name: Project 3
# Module name : radioTask
# Functionality: freeRTOS radio task
# Hardware platform: Netduino (AT91SAM7)
#
# Author name: Merrick Heley
# Creation date: 2013-05-13
------------------------------------------------------------------------------*/

#ifndef RADIOTASK_H_
#define RADIOTASK_H_

/* ------------------ Library includes  ------------------ */
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

/* ------------------ Local includes    ------------------ */
#include "radioLibrary.h"

/* ------------------ Defines           ------------------ */
#define radiotaskRADIO_PRIORITY             ( tskIDLE_PRIORITY + 1 )
#define radiotaskRADIO_PRINT_PRIORITY       ( tskIDLE_PRIORITY + 1 )

#define radiotaskRADIO_TASK_STACK           ( configMINIMAL_STACK_SIZE * 3 )
#define radiotaskRADIO_PRINT_TASK_STACK     ( configMINIMAL_STACK_SIZE * 3 )


/* ------------------ Public Functions  ------------------ */

/* Radio task for freeRTOS */
void vRadioTask();

/* Radio print task for freeRTOS */
void vRadioPrintTask();

#endif /* RADIOTASK_H_ */
