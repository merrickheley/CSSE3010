 /* -----------------------------------------------------------------------------
# Company name: CSSE3010 - Embedded Systems Design & Interfacing
# Project name: Project 3
# Module name : cliTask
# Functionality: Command line interface task
# Hardware platform: Netduino (AT91SAM7)
#
# Author name: Merrick Heley
# Creation date: 2013-05-13
------------------------------------------------------------------------------*/

#ifndef __CLITASK_H__
#define __CLITASK_H__


/* ------------------ Defines           ------------------ */
#define clitaskCLI_PRIORITY       ( tskIDLE_PRIORITY + 1 )
#define clitaskCLI_TASK_STACK     ( configMINIMAL_STACK_SIZE * 6 )

/*
 * USB receive task for CLI
 */
void vCliTask(void *pvParameters);

#endif /* __CLITASK_H__ */
