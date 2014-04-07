 /* -----------------------------------------------------------------------------
# Company name: CSSE3010 - Embedded Systems Design & Interfacing
# Project name: Project 2
# Module name : Servo 
# Functionality: Main file
# Hardware platform: Netduino (AT91SAM7)
#
# Author name: Merrick Heley
# Creation date: 2013-04-21

------------------------------------------------------------------------------*/


/*
    NOTE : Tasks run in System mode and the scheduler runs in Supervisor mode.
    The processor MUST be in supervisor mode when vTaskStartScheduler is
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
#include "queue.h"
#include "board.h"

/* application includes. */
#include "uip.h"
#include "USB-CDC.h"
#include "uIP_Task.h"
#include "BlockQ.h"
#include "blocktim.h"
#include "flash.h"
#include "QPeek.h"
#include "dynamic.h"
#include "semphr.h"
#include "remote.h"
//#include "FreeRTOS_CLI.h"

#include <pio/pio.h>
#include <utility/led.h>
#include <utility/trace.h>

#include "tuioclient.h"
#include "servo.h"
#include "sdcard.h"
#include "cli.h"
#include "remote.h"

/* Priorities for the demo application tasks. */
#define mainUSB_PRIORITY                    ( tskIDLE_PRIORITY + 1 )
#define mainUIP_PRIORITY                    ( tskIDLE_PRIORITY + 1 )
#define mainREACTV_PRIORITY                 ( tskIDLE_PRIORITY + 1 )
#define mainSDSYNC_PRIORITY                 ( tskIDLE_PRIORITY + 1 )
#define mainUSBCLI_PRIORITY					( tskIDLE_PRIORITY + 3 )
#define mainUSARTRX_PRIORITY                ( tskIDLE_PRIORITY + 1 )

#define mainUSB_TASK_STACK                  ( 200 )
#define mainUIP_TASK_STACK                  ( configMINIMAL_STACK_SIZE * 3 )
#define mainREACTV_TASK_STACK               ( configMINIMAL_STACK_SIZE * 3 )
#define mainSDSYNC_TASK_STACK               ( configMINIMAL_STACK_SIZE * 3 )
#define mainUSBCLI_TASK_STACK   			( configMINIMAL_STACK_SIZE * 3 )
#define mainUSARTRX_TASK_STACK              ( configMINIMAL_STACK_SIZE * 3 )

/* The task allocated to the uIP task is large to account for its use of the
sprintf() library function.  Use of a cut down printf() library would allow
the stack usage to be greatly reduced. */

/* The rate at which the idle hook sends data to the USB port. */
#define mainUSB_TX_FREQUENCY        ( 1000 / portTICK_RATE_MS )

/*-----------------------------------------------------------*/
#define mainF2S_WHOLE(sVar) (short) sVar
#define mainF2S_FRAC(sVar) (short)(sVar*100.0)%100

#define mainPRINT_FLOAT(fVar) (short) fVar, (short)(fVar*100.0)%100

#define mainBOUNDING_BOX_MIN 40
#define mainBOUNDING_BOX_MAX 60

// Compute a to the power of b
short sPow(short a, short b) {
    short i;
    short sRet = 1;
    // multiply the return value by a b times
    for (i=0; i<b; i++) {
        sRet*=a;
    }

    return sRet;
}

// very approximated sqrt
// only useful for small numbers
short sApproxSqrt(short a) {
    short i=0;
    // Keep guessing until the guess is too high
    while (i*i < a) {
        i++;
    }
    return i;
}

/*-----------------------------------------------------------*/

/*
 * Configure the processor for use with the Atmel demo board.  Setup is minimal
 * as the low level init function (called from the startup asm file) takes care
 * of most things.
 */
static void prvSetupHardware( void );

void vReactivision_Task( void );

/* The idle hook is just used to stream data to the USB port.  */
void vApplicationIdleHook( void );

/* USB CLI receive task */
void vUSB_cli_receive_task(void *pvParameters);

void vUsartRx_Task( void );

/*-----------------------------------------------------------*/

/*
 * Starts all the other tasks, then starts the scheduler.
 */
int main( void )
{
    /* Setup any hardware that has not already been configured by the low
    level init routines. */
    prvSetupHardware();

    /* Create the USB CDC Serial task - used for debug-printf. */
    xTaskCreate( vUSBCDCTask, ( signed char * ) "USB", mainUSB_TASK_STACK, NULL, mainUSB_PRIORITY, NULL );

	/* RegisterCLI commands */
    vCli_Configure();
	
    // Configure the PWM
    vServo_ConfigurePwm();
    
    // Load the SD card
    vSdCard_Load();
    
    // Start the USART receiver
    xTaskCreate( vUsartRx_Task, (signed char * ) "USARTRX", mainUSARTRX_TASK_STACK, NULL, mainUSARTRX_PRIORITY, NULL );

    // Start Reactivision (netduinoplus/uip/uIP_Task.c)
    xTaskCreate( vReactivision_Task, ( signed char * ) "REACTV", mainREACTV_TASK_STACK, NULL, mainREACTV_PRIORITY, NULL );

	// Start the CLI receive task
	xTaskCreate( vUSB_cli_receive_task, "USBCLI", mainUSBCLI_TASK_STACK, NULL, mainUSBCLI_PRIORITY, NULL );
	
    // Start the UIP stack task (netduinoplus/uip/uIP_Task.c)
    // IMPORTANT NOTE: Your vuIP_Task must be created after your vReactivision Task.
    xTaskCreate( vuIP_Task, ( signed char * ) "UIP", mainUIP_TASK_STACK, NULL, mainUIP_PRIORITY, NULL );


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

// Task for reactivision output processing
void vReactivision_Task( void)
{
    u16_t ripaddr[2];
    portBASE_TYPE xStatus;

    xTuioPacket xValueRead;
    xTuioPacket xDistanceRead;

    // Tracking Variables
    short sPosX = 0;
    short sPosY = 0;
    short sDegreesX = (servoMIN_DEGREES+servoMAX_DEGREES)/2;
    short sDegreesY = servoMAX_DEGREES;

    // Load up the log file and get this sessions log number
    short sFileNo;
    sFileNo = sSdCard_GetLogNo();
    debug_printf("Fileno %d\r\n", sFileNo);
    vSdCard_CreateLogFile(sFileNo);

    //Make TCP connection
    uip_ipaddr(ripaddr, 192,168,0,1);
    uip_connect(ripaddr, HTONS(3000));

    //Make TCP connection for remote logger
    uip_ipaddr(ripaddr, 192,168,0,1);
    uip_connect(ripaddr, HTONS(3010));

    //Create queue
    xTuioQueue = xQueueCreate(5, sizeof(xTuioPacket));
    xLogQueue = xQueueCreate(20, sizeof(xLogStruct));

    vSemaphoreCreateBinary(xControlServo);

    // Servo control loop
    for (;;) {
        if (xSemaphoreTake(xControlServo, 10) == pdFALSE) {
            continue;
        }

        //Read from TUIO queue.
        xStatus = xQueueReceive(xTuioQueue, &xValueRead, 10);     // Block task for 10ms when waiting for the Queue

        if (xStatus == pdPASS) {    // Process received value

            // values are between 0 and 1
            sPosX = (short) (xValueRead.position_x*100.0f);
            sPosY = (short) (xValueRead.position_y*100.0f);

            // Calculate distance
            if (sFiducialDistance) {
                // Loop until two values are set for the fiducial marker
                while (sFiducialDistance && !(xQueueReceive(xTuioQueue, &xDistanceRead, 10) == pdPASS
                                             && xValueRead.class_id != xDistanceRead.class_id));
                if (sFiducialDistance) {
                    // Compute the distance
                    short sX = sApproxSqrt(sPow(sPosX-(short) (xDistanceRead.position_x*100.0f),2) +
                                                sPow(sPosY-(short) (xDistanceRead.position_y*100.0f),2));

                    sprintf(ucRemoteBuffer, ">> Z-dist: %d\r\n", (1849*sPow(sX,2) - 297690*sX + 14309000)/10000);
                    remote_printf(ucRemoteBuffer);
                }

            // Track servo
            } else {

               // Remember, position is taken from TOP LEFT
               if (sPosX < mainBOUNDING_BOX_MIN && sDegreesX < servoMAX_DEGREES) {
                   sDegreesX++;
               } else if (sPosX > mainBOUNDING_BOX_MAX && sDegreesX > servoMIN_DEGREES) {
                   sDegreesX--;
               }

               if (sPosY < mainBOUNDING_BOX_MIN && sDegreesY > servoMIN_DEGREES) {
                   sDegreesY--;
               } else if (sPosY > mainBOUNDING_BOX_MAX && sDegreesY < servoMAX_DEGREES) {
                   sDegreesY++;
               }

               vSdCard_LogData(sFileNo,
                               (unsigned long) xTaskGetTickCount(),
                               sPosX,
                               sPosY,
                               sDegreesX,
                               sDegreesY);
            }
        }
        // Set the servos and give the semaphore
        vServo_SetPan(sDegreesX);
        vServo_SetTilt(sDegreesY);

        xSemaphoreGive(xControlServo);

        vTaskDelay(10);
    }
}

/* Usart receive task */
void vUsartRx_Task( void ) {
    unsigned char pucFileName[30];          // File name
    unsigned char ucReceived;               // Received char
    short i;

    short sFlag = 0;                        // State flag

    FIL Fil;                                // File object
    FRESULT rc;                             // Result code
    DIR dir;                                // Directory object
    FILINFO fno;                            // File information object
    UINT bw, br;                            // File counters

    // Usart receive loop
    for (;;) {
        // Data is available
        if (USART_IsDataAvailable(AT91C_BASE_US0)) {
            USART_ReadBuffer(AT91C_BASE_US0, &ucReceived, 1);

            // Stage 0: receive file name
            if          (sFlag == 0 && ucReceived == '\x01') {
                i = 0;
                memset(pucFileName, 0, 30);
                sFlag = 1;
                //debug_printf("\r\n--");

            // Stage 1: receiving file name
            } else if   (sFlag == 1 && ucReceived != '\x02') {
                //debug_printf("%c", ucReceived);
                pucFileName[i] = ucReceived;
                i++;
            // Stage 1: end of file name
            } else if   (sFlag == 1 && ucReceived == '\x02') {
                //debug_printf("\r\nRec file: %s\r\n", pucFileName);
                sFlag = 2;

                // Create the file
                rc = f_open(&Fil, pucFileName, FA_CREATE_ALWAYS | FA_WRITE);
                if (rc) {
                    sprintf(ucRemoteBuffer, "File creation failure.\r\n");
                    remote_printf(ucRemoteBuffer);
                    sFlag = 0;
                }

            // Stage 2: receive file
            } else if   (sFlag == 2 && ucReceived != '\x03') {
                //debug_printf("%c", ucReceived);
                f_write(&Fil, &ucReceived, 1, &bw);

            // Stage 2: end of file
            } else if   (sFlag == 2 && ucReceived == '\x03') {
                sFlag = 3;

            // Stage 3: end of transmission
            } else if   (sFlag == 3 && ucReceived == '\x04') {
                f_close(&Fil);
                sprintf(ucRemoteBuffer, "\r\nReceived file: %s\r\n", pucFileName);
                remote_printf(ucRemoteBuffer);
                sFlag = 0;
            }
        } else {
            vTaskDelay(5);
        }
    }
}


/*-----------------------------------------------------------*/
/* USB CLI receive task */
void vUSB_cli_receive_task(void *pvParameters) {

	char cRxedChar;
	char cInputString[20];
	char cInputIndex = 0;
	int8_t *pcOutputString;
	portBASE_TYPE xReturned;

	//Initialise pointer to CLI output buffer.
	memset(cInputString, 0, sizeof(cInputString));
	pcOutputString = FreeRTOS_CLIGetOutputBuffer();

	// Cli receive loop
	for (;;) {

		//Receive character from USB receive
		cRxedChar = ucUSBReadByte();
		if ((cRxedChar != 0) && (cRxedChar != 5)) {

			//reflect byte
			vUSBSendByte(cRxedChar);

			//Process only if return is received.
			if (cRxedChar == '\r') {
				
				//Put null character in command input string.
				cInputString[cInputIndex] = '\0';

                //debug_printf("\r\n---%s---%d---\r\n", pcOutputString, strlen(pcOutputString));
				//memcpy(ucRemoteBuffer, cInputString, strlen(cInputString));
				sprintf(ucRemoteBuffer, "%s", cInputString);
				vRemoteSend(ucRemoteBuffer, strlen(ucRemoteBuffer));

			    do {
                    //Process command input string.
                    xReturned = FreeRTOS_CLIProcessCommand( cInputString, pcOutputString, configCOMMAND_INT_MAX_OUTPUT_SIZE );

                    //Display CLI output string
                    sprintf(ucRemoteBuffer, "%s", pcOutputString);
                    remote_printf(ucRemoteBuffer);

                    // Wait for 10ms
					vTaskDelay(10);
					
				// Loop until repeat is done
			    } while (xReturned == cliREPEAT);
				
				memset(cInputString, 0, sizeof(cInputString));
                cInputIndex = 0;
			
			} else {

				if( cRxedChar == '\r' ) {

					// Ignore the character.
				} else if( cRxedChar == '\b' ) {

					// Backspace was pressed.  Erase the last character in the
					//string - if any.
					if( cInputIndex > 0 ) {
						cInputIndex--;
						cInputString[ cInputIndex ] = '\0';
					}

				} else {
					// A character was entered.  Add it to the string
					// entered so far.  When a \n is entered the complete
					// string will be passed to the command interpreter.
					if( cInputIndex < 20 ) {
						cInputString[ cInputIndex ] = cRxedChar;
						cInputIndex++;
					}
				}
			}	
		}

		vTaskDelay(50);
	}
}

/*-----------------------------------------------------------*/
/* Idle Application Task */
void vApplicationIdleHook( void ) {
    static portTickType xLastTx = 0;

    /* The idle hook simply prints the idle tick count */
    if( ( xTaskGetTickCount() - xLastTx ) > mainUSB_TX_FREQUENCY )
    {
        xLastTx = xTaskGetTickCount();
        LED_Toggle(0);
    }
}
