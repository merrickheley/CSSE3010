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

#include <FreeRTOS.h>
#include <queue.h>
#include <board.h>
#include <debug_printf.h>

#include "accelTask.h"
#include "accelLibrary.h"
#include "../../radio/radioLibrary.h"

/* Task for reading data from the accelerometer */
void vAccelTask(void *pvParameters) {
    short tempData[3];
    portBASE_TYPE xStatus;

    extern xQueueHandle xAccelerometer_queue;
    xAccelerometer_queue = xQueueCreate(4, sizeof(short)*3);
    unsigned int i;

    vAccelConfigure();

    /* AccelTask main loop */
    for (;;) {

        // If motors are on, set the blimp
        if (sMotorsEnabled()) {
            // Read accelerometer data
            mma8452_readData();

            xStatus = xQueueReceive(xAccelerometer_queue, (short *) tempData, 10);

            // If data received from the queue
            if (xStatus == pdPASS) {
                psLastTempData = tempData;

                // Extract and format data
                sSetSpeed((tempData[1]/100)*5 + 0x7F);
                sSetDirection((-tempData[0]/100)*5 + 0x7F);

                //debug_printf("Accelerometer X: %02X\tY: %02X\r\n", ucForwardMotors, ucDirection);
                sCommandBlimp();
            }
        }

        vTaskDelay(300);
    }
}
