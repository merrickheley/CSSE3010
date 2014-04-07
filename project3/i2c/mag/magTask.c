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

#include <FreeRTOS.h>
#include <queue.h>
#include <board.h>
#include <debug_printf.h>
#include <math.h>

#include "magTask.h"
#include "magLibrary.h"
#include "../accel/accelLibrary.h"
#include "../accel/accelTask.h";

/* Task for reading data from the accelerometer */
void vMagTask(void *pvParameters) {
    short tempData[3];
    portBASE_TYPE xStatus;

    extern xQueueHandle xMagnetometer_queue;
    xMagnetometer_queue = xQueueCreate(4, sizeof(short)*3);

    short sMagX;
    short sMagY;
    short sMagZ;

    short sAccX;
    short sAccY;
    short sAccZ;

    short sXh;
    short sYh;

    vMagConfigure();

    /* MagTask main loop */
    for (;;) {

        if (sMagEnabled) {
            // Read accelerometer data
            hmc5883l_readData();

            xStatus = xQueueReceive(xMagnetometer_queue, (short *) tempData, 10);

            // If data received from the queue
            if (xStatus == pdPASS) {
                short newData[3] = {0};
                debug_printf("mag   xyz: %d %d %d\r\n", tempData[0], tempData[2], tempData[1]);
                debug_printf("accel xyz: %d %d %d\r\n", -psLastTempData[1], psLastTempData[0], psLastTempData[2]);

                sMagX = tempData[0];
                sMagY = tempData[2];
                sMagZ = tempData[1];

                sAccX = -psLastTempData[1];
                sAccY = psLastTempData[0];
                sAccZ = psLastTempData[2];

                sXh = (short) (sMagX*cos((double) sAccX) + sMagZ*sin((double) sAccX));
                sYh = (short) (sMagX*sin((double) sAccY)*sin((double)sAccX) + sMagY*cos((double) sAccY) + sMagZ*sin((double) sAccY)*cos((double) sAccX));

                debug_printf("north: %d\r\n", (short) (atan2((double) sYh, (double) sXh)*180/3.14));
            }
        }

        vTaskDelay(1000);
    }
}
