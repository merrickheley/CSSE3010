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

/* ------------------ Library includes  ------------------ */
#include <debug_printf.h>
#include <string.h>
#include <uip.h>
#include <FreeRTOS.h>
#include <queue.h>
#include <math.h>

/* ------------------ Local includes    ------------------ */
#include "servoTask.h"
#include "servoLibrary.h"
#include "../uip/tuioclient.h"
#include "../radio/radioLibrary.h"

/* ------------------ Defines           ------------------ */
#define servoBOUNDING_BOX_MIN   40
#define servoBOUNDING_BOX_MAX   60

#define servotaskFLIGHT_PLAN_1  1
#define servotaskFLIGHT_PLAN_2  2
#define servotaskFLIGHT_PLAN_3  3
#define servotaskFLIGHT_PLAN_4  4
/* ------------------ Functions         ------------------ */

short sDistanceUsed[6] = {0};
short sX, sY, sZ;

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

// Get an in-use fiducial next to the current fiducial to compare against for distancing
short sGetId(short sCId) {
    if (sCId == 0) {
        if (sDistanceUsed[sCId+1]) return sCId+1;
    }

    if (sCId > 0 && sCId < 5) {
        if (sDistanceUsed[sCId-1]) return sCId-1;
        if (sDistanceUsed[sCId+1]) return sCId+1;
    }

    if (sCId == 5) {
        if (sDistanceUsed[sCId-1]) return sCId-1;
    }

    return -1;
}

// Task for reactivision output processing
void vServoTask( void)
{
    u16_t ripaddr[2];
    portBASE_TYPE xStatus;

    xTuioQueuePacket xValueRead;

    xTuioQueuePacket xDistanceValue[6];

    // Tracking Variables
    short sPosX = 0;
    short sPosY = 0;
    short sDegreesX = servoMIN_DEGREES;
    short sDegreesY = (servoMIN_DEGREES+servoMAX_DEGREES)/4;

    vServo_ConfigurePwm(sDegreesX, sDegreesY);

    sX = 0;
    sY = 0;
    sZ = 0;

    // Uip connect
    vTaskDelay(1000);
    uip_ipaddr(ripaddr, 192,168,0,1);
    uip_connect(ripaddr, HTONS(3000));

    xTuioQueue = xQueueCreate(20, sizeof(xTuioQueuePacket));

    vSemaphoreCreateBinary(xControlServo);

    short sFlightPlanStage = servotaskFLIGHT_PLAN_1;
    short sGoalPoint;
    short sGoalCounter;

    // Servo control loop
    for (;;) {

        //Read from TUIO queue.
        xStatus = xQueueReceive(xTuioQueue, &xValueRead, 10);     // Block task for 10ms when waiting for the Queue

        if (xStatus == pdPASS) {   // Process received value

            // values are between 0 and 1
            sPosX = (short) (xValueRead.position_x*100.0f);
            sPosY = (short) (xValueRead.position_y*100.0f);

            short sId = xValueRead.class_id-servoFIDUCIAL_SET;

            // If the middle fiducial marker, track it with the camera
            if (sId >= 0 && sId <= 5) {
                // Remember, position is taken from TOP LEFT
                if (sPosX < servoBOUNDING_BOX_MIN && sDegreesX < servoMAX_DEGREES) {
                    sDegreesX++;
                } else if (sPosX > servoBOUNDING_BOX_MAX && sDegreesX > servoMIN_DEGREES) {
                    sDegreesX--;
                }

                if (sPosY < servoBOUNDING_BOX_MIN && sDegreesY < servoMAX_DEGREES) {
                    sDegreesY++;
                } else if (sPosY > servoBOUNDING_BOX_MAX && sDegreesY > servoMIN_DEGREES) {
                    sDegreesY--;
                }

                // Set the fiducial to being used, and the value to the current packet
                sDistanceUsed[sId] = 1;
                xDistanceValue[sId] = xValueRead;

                // If there is an ID to compare to, calculate distance
                if (sGetId(sId) != -1 && sTask5) {
                    short sNextId = sGetId(sId);

                    // Print markers used for distancing
                    //debug_printf("markers: %d %d\r\n", xValueRead.class_id, xDistanceValue[sNextId].class_id);

                    // Compute the distance to the fiducial
                    double dD = sApproxSqrt(sPow(sPosX-(short) (xDistanceValue[sNextId].position_x*100.0f),2) +
                                           sPow(sPosY-(short) (xDistanceValue[sNextId].position_y*100.0f),2));

                    dD = (33379*sPow((short) dD,2) - 2288800*dD + 44475000)/10000;

                    //debug_printf(">> Distance: %d\r\n", (short) dD);

                    // Calculate the XYZ coordinates.
                    double dDegX = sDegreesX - servoMIN_DEGREES - (servoMIN_DEGREES+servoMAX_DEGREES)/2;
                    double dDegY = sDegreesY - servoMIN_DEGREES;

                    sX = (short) (dD*(sin((double) (dDegX/180.0f*3.14f))));
                    sY = (short) (dD*(sin((double) (dDegY/180.0f*3.14f))));
                    sZ = (short) (dD*(cos((double) (dDegX/180.0f*3.14f))));

                    //debug_printf(">> Angles: %d %d\r\n", (short) dDegX, (short) dDegY);
                    //debug_printf(">> Point: %d %d %d\r\n", sX, sY, sZ);

                    // On detecting the blimp, set the goal to 1.5m in X and flight plan to 2
                    if (sId < 3 && sFlightPlanStage == servotaskFLIGHT_PLAN_1) {
                        sGoalPoint = sX + 1500;
                        sGoalCounter = 0;
                        sFlightPlanStage = servotaskFLIGHT_PLAN_2;
                        debug_printf("Starting stage 2\t\t\ Goal: %d\r\n", sGoalPoint);
                    // If in stage 2, check if it has reached it's goal point. if this is confirmed
                    // using a counter, move to stage 3
                    } else if (sFlightPlanStage == servotaskFLIGHT_PLAN_2) {
                        sSetSpeed(0xFF);

                        if (sX > sGoalPoint) {
                            sGoalCounter++;
                        }

                        if (sGoalCounter > 10) {
                            sFlightPlanStage = servotaskFLIGHT_PLAN_3;
                            debug_printf("Starting stage 3\r\n");
                        }

                    // Set the goal point 1.5m back in x and move to stage 4
                    } else if (sFlightPlanStage == servotaskFLIGHT_PLAN_3) {
                        sRequestTurnLeft();

                        if (sId > 2) {
                            sGoalPoint = sX - 1500;
                            sGoalCounter = 0;
                            sFlightPlanStage = servotaskFLIGHT_PLAN_4;
                            debug_printf("Starting stage 4\t\t\ Goal: %d\r\n", sGoalPoint);
                        }
                    // If in stage 4, check if it has reached it's goal point and end
                    } else if (sFlightPlanStage == servotaskFLIGHT_PLAN_4) {
                        sSetSpeed(0xFF);

                        if (sX < sGoalPoint) {
                            sGoalCounter++;
                        }

                        if (sGoalCounter > 10) {
                            sFlightPlanStage = servotaskFLIGHT_PLAN_1;
                            debug_printf("Ending stage 4\r\n");
                        }
                    }
                }
            }


        }
        //debug_printf("cos: %d %d\r\n", (int) cos(3.1415), (int) sin(3.1415));
        // Set the servos and give the semaphore
        vServo_SetPan(sDegreesX);
        vServo_SetTilt(sDegreesY);

        vTaskDelay(10);
    }
}

// Clear the used values every 5 seconds.
void vServoClearTask() {
    for (;;) {
        if (sTask5) {
            debug_printf(">> Point: %d %d %d\r\n", sX, sY, sZ);
            memset(sDistanceUsed, 0, sizeof(short)*6);
        }
        vTaskDelay(5000);
    }
}
