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

/* ------------------ Library includes  ------------------ */
#include <debug_printf.h>
#include <string.h>

/* ------------------ Local includes    ------------------ */
#include "radioTask.h"
#include "radioLibrary.h"
#include "nrf24l01plus.h"
#include "../cli/cliCallbacks.h"

/* ------------------ Defines           ------------------ */
#define TX_PLOAD_WIDTH  32  // 32 unsigned chars TX payload

/* ------------------ Functions         ------------------ */

void vRadioTask() {

    unsigned char tx_buf[TX_PLOAD_WIDTH]; // TX Buffer
    unsigned char rx_buf[TX_PLOAD_WIDTH]; // RX Buffer
    short i, logNum;

    // Create Queues.
    xRadioRX_Queue = xQueueCreate(5, sizeof(char)*32);
    xRadioTX_Queue = xQueueCreate(5, sizeof(char)*32);

    ucDebugPrinting = 0;

    // Configure the radio task
    vRadioConfigure();

    for (;;) {
        nrf24l01plus_receive_packet();

        if (xQueueReceive(xRadioTX_Queue, tx_buf, 1) == pdPASS) {    //Process received value
            if (ucDebugPrinting == 2) {
                debug_printf("Send: ");
                for(i = 0; i < 32; i++ ) {
                    debug_printf("%02x ", tx_buf[i]);
                }
                debug_printf("\r\n");
            }
            nrf24l01plus_mode_tx_send(tx_buf);
            while(!nrf24l01plus_Send_Packet(tx_buf));
            nrf24l01plus_mode_rx();
        }
        if (xQueueReceive(xRadioRX_Queue, rx_buf, 1) == pdPASS) {    //Process received value
            if (ucDebugPrinting == 1) {
                debug_printf("Received: ");
                for(i = 0; i < 32; i++ ) {
                    debug_printf("%02x ", rx_buf[i]);
                }
                debug_printf("\r\n");
            }
            vProcessPacket(rx_buf);
        }
        vTaskDelay(10 / portTICK_RATE_MS);
    }
}

void vRadioPrintTask() {

    // Clear the print packet
    memset(&xPrintPacket, 0, sizeof(xBlimpPacket));

    // Turn printing off
    ucPeriodicPrinting = 0;

    /* loop over printing */
    for (;;) {

        if (ucPeriodicPrinting) {
            debug_printf("%d, %d, %d, %d, %d, %d \r\n", xPrintPacket.ucBlimpId,
                                                        xPrintPacket.usIRValues,
                                                        xPrintPacket.usMagnometer,
                                                        xPrintPacket.ucAckForwardMotors,
                                                        xPrintPacket.ucAckDirectionCommand,
                                                        xPrintPacket.ucAckAltitude);
        }

        vTaskDelay(1000 / portTICK_RATE_MS);
    }
}
