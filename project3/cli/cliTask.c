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

#include "cliTask.h"
#include "cliCommands.h"

#include <FreeRTOS.h>
#include <FreeRTOS_CLI.h>
#include <debug_printf.h>

/*-----------------------------------------------------------*/
/* USB CLI receive task */
void vCliTask(void *pvParameters) {

    char cRxedChar;
    char cInputString[20];
    char cInputIndex = 0;
    int8_t *pcOutputString;
    portBASE_TYPE xReturned;

    //Initialise pointer to CLI output buffer.
    memset(cInputString, 0, sizeof(cInputString));
    pcOutputString = FreeRTOS_CLIGetOutputBuffer();

    vConfigureCli();

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


                do {
                    //Process command input string.
                    xReturned = FreeRTOS_CLIProcessCommand( cInputString, pcOutputString, configCOMMAND_INT_MAX_OUTPUT_SIZE );

                    debug_printf("%s", pcOutputString);

                    // Wait for 10ms
                    vTaskDelay(10);

                // Loop until repeat is done
                } while (xReturned == clicommandREPEAT);

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
                        
                        // Here I forgot to actually clear the character.
                        // I don't have the board anymore so I can't check, but
                        // it's probably:
                        // vUSBSendByte(' ');
                        // vUSBSendByte('\b');
                        // To send a space character then backspace over it again.
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

