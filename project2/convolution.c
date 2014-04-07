 /* -----------------------------------------------------------------------------
# Company name: CSSE3010 - Embedded Systems Design & Interfacing
# Project name: Project 2
# Module name : convolution
# Functionality: gives the convolution encoder commands
# Hardware platform: Netduino (AT91SAM7)
#
# Author name: Merrick Heley
# Creation date: 2013-04-28
------------------------------------------------------------------------------*/

#include "convolution.h"

#include <stdio.h>
#include <string.h>
#include "debug_printf.h"

// convolution node definitions
#define convNODE_A 1
#define convNODE_B 2
#define convNODE_C 3
#define convNODE_D 4

// Structure for a path
typedef struct {
    short sNode;
    short psBits[8];
    short sError;
} xPath;

// Convolution encode a char to a string of 6 hex chars
// Allows initial state to be set in psState
void vEncoder(unsigned char ucInputByte, unsigned char *pucOutput, short *psState) {
    short i,j;
    unsigned char pucOutArr[24] = {0};         // Output array of short (binary) numbers

    // Iterate accross all bits in the input byte
    for (i=0; i<8; i++) {
        // Determine the state
        psState[0] = psState[1];
        psState[1] = psState[2];
        psState[2] = !!(ucInputByte & (1 << i));

        // Add the output to the output array
        pucOutArr[i*3 + 0] = psState[2] ^ psState[1];
        pucOutArr[i*3 + 1] = psState[1] ^ psState[0];
        pucOutArr[i*3 + 2] = psState[2] ^ (psState[1] ^ psState[0]);
    }

    // Iterate over the 3 output characters
    for (i=0; i<3; i++) {
        pucOutput[i] = 0;
        // Set the bits in the output char
        // Start at the end of the output array, and add it
        // to char (starting at MSB)
        for (j=0; j<8; j++) {
            pucOutput[i] |= pucOutArr[23-i*8-j] << (7-j);
        }
    }
}

// Convert a string of hex chars to a number
short sHexStringToNumConv(char *pucInput, short sLen, long *lOutput) {
    *lOutput = 0;
    short sCountLen = 0;

    // Iterate over the chars
    // Multiply the current number by 16 and add the new number
    while (sCountLen < sLen) {
        if ('0' <= *pucInput && *pucInput <= '9') {
            *lOutput = *lOutput*16 + (*pucInput-'0');
        } else if ('a' <= *pucInput && *pucInput <= 'f') {
            *lOutput = *lOutput*16 + (*pucInput-'a' + 10);
        } else if ('A' <= *pucInput && *pucInput <= 'F') {
            *lOutput = *lOutput*16 + (*pucInput-'A' + 10);
        } else {
            return 1;
        }

        pucInput++;
        sCountLen++;
    }

    return 0;
}

// Transition to the next node
short sDecoderTransition(short sNode, short sBit) {
    switch(sNode*10+sBit) {
        case (convNODE_A*10+0): return convNODE_A;
        case (convNODE_A*10+1): return convNODE_C;
        case (convNODE_B*10+0): return convNODE_A;
        case (convNODE_B*10+1): return convNODE_C;
        case (convNODE_C*10+0): return convNODE_B;
        case (convNODE_C*10+1): return convNODE_D;
        case (convNODE_D*10+0): return convNODE_B;
        case (convNODE_D*10+1): return convNODE_D;
    }
    return 0;
}

// Convert a 3 bit number to a short binary array
void vNumToBits(short sNum, short *psBits){
    short i;
    // Iterate over each bit, move it to an array
    for (i=0; i<3; i++) {
        psBits[i] = !!(sNum & (1 << i));
    }
}

// Calculates the error between the input bits and a node transition
short sDecoderError(short *psInputBits, short sNode, short sBit) {
    short psTransBits[3];           // Transition bits
    short sError = 0;               // Error in this transition
    short i;

    // Bit order must be swapped for LSB notation. E.g. 011 -> 110
    switch(sNode*10+sBit) {
        case (convNODE_A*10+0): vNumToBits(0b000, psTransBits); break;
        case (convNODE_A*10+1): vNumToBits(0b101, psTransBits); break;
        case (convNODE_B*10+0): vNumToBits(0b110, psTransBits); break;
        case (convNODE_B*10+1): vNumToBits(0b011, psTransBits); break;
        case (convNODE_C*10+0): vNumToBits(0b111, psTransBits); break;
        case (convNODE_C*10+1): vNumToBits(0b010, psTransBits); break;
        case (convNODE_D*10+0): vNumToBits(0b001, psTransBits); break;
        case (convNODE_D*10+1): vNumToBits(0b100, psTransBits); break;
    }

    // Reconstruct the bit, add error if bits are different
    for (i=0; i<3;i++) {
        sError += psInputBits[i]^psTransBits[i];
    }

    return sError;
}

// Check which error is larger, and copy the smallest to the destination
xPath *pxDecoderCompare(xPath *pxNode1, xPath *pxNode2) {
    if (pxNode1->sError <= pxNode2->sError || (pxNode1->sNode && !pxNode2->sNode)) {
        return pxNode1;
    } else {
        return pxNode2;
    }
}

// Decode a convolution encoded string of 6 hex chars
// using the viterbi algorithm
unsigned char ucDecoder(unsigned char *pucEncoded) {
    short i,j,k;
    short sBits[24] = {0};
    long lTempNum;

    static xPath xSurvivors[4] = {{0, {0}, 0}};        // Survivors from pruning
    static xPath xNextStage[8] = {{0, {0}, 0}};        // Paths generated from survivors
    xPath xMinError;

    unsigned char ucReturn = 0;

    j=0;
    // Convert the input into an array of short binary numbers
    for (i=0; i<=4; i+=2) {
        lTempNum=0;

        sHexStringToNumConv((char *) (pucEncoded+i+1), 1, &lTempNum);

        // Extract the bits from the number
        sBits[j++] = !!(((unsigned char) lTempNum) & 0x01);
        sBits[j++] = !!(((unsigned char) lTempNum) & 0x02);
        sBits[j++] = !!(((unsigned char) lTempNum) & 0x04);
        sBits[j++] = !!(((unsigned char) lTempNum) & 0x08);

        sHexStringToNumConv((char *) (pucEncoded+i), 1, &lTempNum);

        // Extract the bits from the number
        sBits[j++] = !!(((unsigned char) lTempNum) & 0x01);
        sBits[j++] = !!(((unsigned char) lTempNum) & 0x02);
        sBits[j++] = !!(((unsigned char) lTempNum) & 0x04);
        sBits[j++] = !!(((unsigned char) lTempNum) & 0x08);
    }

    xSurvivors[0].sNode = convNODE_A;
    // Iterate over the transitions
    for (i=0; i < 8; i++) {
        // Iterate over each node
        for (j=0; j < 4; j++){
            if (xSurvivors[j].sNode == 0) {
                continue;
            }

            // Iterate over each transition from the node, and compute the next node
            for (k=0; k < 2; k++) {
                memcpy(&xNextStage[2*j+k], &xSurvivors[j], sizeof(xPath));
                xNextStage[2*j+k].sNode = sDecoderTransition(xSurvivors[j].sNode, k);
                xNextStage[2*j+k].psBits[i] = k;
                xNextStage[2*j+k].sError += sDecoderError(sBits+i*3, xSurvivors[j].sNode, k);
            }
        }

        // Copy the result of the comparison to survivors
        memcpy(&xSurvivors[0], pxDecoderCompare(&xNextStage[0], &xNextStage[2]), sizeof(xPath));
        memcpy(&xSurvivors[1], pxDecoderCompare(&xNextStage[4], &xNextStage[6]), sizeof(xPath));
        memcpy(&xSurvivors[2], pxDecoderCompare(&xNextStage[1], &xNextStage[3]), sizeof(xPath));
        memcpy(&xSurvivors[3], pxDecoderCompare(&xNextStage[5], &xNextStage[7]), sizeof(xPath));

    }

    // Find the node with minimum error
    xMinError = xSurvivors[0];
    for (k=1; k<4; k++) {
        if (xSurvivors[k].sError < xMinError.sError ) {
            xMinError = xSurvivors[k];
        }
    }
    // Reconstruct the decoded char
    for (k=0; k<8; k++) {
        ucReturn |= (xMinError.psBits[k] << k);
    }

    return ucReturn;
}

