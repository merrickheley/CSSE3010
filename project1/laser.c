#include "laser.h"

// Turn an unencoded byte into two hamming-encoded bytes
unsigned char* encodeHamming(unsigned char dataByte) {
    unsigned char d[4], h[3], p;
    unsigned char halfByte;
    int half;
    static unsigned char encodedByte[2];
    
    // Loop across both halves of the byte
    for (half = 0; half < 2; half++) {
        halfByte = half ? (dataByte & 0xF0) >> 4: dataByte & 0x0F;
        
        d[0] = (halfByte & 0x01) ? 1 : 0;
        d[1] = (halfByte & 0x02) ? 1 : 0;   // Can also do !!(halfByte & 0x02)
        d[2] = (halfByte & 0x04) ? 1 : 0;   // Which is a double-not
        d[3] = (halfByte & 0x08) ? 1 : 0;

        // Create the hamming bits
        h[2] = (d[3] + d[1] + d[0]) % 2;
        h[1] = (d[3] + d[2] + d[0]) % 2;
        h[0] = (d[3] + d[2] + d[1]) % 2;

        // Create the overall parity bit
        p = (d[0] + d[1] + d[2] + d[3] + h[0] + h[1] + h[2]) % 2;

        // Create the encoded byte
        encodedByte[half] = p << 7 |h[2] << 6 | h[1] << 5 | h[0] << 4 | halfByte;    
    }
    
    return encodedByte;
}

// Decode Hamming function.
// Returns the decoded halfbyte and the bit+1 the error is on. 
// For multiple errors return 9. 
unsigned char* decodeHamming(unsigned char dataByte) {
    unsigned char d[4], h[3], p;
    unsigned char s[3];
    int sNum;
    int errBit = 0;
    static unsigned char ret[2];
    ret[0] = 0;
    ret[1] = 0;
    
    // p h2 h1 h0 d3 d2 d1 d0
    //d3 d2 d1 d0 h0 h1 h2 p
 
    // Extract the bits
    p    = (dataByte & 0x80) ? 1 : 0;
    h[2] = (dataByte & 0x40) ? 1 : 0;
    h[1] = (dataByte & 0x20) ? 1 : 0;
    h[0] = (dataByte & 0x10) ? 1 : 0;

    d[3] = (dataByte & 0x08) ? 1 : 0;
    d[2] = (dataByte & 0x04) ? 1 : 0;
    d[1] = (dataByte & 0x02) ? 1 : 0;
    d[0] = (dataByte & 0x01) ? 1 : 0;
    
    // Create the syndromes
    s[0] = (d[1] + d[2] + d[3] + h[0]) % 2;
    s[1] = (d[0] + d[2] + d[3] + h[1]) % 2;
    s[2] = (d[0] + d[1] + d[3] + h[2]) % 2;
    
    // Find the bit with an error
    sNum = s[0]*4 + s[1]*2 + s[2];
    switch (sNum) {
        case 0x00: errBit = 0; break;
        case 0x01: errBit = 7; h[2] = !h[2]; break;
        case 0x02: errBit = 6; h[1] = !h[1]; break;
        case 0x03: errBit = 1; d[0] = !d[0]; break;
        case 0x04: errBit = 5; h[0] = !h[0]; break;
        case 0x05: errBit = 2; d[1] = !d[1]; break;
        case 0x06: errBit = 3; d[2] = !d[2]; break;
        case 0x07: errBit = 4; d[3] = !d[3]; break;
    }
    
    // Check parity
    if ((d[3] + d[2] + d[1] + d[0] + h[0] + h[1] + h[2]) % 2 != p) {
        if (errBit == 0) {
            errBit = 8;
            p = !p;
        } else {
            errBit = 9;
        }
    }
    
    ret[0] = (d[3] << 3) | (d[2] << 2) | (d[1] << 1) | d[0];
    ret[1] = errBit;
    
    // return the data and the bit with an error on it (if any)
    return ret;
}

// Get an array of manchester bits for a character
unsigned char *encodeManchester(unsigned char transmitChar) {
    unsigned char padHamBuffer[22] = {0}; // Hamming buffer padded with start and stop bits, e.g. 11DATA0
    static unsigned char manchesterBuffer[44] = {0}; // Manchester encoded

    unsigned char* hammingret;

    int i, j;
            
    // Get the hamming encoding
    hammingret = encodeHamming(transmitChar);

    // Create a padded array of bits from the hamming bytes
    for (i = 0; i < 2; ++i) {
        // Add padding bits (remember: 11 bits per byte of data)
        padHamBuffer[i*11 + 0] = 1;             // Initial 1
        padHamBuffer[i*11 + 1] = 1;             // Second 1
        
        // Copy the bits into the array (remember 2 padded bits)
        for (j = 0; j < 8; ++j) {
            padHamBuffer[i*11+j+2] = !!(hammingret[i] & (1 << j));
        }
        
        // Previous bits + the 10 just added
        padHamBuffer[i*11+10] = 0;             // Trailing 0
    }
    
    // Encode in manchester format. 1->01, 0->10
    for (i = 0; i < 22; ++i) {
        manchesterBuffer[i*2] = !padHamBuffer[i];
        manchesterBuffer[i*2+1] = padHamBuffer[i];
    }

    return manchesterBuffer;
}

// Decode an char array of binary numbers into a char
unsigned char *decodeManchester(unsigned char *rxBuffer) {
    unsigned char demodulatedBuffer[22] = {0};
    unsigned char hammingByte1 = 0;
    unsigned char hammingByte2 = 0;
    unsigned char *hammingret1;
    unsigned char *hammingret2;

    static unsigned char retarray[5] = {0};
    
    int i = 0;

    for (i=0; i < 5; i++) {
        retarray[i] = 0;
    }
    
    // Iterate across the manchester buffer and create a demodulated buffer
    for (i = 0; i < 22; ++i) {
        // If 01, 1. Elseif 10, 0.
        if (rxBuffer[i*2+0] == 0 && rxBuffer[i*2+1] == 1){
            demodulatedBuffer[i] = 1;
        } else if (rxBuffer[i*2+0] == 1 && rxBuffer[i*2+1] == 0) {
            demodulatedBuffer[i] = 0;
        }
    }
    
    // Get the two hamming bytes from the demodulated buffer
    for (i = 0; i < 8; ++i) {
         hammingByte1 |= demodulatedBuffer[i+2] << i;
         hammingByte2 |= demodulatedBuffer[i+13] << i;
    }
    
    // Decode the hamming byte and note how many errors, and how many were fixed
    hammingret1 = decodeHamming(hammingByte1);
    retarray[0] = hammingret1[0];
    if (hammingret1[1] > 0 && hammingret1[1] < 9) {
        retarray[1] = 1;
        retarray[3] = 1;
    } else if (hammingret1[1] == 9) {
        retarray[1] = 2;
        retarray[3] = 0;
    }

    // Decode the second hamming byte and note how many errors, and how many were fixed
    hammingret2 = decodeHamming(hammingByte2);
    retarray[0] |= hammingret2[0]  << 4;
    if (hammingret2[1] > 0 && hammingret2[1] < 9) {
        retarray[2] = 1;
        retarray[4] = 1;
    } else if (hammingret1[1] == 9) {
        retarray[2] = 2;
        retarray[4] = 0;
    }

    return retarray;
}

