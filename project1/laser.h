/* -----------------------------------------------------------------------------
# Header for laser functions
------------------------------------------------------------------------------*/

// Turn an unencoded byte into two hamming-encoded bytes
unsigned char* encodeHamming(unsigned char dataByte);

// Decode Hamming function.
// Returns the decoded halfbyte and the bit+1 the error is on. 
// For multiple errors return 9. 
unsigned char* decodeHamming(unsigned char dataByte);


// Get an array of manchester bits for a character
unsigned char *encodeManchester(unsigned char transmitChar);

// Decode the array of manchester bits for a character
unsigned char *decodeManchester(unsigned char *rxBuffer);
