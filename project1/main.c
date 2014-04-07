 /* -----------------------------------------------------------------------------
# Company name: CSSE3010 - Embedded Systems Design & Interfacing
# Project name: Project 1
# Module name : Main 
# Hardware platform: Netduino (AT91SAM7)

------------------------------------------------------------------------------*/

//------------------------------------------------------------------------------
//         Headers
//------------------------------------------------------------------------------
#include <board.h>
#include <adc/adc.h>
#include <pio/pio.h>
#include <pio/pio_it.h>
#include <pit/pit.h>
#include <aic/aic.h>
#include <dbgu/dbgu.h>
#include <pwmc/pwmc.h>
#include <twi/twi.h>
#include <drivers/twi/twid.h>
#include <utility/math.h>
#include <utility/trace.h>
#include <usb/device/cdc-serial/CDCDSerialDriver.h>
#include <usb/device/cdc-serial/CDCDSerialDriverDescriptors.h>
#include <stdio.h>
#include <string.h>
#include <tc/tc.h>

#include <i2c.h>
#include "nrf24l01plus.h"

#include "laser.h"

//------------------------------------------------------------------------------
//         Definitions and variables
//------------------------------------------------------------------------------

// PWM
#define CHANNEL_SERVO1              2
#define CHANNEL_SERVO2              3
#define CHANNEL_LED                 1

#define SERVO_PWM_FREQUENCY         50
#define SERVO_DUTY_CYCLE            2000
#define MAX_DEGREES                 225  // Servo limits
#define MIN_DEGREES                 55   // Changed from 255 and 50

// ADC
#define BOARD_ADC_FREQ              5000000
#define ADC_VREF                    3300  // 3.3 volts

// Size in bytes of the buffer used for reading data from the USB & USART
#define DATABUFFERSIZE \
    BOARD_USB_ENDPOINTS_MAXPACKETSIZE(CDCDSerialDriverDescriptors_DATAIN)

// Indicates that the conversion is finished for the ADC.
static volatile unsigned char conversionDone;

// Pio pins to configure.
static const Pin pins[] = {
    PINS_DBGU,
    PIN_PWMC_PWM2,
    PIN_PWMC_PWM3,
    PIN_LASER,
    PINS_ADC,
    PINS_TWI,
};

//TWI and Accelerometer Parameters - DO NOT MODIFY
Twid twid;
// Set the scale below either 2, 4 or 8
const unsigned char SCALE = 2;  // Sets full-scale range to +/-2, 4, or 8g. Used to calc real g values. 
// Set the output data rate below. Value should be between 0 and 7
const unsigned char dataRate = 0;  // 0=800Hz, 1=400, 2=200, 3=100, 4=50, 5=12.5, 6=6.25, 7=1.56

/// Pin array used for the nRF24l01+.
static const Pin pinsRF[] = {BOARD_RF_CE, PIN_SPI0_NPCS0, BOARD_RF_SPI_PINS};

//IO button
#define PIN_IOBUTTON_1    {1 << 3, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_INPUT, PIO_DEFAULT}
#define PIN_LASER_INPUT {1 << 0, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT}

#define IO_PB 0
#define IO_LASER 1

// Pushbutton pin instance.
static const Pin pinIO[] = {PIN_PUSHBUTTON_1, PIN_LASER_INPUT}; //PIN_IOBUTTON_1;

// Pio pins to enable/disable mux on the netduino.
static const Pin pinsMux[] = {PIN_MUX1, PIN_MUX2};

// Buffer for storing incoming USB data.
static unsigned char usbBuffer[DATABUFFERSIZE];
static char usbSendBuffer[DATABUFFERSIZE];

//------------------------------------------------------------------------------
//         Task Variables
//------------------------------------------------------------------------------

#define DEV_ACCEL 0x1D

#define I2C_READ 1
#define I2C_WRITE 0
#define I2C_AK 1
#define I2C_NAK 0

#define READ_DEV(reg) (((reg << 1 | I2C_READ)^0xFF)^0xFF)
#define WRITE_DEV(reg) (((reg << 1 | I2C_WRITE)^0xFF)^0xFF)

#define FINE_DEGREES 20
#define FINE_MAX_DEGREES FINE_DEGREES*MAX_DEGREES
#define FINE_MIN_DEGREES FINE_DEGREES*MIN_DEGREES

#define TRUE 1
#define FALSE 0

#define ACC 0       // Input types
#define JOY 1
#define KEY 2

int inputType = ACC;  // ACC, JOY, KEY
unsigned char inputKey; // The key taken from the buffer when inputType is KEY

#define TX_RADIO 0
#define TX_LASER 1

int txMode = TX_RADIO;  // Default transmit mode, radio or laser

#define MAX_LEN_MSG 20                      // Maximum radio message lenght
int lenMsg = 0;                             // length of message
int transmitFlag = FALSE;                   // Ready to transmit?
unsigned char keyBuf[MAX_LEN_MSG + 1];      // Keyboard buffer
unsigned char payload[MAX_LEN_MSG + 1];     // Payload buffer

// Variables for radio transmission
char packetType = 0xA1;         
char srcId[] = {0x42, 0x33, 0x99, 0x15};
char dstId[] = {0x42, 0x33, 0x43, 0x67};
char baseId[] = {0x12, 0x34, 0x56, 0x78};

unsigned int txLzPeriod = 1000;     // Period to transmit at
unsigned char txLzBuffer[44];       // ((8 bits + 3 padding) * 2 hamming ) * 2 manchester
int txLzCounter = 0;                // Count in transmit
int txLzFlag = FALSE;               // Flag for stage of transmit

#define RX_READY 1
#define RX_DISCOVER 2
#define RX_RECEIVE 3
#define RX_PROCESS 4
unsigned int rxLzPeriodDiscover = 50;       // Period to check rx at to determine period
static unsigned char rxLzBuffer[44] = {0};  // Recieve buffer for laser
int rxLzCounter = 0;                        // Count recieved bytes
int rxLzFlag = FALSE;                       // Mode for recieve (ready, discover, recieving)
unsigned int rxLzPulseCounter = 0;          // Measure the time for each pulse
unsigned int rxLzPeriod = 0;                // Discovered period for current signal

int dpFlag = FALSE;                         // Flag for duplex communication
unsigned char lastChar = 0;                 // Last character sent
unsigned char errStr[] = "ERROR";           // String for error

#define SR_START 1
#define SR_STARTWAIT 2
#define SR_SCANROW 3
#define SR_SHIFTROW 4
#define SR_FOUNDROW 5
#define SR_FORWARD 1
#define SR_BACKWARD -1
int searchFlag = FALSE;         // Flag for searching
int hideFlag = FALSE;           // Flag for replying to search
int searchRadioFlag = FALSE;    // Recieved search feedback on radio
int searchDir;                  // Direction to search in
int searchRow;                  // Row for found signal
int searchX;                    // Current X value to be searched
int searchY;                    // Current Y value to be searched
int searchTimestamp;            // Used for creating delays in searching

void ConfigureTc(void);

int laserOn;                    // Status of laser

// Check that the servo is within its range
int servo_degrees(int deg, int min, int max) {

    if (deg > max) {
        deg = max;
    }
    
    if (deg < min) {
        deg = min;
    }

    return deg;
}

// USB callback for CDCD serial
void USBDCallbacks_Resumed(void)
{
}

// USB callback for CDC serial
void USBDCallbacks_Suspended(void)
{
}

// Busy wait loop
void UTIL_Loop(volatile unsigned int loop)
{
    // Decrement loop variable
    while(loop--);    
}

// Busy wait in uS
void UTIL_WaitTimeInUs(unsigned int mck, unsigned int time_us)
{
    volatile unsigned int i = 0;
    i = (mck / 1000000) * time_us;
    i = i / 3;
    UTIL_Loop(i);
}

// Send a character through the laser
void laserTx(unsigned char transmitChar) {
    lastChar = transmitChar;        // Save the last char in case retransmission is needed
    
    UTIL_WaitTimeInUs(BOARD_MCK, 2000);
    sprintf(usbSendBuffer, "SEND VIA LASER: %c\r\n", transmitChar);
    CDCDSerialDriver_Write(usbSendBuffer, strlen(usbSendBuffer), 0, 0);

    memcpy(txLzBuffer, encodeManchester(transmitChar), 44);
    
    // Print the manchester modulated bits
    //int i;
    //for (i=0; i < 44; ++i) {       
    //    sprintf(usbSendBuffer+i, "%d", txLzBuffer[i]);
    //}
    //sprintf(usbSendBuffer+i, "\r\n");
    //CDCDSerialDriver_Write(usbSendBuffer, strlen(usbSendBuffer), 0, 0);
    
    // Start transmitting
    txLzFlag = TRUE;
    TC_Start(AT91C_BASE_TC0);
}

// Send a string using radio
// if disp, print the string to serial
void radioTx(unsigned char *txString, int len, int disp) {
    int i;
    unsigned char unencodedMsg[32];
 
    //Set packet type
    unencodedMsg[0] = packetType;
    
    //Set destination
    for (i = 0; i < 4; ++i) { unencodedMsg[i+1] = dstId[i]; }

    //Set Source
    for (i = 0; i < 4; ++i) { unencodedMsg[i+5] = srcId[i]; }            
    
    //Set payload
    for (i = 0; i < len; ++i) { unencodedMsg[i+9] = txString[i]; }

    //Fill with empty chars
    for (i = 9+len; i < 32; ++i) { unencodedMsg[i] = '\0'; }
    
    memcpy(tx_buf, unencodedMsg, 32);
    
    // Sent the packet and wait until done
    nrf24l01plus_mode_tx_send();
    while(!nrf24l01plus_Send_Packet());
    // Set the radio back to recieve mode
    nrf24l01plus_mode_rx();
    
    if (disp) {
        UTIL_WaitTimeInUs(BOARD_MCK, 2000);
        sprintf(usbSendBuffer, "SEND VIA RADIO: %02X %02X%02X%02X%02X %02X%02X%02X%02X %s\r\n", unencodedMsg[0], unencodedMsg[1],unencodedMsg[2],unencodedMsg[3],unencodedMsg[4],unencodedMsg[5],unencodedMsg[6],unencodedMsg[7],unencodedMsg[8], &unencodedMsg[9]);
        CDCDSerialDriver_Write(usbSendBuffer, strlen(usbSendBuffer), 0, 0);
    }
}


//------------------------------------------------------------------------------
//         Local functions
//------------------------------------------------------------------------------


// Converts raw output from the ADC (10-bit) to a millivolt equivalent.
static unsigned int ConvADC2mV( unsigned int valueToConvert )
{
    return((ADC_VREF * valueToConvert)/1023);
}

// Callback invoked when data has been received on the USB.
static void UsbDataReceived(unsigned int unused,
                            unsigned char status,
                            unsigned int received,
                            unsigned int remaining)
{

    // Check that data has been received successfully
    if (status == USBD_STATUS_SUCCESS) {
        unsigned char newChar = usbBuffer[0];        
        
        // WASD control
        if (inputType==KEY) {
            inputKey = newChar;
        // Check for special keyboard commands documented in workbook
        } else {
            // Stop laser transmit mode
            if (newChar == '$') {
                txMode = TX_LASER;
                PIO_Clear(&pins[3]);
                PIO_DisableIt(&pinIO[IO_PB]);
            // Stop laser transmit mode
            } else if (newChar == '#') {
                txMode = TX_RADIO;
                PIO_Set(&pins[3]);
                PIO_EnableIt(&pinIO[IO_PB]);
                PIT_DisableIT();
            // Start laser receive mode
            } else if (newChar == '(') {
                rxLzFlag = RX_READY;
            // Stop laser receive mode
            } else if (newChar == ')') {
                rxLzFlag = FALSE;
            // Start duplex mode
            } else if (newChar == '!') {
                dpFlag = TRUE;
            // Stop duplex mode
            } else if (newChar == '@') {
                dpFlag = FALSE;
            // Start find mode
            } else if (newChar == '{') {
                searchFlag = SR_START;
            // Stop find mode
            } else if (newChar == '}') {
                searchFlag = FALSE;
            // Start 'be found' mode
            } else if (newChar == '[') {
                hideFlag = TRUE;
            // Stop 'be found' mode
            } else if (newChar == ']') {
                hideFlag = FALSE;
            
            // Keyboard remote control
            } else if (newChar == ';') {
                radioTx(&newChar, 1, 0);
            } else if (newChar == ',') {
                radioTx(&newChar, 1, 0);
            } else if (newChar == '.') {
                radioTx(&newChar, 1, 0);
            } else if (newChar == '/') {
                radioTx(&newChar, 1, 0);
            
            // Increase communication speed
            }  else if (newChar == '^') {
                txLzPeriod -= 10;

                sprintf(usbSendBuffer, "Tx Period: %d\r\n", txLzPeriod);
                CDCDSerialDriver_Write(usbSendBuffer, strlen(usbSendBuffer), 0, 0);

                ConfigureTc();
            // Decrease communication speed
            }  else if (newChar == '&') {
                txLzPeriod += 10;

                sprintf(usbSendBuffer, "Tx Period: %d\r\n", txLzPeriod);
                CDCDSerialDriver_Write(usbSendBuffer, strlen(usbSendBuffer), 0, 0);

                ConfigureTc();

            // Otherwise, add messages to the buffer
            } else {
                // Add char to buffer
                if (lenMsg < MAX_LEN_MSG && newChar != '\r') {
                
                    keyBuf[lenMsg] = newChar;
                    lenMsg++;
                    keyBuf[lenMsg] = '\0';
                }
                // If buffer is full, ready to transmit
                if (lenMsg == MAX_LEN_MSG || newChar == '\r') {

                    transmitFlag = lenMsg;
                    memcpy(payload, keyBuf, MAX_LEN_MSG);
                    lenMsg = 0;

                    memset(keyBuf, 0, MAX_LEN_MSG);
                }
            }
        }
        
        // Check if bytes have been discarded
        if ((received == DATABUFFERSIZE) && (remaining > 0)) {
            TRACE_WARNING("UsbDataReceived: %u bytes discarded\n\r",
                  remaining);
        }
    }
    else {
        TRACE_WARNING( "UsbDataReceived: Transfer error\n\r");
    }
}

// Interrupt handler for the ADC. Signals that the conversion is finished by
// setting a flag variable.
static void ISR_Adc(void)
{
    unsigned int status;
    unsigned int id_channel;

    status = ADC_GetStatus(AT91C_BASE_ADC);
    TRACE_DEBUG("status =0x%X\n\r", status);
    
    // Iterate across adc channels and convert them
    for(id_channel=ADC_CHANNEL_2;id_channel<=ADC_CHANNEL_3;id_channel++) {
        if (ADC_IsChannelInterruptStatusSet(status, id_channel)) {
            TRACE_DEBUG("channel %d\n\r", id_channel);
            ADC_DisableIt(AT91C_BASE_ADC, id_channel);
            conversionDone |= 1<<id_channel;
        }
    }
}

// Interrupt handler for pushbutton 1.
void ISR_Bp(void) {

    // Check if the button has been pressed
    if (!PIO_Get(&pinIO[IO_PB])) {

        inputType = (inputType+1)%3;

        int i = 0;
        
        // Flicker the laser on button press
        for (i = 0; i < 10; ++i) {
            PIO_Clear(&pins[3]);
            UTIL_WaitTimeInUs(BOARD_MCK, 5000);
            PIO_Set(&pins[3]);
            UTIL_WaitTimeInUs(BOARD_MCK, 5000);
        }
    }
}

// Handler for PIT interrupt.
void ISR_Pit(void)
{
    unsigned int status, pit_value;

    // Read the PIT status register
    status = PIT_GetStatus() & AT91C_PITC_PITS;

    // Get PIT value and clear interrupt flag.
    pit_value = PIT_GetPIVR();
    
    // Incrememnt the pulse counter
    if (rxLzFlag == RX_DISCOVER || rxLzFlag == RX_RECEIVE) {
        rxLzPulseCounter++;
    }
}

// Interrupt handler for TC0 interrupt. Used for sending laser data
void ISR_Tc0(void) {
    // Clear status bit to acknowledge interrupt
    AT91C_BASE_TC0->TC_SR;

    // If in transmit mode, read the pin and toggle it.
    // Increase the counter
    if (txLzFlag == TRUE) {
        if (txLzBuffer[txLzCounter] == 0) {
            PIO_Clear(&pins[3]);
        } else if (txLzBuffer[txLzCounter] == 1) {
            PIO_Set(&pins[3]);
        }

        txLzCounter++;
    
        // If transmit is done, stop the timer
        if (txLzCounter == 44) {
            txLzCounter = 0;
            txLzFlag = FALSE;
            TC_Stop(AT91C_BASE_TC0);
        }
    }
    
}

// Interupt handler for laser
void ISR_Laser(void) {
    laserOn = PIO_Get(&pinIO[IO_LASER]);
    
    // If in hide mode, send back a + sign
    if (hideFlag==TRUE) {
        if (laserOn == TRUE) {
            unsigned char temp[] = "+";
            radioTx(temp, 1, 0);
        }
        return;
    // Start when nothing is recieving and a rising edge is received
    } else if (rxLzFlag == RX_READY && laserOn == TRUE) {
        rxLzFlag = RX_DISCOVER;
        rxLzCounter = 0;
        rxLzPulseCounter = 0;
        rxLzPeriod = 0;
        PIT_EnableIT();
    // Discover the clock cycle
    } else if (rxLzFlag == RX_DISCOVER) {
        rxLzPeriod=rxLzPulseCounter;
        
        sprintf(usbSendBuffer, "Discover: %d\r\n", rxLzPeriod*rxLzPeriodDiscover);
        CDCDSerialDriver_Write(usbSendBuffer, strlen(usbSendBuffer), 0, 0);
        
        rxLzFlag = RX_RECEIVE;
        rxLzCounter = 2;
        rxLzBuffer[1] = 1;
        rxLzPulseCounter = 0;
    // Receive the message
    } else if (rxLzFlag == RX_RECEIVE && rxLzCounter <= 44) {
        // If the message is much longer than the discovered time, increment it 2.
        // Otherwise incremement 1
        rxLzBuffer[rxLzCounter++] = !laserOn;
        if (rxLzPulseCounter > (3*rxLzPeriod)/2) {
            rxLzBuffer[rxLzCounter++] = !laserOn;
        }
        rxLzPulseCounter = 0;

        if (rxLzCounter >= 43) {
            PIT_DisableIT();
            rxLzFlag = RX_PROCESS;
        }
    }
}

// Initialise TWI/i2c and MMA8452Q - DO NOT MODIFY
void init_Twi(void) {
	unsigned char temp;

    // Enable MUX -- Turns Analog Pins 4&5 into TWI.
    PIO_Configure(pinsMux, PIO_LISTSIZE(pinsMux));
    PIO_Clear(&pinsMux[0]);
    PIO_Clear(&pinsMux[1]);
    
    AT91C_BASE_PMC->PMC_PCER = 1 << AT91C_ID_TWI;
    TWI_ConfigureMaster(AT91C_BASE_TWI, 400000, BOARD_MCK);
    TWID_Initialize(&twid, AT91C_BASE_TWI);

	i2c_init(&twid);

	TWID_Read(&twid, 0x1D, 0x2A, 1, &temp, 1, 0);
    temp = temp & ~(0x01);
    TWID_Write(&twid, 0x1D, 0x2A, 1, &temp, 1, 0);
   
    temp = SCALE >> 2;
    TWID_Write(&twid, 0x1D, 0x0E, 1, &temp, 1, 0);
   
    TWID_Read(&twid, 0x1D, 0x2A, 1, &temp, 1, 0);
    temp = temp & ~(0x38);
    TWID_Write(&twid, 0x1D, 0x2A, 1, &temp, 1, 0);
   
    TWID_Read(&twid, 0x1D, 0x2A, 1, &temp, 1, 0);
    temp = temp | 0x01;
    TWID_Write(&twid, 0x1D, 0x2A, 1, &temp, 1, 0);

}

// Initialise PWM
void init_Pwm(void) {
    // Enable PWMC peripheral clock
    AT91C_BASE_PMC->PMC_PCER = 1 << AT91C_ID_PWMC;

    // Set clock A for servo's at 50hz * Duty cycle (samples).
    PWMC_ConfigureClocks(2 * SERVO_PWM_FREQUENCY * SERVO_DUTY_CYCLE, 0, BOARD_MCK);

    // Configure PWMC channel for SERVO (center-aligned, inverted polarity)
    PWMC_ConfigureChannel(CHANNEL_SERVO1, AT91C_PWMC_CPRE_MCKA, AT91C_PWMC_CALG, AT91C_PWMC_CPOL);
    PWMC_SetPeriod(CHANNEL_SERVO1, SERVO_DUTY_CYCLE);
    PWMC_SetDutyCycle(CHANNEL_SERVO1, MIN_DEGREES);
    
    PWMC_ConfigureChannel(CHANNEL_SERVO2, AT91C_PWMC_CPRE_MCKA, AT91C_PWMC_CALG, AT91C_PWMC_CPOL);
    PWMC_SetPeriod(CHANNEL_SERVO2, SERVO_DUTY_CYCLE);
    PWMC_SetDutyCycle(CHANNEL_SERVO2, MIN_DEGREES);
    
    // Enable channel #1 and #2
    PWMC_EnableChannel(CHANNEL_SERVO1);
    PWMC_EnableChannel(CHANNEL_SERVO2);
}

// Initialise ADC
void init_Adc(void) {
    ADC_Initialize(AT91C_BASE_ADC,
                AT91C_ID_ADC,
                AT91C_ADC_TRGEN_DIS,
                0,
                AT91C_ADC_SLEEP_NORMAL_MODE,
                AT91C_ADC_LOWRES_10_BIT,
                BOARD_MCK,
                BOARD_ADC_FREQ,
                10,
                1200);
                
    ADC_EnableChannel(AT91C_BASE_ADC, ADC_CHANNEL_2);
    ADC_EnableChannel(AT91C_BASE_ADC, ADC_CHANNEL_3);

    // Configure interrupts..
    AIC_ConfigureIT(AT91C_ID_ADC, 0, ISR_Adc);
    AIC_EnableIT(AT91C_ID_ADC);
}

// Configures the pushbutton to generate interrupts when pressed.
void init_Button(void) {
    // Configure pios
    PIO_Configure(pinIO, 2); // Pin list of size 2

    // Initialize interrupts
    PIO_InitializeInterrupts(AT91C_AIC_PRIOR_LOWEST);
    PIO_ConfigureIt(&pinIO[IO_PB], (void (*)(const Pin *)) ISR_Bp);
    PIO_ConfigureIt(&pinIO[IO_LASER], (void (*)(const Pin *)) ISR_Laser);
    PIO_EnableIt(&pinIO[IO_PB]);
    PIO_EnableIt(&pinIO[IO_LASER]);
}

// Configure the PIT for transmit
void ConfigurePit(void) {

    // Initialize the PIT to the desired frequency
    PIT_Init(rxLzPeriodDiscover, BOARD_MCK / 1000000);

    // Configure interrupt on PIT
    AIC_DisableIT(AT91C_ID_SYS);
    AIC_ConfigureIT(AT91C_ID_SYS, AT91C_AIC_PRIOR_LOWEST, ISR_Pit);
    AIC_EnableIT(AT91C_ID_SYS);

    // Enable the pit
    PIT_Enable();
}

// Configure Timer Counter 0 to generate an interrupt as ofter as set in 
void ConfigureTc(void) {
    unsigned int div;
    unsigned int tcclks;

    // Enable peripheral clock
    AT91C_BASE_PMC->PMC_PCER = 1 << AT91C_ID_TC0;

    // Configure TC for a frequency based on rxLzPeriod and trigger on RC compare
    // timerFreq / desiredFreq
    TC_FindMckDivisor(1000000/txLzPeriod, BOARD_MCK, &div, &tcclks);
    TC_Configure(AT91C_BASE_TC0, tcclks | AT91C_TC_CPCTRG);
    AT91C_BASE_TC0->TC_RC = (BOARD_MCK / div) / (1000000/txLzPeriod);

    // Configure and enable interrupt on RC compare
    AIC_ConfigureIT(AT91C_ID_TC0, AT91C_AIC_PRIOR_HIGHEST, ISR_Tc0);
    AT91C_BASE_TC0->TC_IER = AT91C_TC_CPCS;
    AIC_EnableIT(AT91C_ID_TC0);
}

//------------------------------------------------------------------------------
/// Main function
//------------------------------------------------------------------------------
int main(void)
{
    TRACE_CONFIGURE(DBGU_STANDARD, 115200, BOARD_MCK);
    
    static int degreesX = 90;                   // Motor degrees
    static int degreesY = 90;

    static int fineDegreesX; 
    static int fineDegreesY;
    fineDegreesX = degreesX * FINE_DEGREES;     // Finer motor degrees
    fineDegreesY = degreesY * FINE_DEGREES;     // Does not improve accuracy, removes jitter

    // Accelerometer and joystick variables
    int accX = 0;
    int accY = 0;
    int joyX = 0;
    int joyY = 0;

    // Timestamps for delays
    unsigned int timestamp = 0;

    int i = 0;

    memset(rxLzBuffer, 0, 44);

    // Configure io pins
    PIO_Configure(pins, PIO_LISTSIZE(pins));
    PIO_Set(&pins[3]);

    // Configure RF pins
    PIO_Configure(pinsRF, PIO_LISTSIZE(pinsRF));
        
    // Initialise the nrf24l01+
    nrf24l01plus_init();
    
    // initialise tx buffer
    for(i=0; i<32; i++)
        tx_buf[i] = 0;  
    
    // USB
    CDCDSerialDriver_Initialize(); // BOT driver initialization
    while(USBD_GetState() >= USBD_STATE_CONFIGURED);
    
    // Configure TWI/i2c, ADC, PWM, Push Button.
    init_Twi();
    init_Adc();
    init_Pwm();
    init_Button();
    ConfigurePit();
    ConfigureTc();

    // Infinite loop
    while(1){
  
         
        //sprintf(usbSendBuffer, "X:mV %d \r\n",
        //    ConvADC2mV(ADC_GetConvertedData(AT91C_BASE_ADC, ADC_CHANNEL_2)));
        //CDCDSerialDriver_Write(usbSendBuffer, strlen(usbSendBuffer), 0, 0);

        // If in accelerometer, read the inputs
        if (inputType == ACC) {
            unsigned char readbytes[6] = {0,0,0,0,0,0};

            i2c_start();						        //Send start condition
		    i2c_sendbyte(WRITE_DEV(0x1D));	          	//Send Write Address
		    i2c_sendbyte(0x01);					        //Send OUT_X_MSB Register Address
		    i2c_repeat_start();					        //Send repeat start
		    i2c_sendbyte(READ_DEV(0x1D));		        //Send Read Address
            
            readbytes[0] = i2c_recvbyte(I2C_AK);	    //Receive OUT_X_MSB, send AK
            readbytes[1] = i2c_recvbyte(I2C_AK);        //Receive OUT_X_LSB, send AK
            readbytes[2] = i2c_recvbyte(I2C_AK);        //Receive OUT_Y_MSB, send AK
            readbytes[3] = i2c_recvbyte(I2C_AK);        //Receive OUT_Y_LSB, send AK
            readbytes[4] = i2c_recvbyte(I2C_AK);        //Receive OUT_Z_MSB, send AK
            readbytes[5] = i2c_recvbyte(I2C_NAK);       //Receive OUT_Z_LSB, send NAK

		    i2c_stop();	                                //Send Stop Condition
     		
            // The bytes read are stored as two's complement. Cast them.
            // Only use the MSB's as accuracy is not needed with this method
            accX = (signed char) readbytes[0];
            accY = (signed char) readbytes[2];
            
            // Set the motors
            if (accX < -20 || accX > 20) {
                fineDegreesX+=accX/5;
            }

            if (accY < -20 || accY > 20) {
                fineDegreesY+=accY/5;
            }
        
        // If input in joystick mode, read the inputs
        } else if (inputType == JOY) {
            // ADC
            conversionDone = 0;
            // Re-enable interupts for new measurement.
            ADC_EnableIt(AT91C_BASE_ADC, ADC_CHANNEL_2);
            ADC_EnableIt(AT91C_BASE_ADC, ADC_CHANNEL_3);
            // Start measurement
            ADC_StartConversion(AT91C_BASE_ADC);        
            while(conversionDone != ((1<<ADC_CHANNEL_2)|(1<<ADC_CHANNEL_3)));
    
            joyX = ADC_GetConvertedData(AT91C_BASE_ADC, ADC_CHANNEL_2);
            joyY = ADC_GetConvertedData(AT91C_BASE_ADC, ADC_CHANNEL_3);
            
            // Move the servos if the joystick has been moved significantly
            if (joyX < 400 || joyX > 600) {
                fineDegreesX+=(joyX-512)/100;
            }

            if (joyY < 400 || joyY > 600) {
                fineDegreesY+=(joyY-512)/100;
            }

        // If in keyboard mode, move the servo using keys
        } else if (inputType == KEY) {

            if (inputKey == 'w') {
                fineDegreesY+=FINE_DEGREES;
            } else if (inputKey == 'a') {
                fineDegreesX+=FINE_DEGREES;
            } else if (inputKey == 's') {
                fineDegreesY-=FINE_DEGREES;
            } else if (inputKey == 'd') {
                fineDegreesX-=FINE_DEGREES;
            }

            inputKey = 0;
        }
        
        // If searching for laser receiver
        if (searchFlag) {
            // Initialise variables and begin searching
            if (searchFlag == SR_START) {
                searchRow = FALSE;
                fineDegreesX = FINE_MIN_DEGREES;
                fineDegreesY = FINE_MIN_DEGREES;
                searchDir = SR_FORWARD;
                searchRadioFlag = FALSE;
                searchTimestamp = timestamp - 1;

                searchFlag = SR_STARTWAIT;
            // Wait until servo is at start point
            } else if (searchFlag == SR_STARTWAIT) {
                if (searchTimestamp == timestamp) {
                    searchFlag = SR_SCANROW;
                }
            // Increase the degrees until outside range
            // Reverse direction when outside range
            } else if (searchFlag == SR_SCANROW) {
                if (fineDegreesX <= FINE_MAX_DEGREES && fineDegreesX >= FINE_MIN_DEGREES) {
                    fineDegreesX += FINE_DEGREES*searchDir;                
                }
                
                if (fineDegreesX >= FINE_MAX_DEGREES || fineDegreesX <= FINE_MIN_DEGREES) {
                    if (searchDir == SR_FORWARD) {
                        searchDir = SR_BACKWARD;
                    } else {
                        searchDir = SR_FORWARD;
                    }
    
                    searchTimestamp = timestamp - 1;
                    searchFlag = SR_SHIFTROW;
                }

            // Wait and see if there was a radio message
            // If there was, move to the next stage
            // Otherwise start searching again
            } else if (searchFlag == SR_SHIFTROW) {
                if (searchTimestamp == timestamp) {
                    if (searchRadioFlag == TRUE) {
                        searchRadioFlag = FALSE;
                        searchFlag = SR_FOUNDROW;                       
                    } else {
                        fineDegreesY+=FINE_DEGREES;
                        searchFlag = SR_SCANROW;
                    }
                }
            // The row the radio is on is known, start scanning slowly backwards
            } else if (searchFlag == SR_FOUNDROW) {
               if (searchRadioFlag == TRUE) {
                    searchFlag = FALSE;
                    txMode = TX_LASER;
                    PIO_Clear(&pins[3]);
                    PIO_DisableIt(&pinIO[IO_PB]);
                    unsigned char temp[] = "-";
                    radioTx(temp, 1, 0);
                } else {
                    fineDegreesX += 2*searchDir;
                }
            }
        }
        
        // Set servos
        fineDegreesX = servo_degrees(fineDegreesX, FINE_MIN_DEGREES, FINE_MAX_DEGREES);
        fineDegreesY = servo_degrees(fineDegreesY, FINE_MIN_DEGREES, FINE_MAX_DEGREES);
        
        degreesX = servo_degrees(fineDegreesX/FINE_DEGREES, MIN_DEGREES, MAX_DEGREES);
        degreesY = servo_degrees(fineDegreesY/FINE_DEGREES, MIN_DEGREES, MAX_DEGREES);

        // Delay printing
        timestamp++;            
        if (timestamp == 60) {
            UTIL_WaitTimeInUs(BOARD_MCK, 10000);
            sprintf(usbSendBuffer, "Pan: %d Tilt: %d\r\n", degreesX, degreesY);
            CDCDSerialDriver_Write(usbSendBuffer, strlen(usbSendBuffer), 0, 0);
            timestamp = 0;
        }
        
        // Set the motors
        PWMC_SetDutyCycle(CHANNEL_SERVO1, degreesX);
        PWMC_SetDutyCycle(CHANNEL_SERVO2, degreesY);

        // On packet receive
        if(nrf24l01plus_receive_packet()){ 
            unsigned char unencodedMsg[32];
            unsigned char *msg;

            memcpy(unencodedMsg, rx_buf, 32);

            UTIL_WaitTimeInUs(BOARD_MCK, 2000);
            sprintf(usbSendBuffer, "%02x %02x%02x%02x%02x %02x%02x%02x%02x %s \r\n", rx_buf[0],rx_buf[1],rx_buf[2],rx_buf[3],rx_buf[4],rx_buf[5],rx_buf[6],rx_buf[7],rx_buf[8],&rx_buf[9]);
            //CDCDSerialDriver_Write(usbSendBuffer, strlen(usbSendBuffer), 0, 0);
            
            // Check src and destination of message
            // Also can receive messages from self, if the code must be flashed on
            // to multiple boards
            if ((!memcmp(&unencodedMsg[1], srcId, 4) || !memcmp(&unencodedMsg[1], dstId, 4)) && (!memcmp(&unencodedMsg[5], dstId, 4) || !memcmp(&unencodedMsg[5], baseId, 4) || !memcmp(&unencodedMsg[5], srcId, 4))) {
                msg = &unencodedMsg[9];

                UTIL_WaitTimeInUs(BOARD_MCK, 2000);
                sprintf(usbSendBuffer, "RECEIVED FROM RADIO: %s \r\n", msg);
                CDCDSerialDriver_Write(usbSendBuffer, strlen(usbSendBuffer), 0, 0);
                
                // If in duplex mode and there's an error, retransmit
                if (dpFlag == TRUE && !memcmp(msg, errStr, 6)) {
                    laserTx(lastChar);
                }
            
                // If in search mode, tell it there's been feedback from remote station
                if (searchFlag && msg[0] == '+') {
                    searchRadioFlag = TRUE;
                } else if (msg[0] == '-') {
                    hideFlag = FALSE;
                // Check if there's been any remote keyboard input
                } else if (msg[0] == ';') {
                    fineDegreesY+=FINE_DEGREES;
                } else if (msg[0] == ',') {
                    fineDegreesX+=FINE_DEGREES;
                } else if (msg[0] == '.') {
                    fineDegreesY-=FINE_DEGREES;
                } else if (msg[0] == '/') {
                    fineDegreesX-=FINE_DEGREES;
                }
            }
        }
        
        // Process the laser input
        if (rxLzFlag == RX_PROCESS) {
            static unsigned char buf[44] = {0};
            unsigned char *retarray;

            memcpy(buf, rxLzBuffer, 44);
            // Decode the buffer into char and errors
            retarray = decodeManchester(buf);
            
            UTIL_WaitTimeInUs(BOARD_MCK, 10000);
            sprintf(usbSendBuffer, "RECEIVED FROM LASER: %c (Errors = %d::%d, Corrected = %d::%d)\r\n", retarray[0], retarray[1], retarray[2], retarray[3], retarray[4]);
            CDCDSerialDriver_Write(usbSendBuffer, strlen(usbSendBuffer), 0, 0);
            
            // If in duplex, transmit back through the radio
            if (dpFlag == TRUE) {
                if (retarray[1] == 0 && retarray[2] == 0) {
                    unsigned char tempStr[2];
                    tempStr[0] = retarray[0];
                    tempStr[1] = 0;
                    radioTx(tempStr, 2, 1);
                } else {
                    radioTx(errStr, 6, 1);    
                }
            }
            memset(rxLzBuffer, 0, 44);
            rxLzFlag = RX_READY;
        }

        if (rxLzFlag == RX_RECEIVE && rxLzPulseCounter > 1000000) {
            memset(rxLzBuffer, 0, 44);
            rxLzFlag = RX_READY;
            radioTx(errStr, 6, 1);
        }

        // Check for user input on debug line.
        CDCDSerialDriver_Read(usbBuffer, DATABUFFERSIZE, 
            (TransferCallback) UsbDataReceived,0);

        // Send a packet if necessary
        if(transmitFlag && (txMode == TX_RADIO)) {

            radioTx(payload, transmitFlag, 1);

            transmitFlag = FALSE;
        // Send a laser message if necessary
        } else if (transmitFlag && (txMode == TX_LASER)) {
 
            laserTx(payload[transmitFlag-1]);
            
            transmitFlag = FALSE;
        }

        UTIL_WaitTimeInUs(BOARD_MCK, 2000); // delay
    }
}
