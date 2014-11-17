// code to wreck the flash in a HypnoLSD module
// used for testing read/write capability of flash over time

#include <xc.h>   // standard part port definitions
#include <plib.h> // peripherial definitions


#define SYS_CLOCK 48000000L   // system clock speed

// version stored in nibbles MAJOR.MINOR
#define VERSION  0x10 // version 1.0, Nov 2014

/* Version history
 * 1.0 - intial release
  */

#define TEXT_SIZE 100 // size of input/output buffer



/**********************
 * Configuration Bits *
 **********************/

//#pragma config UPLLEN   = OFF           // USB PLL Enabled
//#pragma config UPLLIDIV = DIV_3         // USB PLL Input Divider

#pragma config FPLLMUL  = MUL_24        // PLL Multiplier
#pragma config FPLLIDIV = DIV_3         // PLL Input Divider
#pragma config FPLLODIV = DIV_2         // PLL Output Divider
#pragma config FPBDIV   = DIV_1         // Peripheral Clock divisor

#pragma config FWDTEN   = OFF           // Watchdog Timer
#pragma config WDTPS    = PS1           // Watchdog Timer Postscale
#pragma config FCKSM    = CSDCMD        // Clock Switching & Fail Safe Clock Monitor
#pragma config OSCIOFNC = OFF           // CLKO Enable
#pragma config POSCMOD  = HS            // Primary Oscillator
#pragma config IESO     = OFF           // Internal/External Switch-over
#pragma config FSOSCEN  = OFF           // Secondary Oscillator Enable (KLO was off)
#pragma config FNOSC    = PRIPLL        // Oscillator Selection
#pragma config CP       = ON            // Code Protect
#pragma config BWP      = OFF           // Boot Flash Write Protect
#pragma config PWP      = OFF           // Program Flash Write Protect
#pragma config ICESEL   = ICS_PGx2      // ICE/ICD Comm Channel Select
#pragma config DEBUG    = OFF           // Background Debugger Enable
    // todo - turn on watchdog timer? tickle


/*********************** UART code ********************************************/

// select which to use
#if 1
#define HC_UART1
#else
#define HC_UART2
#endif

//The desired startup baudRate
#define DESIRED_BAUDRATE   (9600)
UINT16 clockDivider = SYS_CLOCK/(4*DESIRED_BAUDRATE) - 1;

// if there is any UART error, return 1, else return 0
int UARTError()
{
        int err =
        U1STAbits.OERR | // overrun error
        U1STAbits.FERR | // framing error
        U1STAbits.PERR ; // parity error

        err |=
        U2STAbits.OERR | // overrun error
        U2STAbits.FERR | // framing error
        U2STAbits.PERR ; // parity error

        return err!=0?1:0;
}

// blocking call to write byte to UART
void UARTWriteByte(char byte)
{
#ifdef HC_UART1
        while (!U1STAbits.TRMT);   // wait till bit clear, then ready to transmit
        U1TXREG = byte; // write a byte
#else
        while (!U2STAbits.TRMT);   // wait till bit clear, then ready to transmit
        U2TXREG = byte; // write a byte
#endif
}

// print the message to the serial port.
void PrintSerial(const char * message)
{
    while (*message != 0)
    {
        UARTWriteByte(*message);
        message++;
    }
}

// set the baud rate divider
// The baud is Floor[80000000/(4*(divider+1)].
void SetUARTClockDivider(UINT16 divider)
{

	// define setup Configuration 1 for OpenUARTx
		// Module Enable
		// Work in IDLE mode
		// Communication through usual pins
		// Disable wake-up
		// Loop back disabled
		// Input to Capture module from ICx pin
		// no parity 8 bit
		// 1 stop bit
		// IRDA encoder and decoder disabled
		// CTS and RTS pins are disabled
		// UxRX idle state is '1'
		// 4x baud clock - high speed
	#define config1 UART_EN | UART_IDLE_CON | UART_RX_TX | UART_DIS_WAKE | UART_DIS_LOOPBACK | UART_DIS_ABAUD | UART_NO_PAR_8BIT | UART_1STOPBIT | UART_IRDA_DIS | UART_DIS_BCLK_CTS_RTS| UART_NORMAL_RX | UART_BRGH_FOUR

    // define setup Configuration 2 for OpenUARTx
		// IrDA encoded UxTX idle state is '0'
		// Enable UxRX pin
		// Enable UxTX pin
		// No interrupt on transfer of every character to TSR
		// Interrupt on every char received
		// Disable 9-bit address detect
		// Rx Buffer Over run status bit clear
	#define config2 UART_TX_PIN_LOW | UART_RX_ENABLE | UART_TX_ENABLE | /*UART_INT_TX |  UART_INT_RX_CHAR | */ UART_ADR_DETECT_DIS | UART_RX_OVERRUN_CLEAR


#ifdef HC_UART1
    // Open UART1 with config1 and config2
    OpenUART1( config1, config2, divider);

    // Configure UART RX Interrupt
    //ConfigIntUART1(UART_INT_PR2 | UART_RX_INT_EN /* | UART_TX_INT_EN */ );
#else
    // Open UART2 with config1 and config2
    OpenUART2( config1, config2, divider);

    // Configure UART RX Interrupt
    //ConfigIntUART2(UART_INT_PR2 | UART_RX_INT_EN /* | UART_TX_INT_EN */ );
#endif
    clockDivider = divider;
}

// init the UART settings
void InitializeUART(int clock)
{

     clockDivider = clock/(4*DESIRED_BAUDRATE) - 1;

     // UART1
     // U1RX on RPA4
     // U1TX on RPA0
     mPORTAClearBits(BIT_0 | BIT_4);
     mPORTASetPinsDigitalIn(BIT_4);
     mPORTASetPinsDigitalOut(BIT_0);
     U1RXR = 2; // RPB2 = U1RX
     RPA0R = 1; // PIN RPA0 = U1TX

    SetUARTClockDivider(clockDivider);
}


/*********************** FLASH CODE *******************************************/

// Note that:
 //    "bytes" needs to be a multiple of BYTE_PAGE_SIZE (and aligned that way) if you intend to erase
 //    "bytes" needs to be a multiple of BYTE_ROW_SIZE (and aligned that way) if you intend to write rows
 //    "bytes" needs to be a multiple of sizeof(int) if you intend to write words
 #define NVM_ALLOCATE(name, align, bytes) volatile UINT8 name[(bytes)] \
     __attribute__ ((aligned(align),section(".text,\"ax\",@progbits #"))) = \
     { [0 ...(bytes)-1] = 0xFF }

NVM_ALLOCATE(RomData,BYTE_PAGE_SIZE,STATIC_BLOCK_SIZE);  // allocates "RomData" of 8k bytes, aligned on erasable page boundary

// all system settings - keep a copy here for eash reference
// int FlashPageErasureCount, todo - how does this counter work?
// long Blits
// long mainLoopPasses;
// unsigned int profileInterruptTime;
// long mainLoopPasses;
// long completePowerUps;
// long completePowerDowns;
// int interruptTime;

UINT32 flashErasedCount = 0;
UINT32 flashBadChecksums = 0;
UINT32 flashNoBlocksFound = 0;
UINT32 flashCannotGetBlockErrors = 0;
UINT32 flashWriteFailedCount = 0;

// call this to erase all settings
void FlashErase()
{
    flashErasedCount++;

    // erase pages
    int index = 0;
    while (index < STATIC_BLOCK_SIZE)
    {
        NVMErasePage((void*)(RomData + index)); // ignore return codes here
        index += BYTE_PAGE_SIZE;
    }
}


// read flash storage block. Return 1 on success, else 0
// size is at most 126 bytes
// on failure buffer is 0 filled
int FlashReadBlock(UINT8 * buffer, int size)
{
    int index;

    if ((size < 0) || (MAX_BUFFER_SIZE < size))
        return 0; // out of bounds failure

    // zero buffer
    for (index = 0; index < size; ++index)
        buffer[index] = 0;
    // find last free good one
    index = STATIC_BLOCK_SIZE-128;
    while (index >= 0)
    {
        if (RomData[index] < 128)
        { // block here - if checksum is good, we're done
            int index2, checksum=0;
            int goodChecksum = RomData[index]*256+RomData[index+1];
            for (index2 = 0; index2 < size; index2++)
                checksum += RomData[index+index2+2];
            checksum &= (1<<CHECKSUM_BITS)-1; // mask off unused bits
            if (checksum == goodChecksum)
            { // good entry. Return it
                for (index2 = 0; index2 < size; ++index2)
                    buffer[index2] = RomData[index+index2+2];
                return 1;
            }
            else
            {
                flashBadChecksums++;
            }
        }
        index -= 128; // try previous one
    }
    flashNoBlocksFound++;
    return 0; // none found
}

// write flash storage block. Return 1 on success, else 0
// size is at most 126 bytes
int FlashWriteBlock(UINT8*buffer, int size)
{
    UINT8 buf2[128];
    int index;
    int index2, checksum=0;
    unsigned int word;

    if ((size < 0) || (MAX_BUFFER_SIZE < size))
        return 0; // out of bounds failure

    // try and find free row index
    index = 0;
    while ((index < STATIC_BLOCK_SIZE) && (RomData[index] < 128))
        index += 128; // try next one

    if (index >= STATIC_BLOCK_SIZE)
    { // went over the top, time to clear the flash block, and start over
        FlashErase();

        // try and find free row index, this time failure is failure :)
        index = 0;
        while ((index < STATIC_BLOCK_SIZE) && (RomData[index] < 128))
            index += 128; // try next one

        if (index >= STATIC_BLOCK_SIZE)
        {
            flashCannotGetBlockErrors++;
            return 0; // off the top. Must be an error
        }
    }

    // zero temp buffer
    memset(buf2,0,128);

    // create checksum and prepare temp buffer
    for (index2 = 0; index2 < size; ++index2)
    {
        checksum += buffer[index2];
        buf2[index2+2] = buffer[index2];
    }
    checksum &= (1<<CHECKSUM_BITS)-1;
    buf2[0] = (checksum>>8);
    buf2[1] = (checksum&255);

    // write the data 4 bytes at a time, checksum part last
    for (index2 = 128-4; index2 >= 0; index2 -= 4)
    {
        word  = buf2[index2+3] << 24;
        word += buf2[index2+2] << 16;
        word += buf2[index2+1] <<  8;
        word += buf2[index2  ]      ;
        if (NVMWriteWord((void*)(RomData+index+index2),word) != 0)
        {
            flashWriteFailedCount++;
            return 0; // failed
        }
    }

    return 1; // success
}


/*********************** main code ********************************************/

// initialize hardware
void Initialize()
{
        // All of these items will affect the performance of your code and cause it to run significantly slower than you would expect.
	SYSTEMConfigPerformance(SYS_CLOCK);

	WriteCoreTimer(0); // Core timer ticks once every two clocks (verified)

	// set default digital port A for IO
	DDPCONbits.JTAGEN = 0; // turn off JTAG
	DDPCONbits.TROEN = 0; // ensure no tracing on
	//mPORTASetPinsDigitalOut(BIT_0 | BIT_1 | BIT_2 | BIT_3 | BIT_4 | BIT_5 | BIT_6 | BIT_7);

        // todo - set this based on width of image. make rest inputs?
        mPORTBSetPinsDigitalOut(BIT_0|BIT_1|BIT_2|BIT_3|BIT_4|BIT_5|BIT_6|BIT_7|BIT_8|BIT_9|BIT_10|BIT_11|BIT_12|BIT_13|BIT_14|BIT_15);

        // Configure the device for maximum performance but do not change the PBDIV
        // Given the options, this function will change the flash wait states, RAM
        // wait state and enable prefetch cache but will not change the PBDIV.
        // The PBDIV value is already set via the pragma FPBDIV option above..
        int pbClk = SYSTEMConfig( SYS_CLOCK, SYS_CFG_WAIT_STATES | SYS_CFG_PCACHE);

        InitializeUART(pbClk);

        mPORTASetPinsDigitalOut(BIT_1);


        // set internals
        // SetUARTClockDivider(flashOptions.baudDivisor);

	// prepare 32 bit timer 45 to trigger interrupts
        //OpenTimer45(T45_ON | T45_SOURCE_INT | T45_PS_1_1, interruptTime);

        // set up the timer interrupt and priority
	//ConfigIntTimer45(T4_INT_ON | T4_INT_PRIOR_7);

        // enable multivectored interrupts
	//INTEnableSystemMultiVectoredInt();

	// start watchdog timer
	//tickle in interrupt, turn off during reset of device, causes a reset
	//The next statement enables the Watchdog Timer:
	// WDTCONbits.ON = 1;

        //sprintf(text,"Wait states %d\r\n.",BMXCONbits.BMXWSDRM);
        //PrintSerial(text);
        //BMXCONbits.BMXWSDRM = 0; // set RAM access to zero wait states

        //sprintf(text,"TODO _ REMOVE:override %d\r\n",pinOverride);
        //PrintSerial(text);
        //sprintf(text,"TODO _ REMOVE:actual %d\r\n",PORTAbits.RA1);
        //PrintSerial(text);
}

char text[TEXT_SIZE];

int main(void)
{
    Initialize();

    sprintf(text,"\r\n\r\nHypnocube FLASH destroyer %d.%d\r\n",VERSION>>4, VERSION&15);    
    PrintSerial(text);

    


    return 0;
}


