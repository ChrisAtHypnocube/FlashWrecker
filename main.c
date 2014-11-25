// code to wreck the flash in a HypnoLSD module
// used for testing read/write capability of flash over time

#include <xc.h>   // standard part port definitions
#include <plib.h> // peripherial definitions
#include <stdbool.h>
#include <stdint.h>  // basic types like uint16_t, etc.


#define SYS_CLOCK 48000000L   // system clock speed

// version stored in nibbles MAJOR.MINOR
#define VERSION  0x10 // version 1.0, Nov 2014

/* Version history
 * 1.0 - intial release
  */

#define TEXT_SIZE 100 // size of input/output buffer

// place to format messages
char text[TEXT_SIZE];


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
#define DESIRED_BAUDRATE   (1000000)
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


// read byte if one is ready.
// if exists, return true and byte
// if return false, byte = 0 is none avail, else
// byte != 0 means error
bool UARTReadByte(uint8_t * byte)
    {
    if (UARTReceivedDataIsAvailable(UART1))
    {
        *byte = UARTGetDataByte(UART1);
        return true;
    }
    *byte = 0;
    return false;
    } // UARTReadByte


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

// value of an erased flash word.
// Bits can be flipped away from this state without requiring a page reset
// used to tell if a word has not been written
#define FLASH_ERASED_WORD_VALUE 0xFFFFFFFF


#if defined(__32MX150F128C__) || defined(__32MX150F128B__)
#define FLASH_MODEL_ROW_SIZE_WORDS    32   // # of 32-bit Instructions per Row
#define FLASH_MODEL_PAGE_SIZE_WORDS  256   // # of 32-bit Instructions per Page
#endif

#ifdef BYTE_PAGE_SIZE
#undef BYTE_PAGE_SIZE
#endif
#define BYTE_PAGE_SIZE ((FLASH_MODEL_PAGE_SIZE_WORDS)*4)

// size of FLASH to test in bytes
#define FLASH_BLOCK_SIZE_BYTES (BYTE_PAGE_SIZE*16) // # of pages in flash

// a frame is how much we test at once
#define PAGES_PER_FRAME 2
// number of frames
#define FRAMES (FLASH_BLOCK_SIZE_BYTES/BYTE_PAGE_SIZE)/(PAGES_PER_FRAME)

// 32 bit words per frame
#define WORDS_PER_FRAME ((PAGES_PER_FRAME)*((BYTE_PAGE_SIZE)/4))


// Note that:
 //    "bytes" needs to be a multiple of BYTE_PAGE_SIZE (and aligned that way) if you intend to erase
 //    "bytes" needs to be a multiple of BYTE_ROW_SIZE (and aligned that way) if you intend to write rows
 //    "bytes" needs to be a multiple of sizeof(int) if you intend to write words
 #define NVM_ALLOCATE(name, align, bytes) volatile UINT8 name[(bytes)] \
     __attribute__ ((aligned(align),section(".text,\"ax\",@progbits #"))) = \
     { [0 ...(bytes)-1] = 0xFF }

NVM_ALLOCATE(FlashData,BYTE_PAGE_SIZE,FLASH_BLOCK_SIZE_BYTES);  // allocates "RomData" of 8k bytes, aligned on erasable page boundary

UINT32 flashErasedCount = 0;
UINT32 flashBadChecksums = 0;
UINT32 flashNoBlocksFound = 0;
UINT32 flashCannotGetBlockErrors = 0;
UINT32 flashWriteFailedCount = 0;

// map a virtual address to a physical address.
// needed for FLASH writing and erasing routines
// needed for any hardware peripherals
// todo - move to utils.h ultimately
void * VirtualToPhysicalAddress(const void * virtualAddress)
{
    uint32_t addr = (uint32_t)(virtualAddress);
    return (void*)(addr & 0x1FFFFFFF);

    /* from http://ww1.microchip.com/downloads/en/DeviceDoc/61115F.pdf
    3.4.1 Virtual to Physical Address Calculation (and Vice-Versa)

    1. virtual kernel KSEG0 or KSEG1 to physical:
    ? Physical Address = Virtual Address and 0x1FFFFFFF
    2. physical to KSEG0 virtual
    ? KSEG0 Virtual Address = Physical Address | 0x80000000
    3. physical to KSEG1 virtual
    ? KSEG1 Virtual Address = Physical Address | 0xA0000000

    4. To translate from KSEG0 virtual to KSEG1 virtual address
    ? KSEG1 Virtual Address = KSEG0 Virtual Address | 0x20000000

    */
}



// call this to erase pages
// return true on success, else false
bool FlashErase(uint32_t startOffset, uint32_t length)
{
    uint32_t offset = startOffset;
    uint32_t endOffset = startOffset + length;
    int errors = 0;
    while (offset < endOffset)
    {
        uint32_t * ptr = (void*)FlashData;
        ptr   += offset;
        void * physicalAddress = VirtualToPhysicalAddress(ptr);

        // Claim: Microchip has serious bug in their PIC32 peripheral lib - erase page depends on series
        // from http://www.microchip.com/forums/m772388.aspx
        // next solution: In "PIC32 Flash Programming Specification" DS61145L, Table 5.1: I finally found the details.
        // Flash page size 256 Word for PIC32MX1 and MX2. Flash page size 1024 Words for MX3 and higher, and 4096 Words for MZ devices.
        // shizzle fuck :)

        // needs PHYSICAL address, not virtual address
        if (NVMErasePage(physicalAddress) != 0)
        {   // double try
            if (NVMErasePage(physicalAddress) != 0)
            {
                errors++;
            }
        }
        offset += BYTE_PAGE_SIZE/4; // note this size changes by PIC model

    }
    if (errors == 0)
        return true;
    return false;
}


uint32_t FlashRead(uint32_t offset)
{
    uint32_t * ptr = (void*)FlashData;
    return *(ptr+offset);
}

// return true on success, else false
bool FlashWrite(uint32_t wordOffset, uint32_t value)
{
    if (FLASH_BLOCK_SIZE_BYTES / 4 <= wordOffset)
        return false; // out of bounds

    uint32_t * ptr = (void*)FlashData;
    ptr += wordOffset;
    void * physicalAddress = VirtualToPhysicalAddress(ptr);

    // try writing twice, and check the value each try
    if (NVMWriteWord(physicalAddress, value) != 0 || *ptr != value)
    {
        if (NVMWriteWord(physicalAddress, value) != 0 || *ptr != value)
        {
            return false;
        }
    }
    return true;
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

// RAM buffer to track recent reads
uint32_t ramBufferErased[WORDS_PER_FRAME];
uint32_t ramBufferWritten[WORDS_PER_FRAME];

void InitRamFrame()
{
    PrintSerial("Resetting RAM copy.\r\n");
    int i;
    for (i = 0; i < WORDS_PER_FRAME; ++i)
    {
        ramBufferErased[i]  = FLASH_ERASED_WORD_VALUE;
        ramBufferWritten[i] = ~FLASH_ERASED_WORD_VALUE;
    }
}

void OutputError(uint8_t charType, uint32_t offset, uint32_t readWord, uint32_t desiredWord)
{
    snprintf(text,TEXT_SIZE,"ERROR: (%c) offset %08X read %08X desired %08X.\r\n",charType,offset,readWord,desiredWord);
    PrintSerial(text);
}

uint32_t passes[FRAMES];


int main(void)
{
    Initialize();

    sprintf(text,"\r\n\r\nHypnocube FLASH destroyer %d.%d\r\n",VERSION>>4, VERSION&15);    
    PrintSerial(text);

    sprintf(text,"Testing %d frames of %d bytes each.\r\n",FRAMES,WORDS_PER_FRAME*4);
    PrintSerial(text);

    sprintf(text,"Flash starts at 0x%08x.\r\n",FlashData);
    PrintSerial(text);

    int frame = 0;
    int ramFrame = -1;
    int i;
    uint32_t errors = 0;

    WriteCoreTimer(0);
    while (1)
    {
        uint32_t startOffset = WORDS_PER_FRAME*frame;

        if (ramFrame != frame)
        {
            InitRamFrame();
            ramFrame = frame;
        }

        ++passes[frame];

        // message for start
        uint32_t time = ReadCoreTimer();
        sprintf(text,"Pass %d, frame %d, offset %08x, time %08x, errors %d\r\n",
                passes[frame],frame,startOffset,time,errors
                );
        PrintSerial(text);

        // erase frame
        if (!FlashErase(startOffset,WORDS_PER_FRAME))
            PrintSerial("ERROR: flash erase failed.\r\n");

        // check all entries are FLASH_ERASED_WORD_VALUE
        for (i = 0; i < WORDS_PER_FRAME; ++i)
        {
            uint32_t word = FlashRead(startOffset+i);
            if (ramBufferErased[i] != word)
            { // an error, output, and set buffer
                OutputError('E',startOffset+i,word,ramBufferErased[i]);
                ramBufferErased[i] = word;
                ++errors;
            }
        }

        // write all ~FLASH_ERASED_WORD_VALUE
        for (i = 0; i < WORDS_PER_FRAME; ++i)
        {
            if (!FlashWrite(startOffset+i,~(FLASH_ERASED_WORD_VALUE)))
            {
                snprintf(text,TEXT_SIZE,"ERROR: flash write failed ar offset %08X.\r\n",startOffset+i);
                PrintSerial(text);
            }
        }

        // check all entries are FLASH_ERASED_WORD_VALUE
        for (i = 0; i < WORDS_PER_FRAME; ++i)
        {
            uint32_t word = FlashRead(startOffset+i);
            if (ramBufferWritten[i] != word)
            { // an error, output, and set buffer
                OutputError('W',startOffset+i,word,ramBufferWritten[i]);
                ramBufferWritten[i] = word;
                ++errors;
            }
        }

        // check if frame changed
        uint8_t byte;
        if (UARTReadByte(&byte))
        {
            snprintf(text,TEXT_SIZE,"Command received %c [%02x].\r\n",byte,byte);
            PrintSerial(text);
            if ('0' <= byte && byte <= FRAMES+'0'-1)
            { // change frame
                frame = byte-'0';
                snprintf(text,TEXT_SIZE,"Switching to frame %d.\r\n",frame);
                PrintSerial(text);
            }
            if (byte == 'q')
                break;
        }
    }

    PrintSerial("Quitting...\r\n");
    return 0;
}


