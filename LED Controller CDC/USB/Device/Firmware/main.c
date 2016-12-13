/*********************************************************************************************
 * MAIN file for LED COntroller CDC
 * Adapted from Device - CDC - Basic Demo
 * Microchip Solutions V2013-06-15
 * 
 * 4-19-16: CDC serial port working very well with 4096 byte transfers & webcam very smooth
 * 5-19-16: Cleared out definitions for other processors
 * 5-20-16: Works great with Visual Studio C# CDC LED Controller
 * Offset error fixed. Works swell with four panels.
 * 6-1-16: Restored no compression version. Works nicely with four panels.
 * 6-3-16: Added 32x32 matrix capability, using tempPanel[] to store half of panel
 *          until second half is received.
 * 6-7-16: Test with three serial ports and VS
 * 6-28-16: Modified to work with one byte color using color depth = 3
 * 12-13-16: First version stored on GitHub
 *********************************************************************************************/
#define TEST_OUT LATCbits.LATC4
#include "Defs.h"
#include "USB/usb.h"
#include "usb_function_cdc.h"
#include "HardwareProfile.h"
#include "GenericTypeDefs.h"
#include "Compiler.h"
#include "usb_config.h"
#include "USB/usb_device.h"


#pragma config UPLLEN   = ON            // USB PLL Enabled
#pragma config FPLLMUL  = MUL_20        // PLL Multiplier
#pragma config UPLLIDIV = DIV_2         // USB PLL Input Divider
#pragma config FPLLIDIV = DIV_2         // PLL Input Divider
#pragma config FPLLODIV = DIV_1         // PLL Output Divider
#pragma config FPBDIV   = DIV_1         // Peripheral Clock divisor
#pragma config FWDTEN   = OFF           // Watchdog Timer
#pragma config WDTPS    = PS1           // Watchdog Timer Postscale
#pragma config FCKSM    = CSDCMD        // Clock Switching & Fail Safe Clock Monitor
#pragma config OSCIOFNC = OFF           // CLKO Enable
#pragma config POSCMOD  = HS            // Primary Oscillator
#pragma config IESO     = OFF           // Internal/External Switch-over
#pragma config FSOSCEN  = OFF           // Secondary Oscillator Enable (KLO was off)
#pragma config FNOSC    = PRIPLL        // Oscillator Selection
#pragma config CP       = OFF           // Code Protect
#pragma config BWP      = OFF           // Boot Flash Write Protect
#pragma config PWP      = OFF           // Program Flash Write Protect
#pragma config ICESEL   = ICS_PGx2      // ICE/ICD Comm Channel Select


/** V A R I A B L E S ********************************************************/
#define USBPACKETSIZE 64
unsigned char USBreplyBuffer[USBPACKETSIZE] = {1, 2, 3, 4};
unsigned char USBerrorBuffer[USBPACKETSIZE] = {9, 8, 7, 6};
#define MAX_PANEL_DATASIZE (32 * 16 * 3 * 3)
unsigned char USBpanelData[MAX_PANEL_DATASIZE];
#ifdef PANEL32X32
unsigned char tempPanel[PANELSIZE];
#endif

#define HOSTuart UART2
#define HOSTbits U2STAbits
#define HOST_VECTOR _UART_2_VECTOR 
unsigned char HOSTTxBuffer[MAXHOSTBUFFER];
unsigned char HOSTRxBuffer[MAXHOSTBUFFER];
unsigned short HOSTTxLength;
unsigned short HOSTRxLength;
unsigned char USBstatus = 0;

BOOL stringPrinted;
volatile BOOL buttonPressed;
volatile BYTE buttonCount;


#define MAXPOTS 4

unsigned char command = 0;
unsigned char arrPots[MAXPOTS];
unsigned char displayMode = TRUE;

// extern unsigned long colorWheel[MAXCOLOR];
unsigned short matrixOutData[(NUMWRITES*COLORDEPTH*MAXLINE)];
unsigned short line = 0;
unsigned short dataOffset = 0;
unsigned char colorPlane = 0;
unsigned short latchHigh[1] = {OE_LOW_LATCH_HIGH};

unsigned long Delaymultiplier = TIMER_ROLLOVER;

unsigned long redAdjust, greenAdjust, blueAdjust, brightAdjust;

unsigned char gammaTable[256] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5,
    5, 6, 6, 6, 6, 7, 7, 7, 8, 8, 8, 9, 9, 9, 10, 10, 10, 11, 11, 11, 12, 12, 13, 13, 14,
    14, 14, 15, 15, 16, 16, 17, 17, 18, 18, 19, 19, 20, 21, 21, 22, 22, 23, 23, 24, 25,
    25, 26, 27, 27, 28, 29, 29, 30, 31, 31, 32, 33, 34, 34, 35, 36, 37, 37, 38, 39, 40,
    41, 42, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 52, 53, 54, 55, 56, 57, 59, 60,
    61, 62, 63, 64, 65, 66, 67, 68, 69, 71, 72, 73, 74, 75, 77, 78, 79, 80, 82, 83, 84,
    85, 87, 88, 89, 91, 92, 93, 95, 96, 98, 99, 100, 102, 103, 105, 106, 108, 109, 111,
    112, 114, 115, 117, 119, 120, 122, 123, 125, 127, 128, 130, 132, 133, 135, 137, 138,
    140, 142, 144, 145, 147, 149, 151, 153, 155, 156, 158, 160, 162, 164, 166, 168, 170,
    172, 174, 176, 178, 180, 182, 184, 186, 188, 190, 192, 194, 197, 199, 201, 203, 205,
    207, 210, 212, 214, 216, 219, 221, 223, 226, 228, 230, 233, 235, 237, 240, 242, 245,
    247, 250, 252, 255};

unsigned long RGBtable[256] = {
    0xff0000, 0xfc0300, 0xf90600, 0xf60900,
    0xf30c00, 0xf00f00, 0xed1200, 0xea1500,
    0xe71800, 0xe41b00, 0xe11e00, 0xde2100,
    0xdb2400, 0xd82700, 0xd52a00, 0xd22d00,
    0xcf3000, 0xcc3300, 0xc93600, 0xc63900,
    0xc33c00, 0xc03f00, 0xbd4200, 0xba4500,
    0xb74800, 0xb44b00, 0xb14e00, 0xae5100,
    0xab5400, 0xa85700, 0xa55a00, 0xa25d00,
    0x9f6000, 0x9c6300, 0x996600, 0x966900,
    0x936c00, 0x906f00, 0x8d7200, 0x8a7500,
    0x877800, 0x847b00, 0x817e00, 0x7e8100,
    0x7b8400, 0x788700, 0x758a00, 0x728d00,
    0x6f9000, 0x6c9300, 0x699600, 0x669900,
    0x639c00, 0x609f00, 0x5da200, 0x5aa500,
    0x57a800, 0x54ab00, 0x51ae00, 0x4eb100,
    0x4bb400, 0x48b700, 0x45ba00, 0x42bd00,
    0x3fc000, 0x3cc300, 0x39c600, 0x36c900,
    0x33cc00, 0x30cf00, 0x2dd200, 0x2ad500,
    0x27d800, 0x24db00, 0x21de00, 0x1ee100,
    0x1be400, 0x18e700, 0x15ea00, 0x12ed00,
    0x0ff000, 0x0cf300, 0x09f600, 0x06f900,
    0x03fc00,
    0x00ff00, 0x00fc03, 0x00f906, 0x00f609,
    0x00f30c, 0x00f00f, 0x00ed12, 0x00ea15,
    0x00e718, 0x00e41b, 0x00e11e, 0x00de21,
    0x00db24, 0x00d827, 0x00d52a, 0x00d22d,
    0x00cf30, 0x00cc33, 0x00c936, 0x00c639,
    0x00c33c, 0x00c03f, 0x00bd42, 0x00ba45,
    0x00b748, 0x00b44b, 0x00b14e, 0x00ae51,
    0x00ab54, 0x00a857, 0x00a55a, 0x00a25d,
    0x009f60, 0x009c63, 0x009966, 0x009669,
    0x00936c, 0x00906f, 0x008d72, 0x008a75,
    0x008778, 0x00847b, 0x00817e, 0x007e81,
    0x007b84, 0x007887, 0x00758a, 0x00728d,
    0x006f90, 0x006c93, 0x006996, 0x006699,
    0x00639c, 0x00609f, 0x005da2, 0x005aa5,
    0x0057a8, 0x0054ab, 0x0051ae, 0x004eb1,
    0x004bb4, 0x0048b7, 0x0045ba, 0x0042bd,
    0x003fc0, 0x003cc3, 0x0039c6, 0x0036c9,
    0x0033cc, 0x0030cf, 0x002dd2, 0x002ad5,
    0x0027d8, 0x0024db, 0x0021de, 0x001ee1,
    0x001be4, 0x0018e7, 0x0015ea, 0x0012ed,
    0x000ff0, 0x000cf3, 0x0009f6, 0x0006f9,
    0x0003fc,
    0x0000ff, 0x0300fc, 0x0600f9, 0x0900f6,
    0x0c00f3, 0x0f00f0, 0x1200ed, 0x1500ea,
    0x1800e7, 0x1b00e4, 0x1e00e1, 0x2100de,
    0x2400db, 0x2700d8, 0x2a00d5, 0x2d00d2,
    0x3000cf, 0x3300cc, 0x3600c9, 0x3900c6,
    0x3c00c3, 0x3f00c0, 0x4200bd, 0x4500ba,
    0x4800b7, 0x4b00b4, 0x4e00b1, 0x5100ae,
    0x5400ab, 0x5700a8, 0x5a00a5, 0x5d00a2,
    0x60009f, 0x63009c, 0x660099, 0x690096,
    0x6c0093, 0x6f0090, 0x72008d, 0x75008a,
    0x780087, 0x7b0084, 0x7e0081, 0x81007e,
    0x84007b, 0x870078, 0x8a0075, 0x8d0072,
    0x90006f, 0x93006c, 0x960069, 0x990066,
    0x9c0063, 0x9f0060, 0xa2005d, 0xa5005a,
    0xa80057, 0xab0054, 0xae0051, 0xb1004e,
    0xb4004b, 0xb70048, 0xba0045, 0xbd0042,
    0xc0003f, 0xc3003c, 0xc60039, 0xc90036,
    0xcc0033, 0xcf0030, 0xd2002d, 0xd5002a,
    0xd80027, 0xdb0024, 0xde0021, 0xe1001e,
    0xe4001b, 0xe70018, 0xea0015, 0xed0012,
    0xf0000f, 0xf3000c, 0xf60009, 0xf90006,
    0xfc0003, 0xff0000
};

unsigned long matrix[MAXROW][MAXCOL];
unsigned long panelData[NUMPANELS][PANELROWS][PANELCOLS];
unsigned char panelUpdateBytes[PANELSIZE];

/** P R I V A T E  P R O T O T Y P E S ***************************************/
static void InitializeSystem(void);
void ProcessIO(void);
void USBDeviceTasks(void);
void YourHighPriorityISRCode();
void YourLowPriorityISRCode();
void USBCBSendResume(void);
void BlinkUSBStatus(void);
void UserInit(void);

unsigned short decodePacket(unsigned char *ptrPacket, unsigned char *ptrBitmap);
unsigned short processUSBpanelData(unsigned char *ptrReceivedData, unsigned short packetLength);
void loadMatrix(void);
void updateOutputBuffer(void);
void ConfigAd(void);
unsigned long getRGBColor(unsigned short crossfade, unsigned short potVal);
unsigned char updateOutputPanel(unsigned char panelNumber);
unsigned char updateOutputPanelPtr(unsigned char panelNumber, unsigned char *ptrPanelData);
unsigned char uncompressData(unsigned char *ptrCompressedData, unsigned char *ptrPanelData);

void resetTimer(void) {
    line = 0;
    dataOffset = 0;
    colorPlane = 0;
    Delaymultiplier = TIMER_ROLLOVER;
    mT23ClearIntFlag();
    OpenTimer23(T23_ON | T23_SOURCE_INT | T23_PS_1_1, TIMER_ROLLOVER);
}

void initMatrix(unsigned long color) {
    unsigned short i, j;

    for (i = 0; i < (MAXROW); i++) {
        for (j = 0; j < MAXCOL; j++) {
            // matrixPrev[i][j] = matrix[i][j] = color;
            // matrixBlackout[MAXROW][MAXCOL] = 0;
            matrix[i][j] = color;
        }
    }
}

void initPanels(unsigned long color) {
    unsigned short panel, row, col;

    for (panel = 0; panel < NUMPANELS; panel++)
        for (row = 0; row < PANELROWS; row++)
            for (col = 0; col < PANELCOLS; col++)
                panelData[panel][row][col] = color;
}

unsigned long colorWheel[MAXCOLOR] = {MAGENTA, PURPLE, CYAN, LIME, YELLOW, ORANGE, RED, GREEN, BLUE, PINK, LAVENDER, TURQUOISE, GRAY, DARKGRAY, WHITE};


#define BARWIDTH (MAXCOL/MAXCOLOR)

void panelColorBars(void) {
    unsigned short i, row, col;
    unsigned char colorIndex = 0, k = BARWIDTH;
    unsigned long color;

    i = 0;
    do {        
        for (col = 0; col < PANELCOLS; col++) {
            for (row = 0; row < PANELROWS; row++) {
                if (colorIndex < MAXCOLOR) {
                    color = colorWheel[colorIndex];
                    panelData[i][row][col] = color;
                    panelData[i + 1][row][col] = color;
                }
            }
            if (k)k--;
            if (!k) {
                colorIndex++;
                k = BARWIDTH;
            }
        }
        i = i + 2;
    } while (i < (PANELS_ACROSS * 2));
    for (i = 0; i < NUMPANELS; i++) updateOutputPanel(i);
}

void adjustColorBars(void) {
#define BARWIDTH (MAXCOL/MAXCOLOR)
    unsigned char colorIndex = 0, k = BARWIDTH;
    unsigned short i, j;
    unsigned long color, blue, green, red;
    unsigned char potVal;
    unsigned char intensity;

    for (j = 0; j < MAXCOL; j++) {
        for (i = 0; i < (MAXROW); i++) {
            if (colorIndex < MAXCOLOR) {
                color = colorWheel[colorIndex];
                potVal = arrPots[3];
                if (potVal > 255) potVal = 255;

                intensity = gammaTable[potVal];

                blue = (long) intensity;
                green = (long) intensity;
                red = (long) intensity;

                color = ((blue << 16) | (green << 8) | red);

                matrix[i][j] = color;
            }

        }

        if (k)k--;
        if (!k) {
            colorIndex++;
            k = BARWIDTH;
        }
    }
    updateOutputBuffer();
}

unsigned long getRGBColor(unsigned short crossfade, unsigned short potVal) {
    short r, g, b, intensity;
    unsigned long color, red, blue, green;

    if (potVal > 255) potVal = 255;

    intensity = gammaTable[potVal];

    if (crossfade == 0 || crossfade == 255)
        return (0x0000FF);

    // GREEN
    if (crossfade < 171) {
        g = ((85 - abs(85 - crossfade)) * intensity) / 85;
    } else g = 0;

    // BLUE
    if (crossfade > 85 && crossfade <= 255) {
        b = ((85 - abs(171 - crossfade)) * intensity) / 85;
    } else b = 0;

    // RED
    if (crossfade <= 85) {
        r = ((85 - crossfade) * intensity) / 85;
    } else if (crossfade > 171) {
        r = ((crossfade - 171) * intensity) / 85;
    } else r = 0;

    if (r > 255) r = 255;
    if (g > 255) g = 255;
    if (b > 255) b = 255;

    blue = (b << 16) & 0xFF0000;
    green = (g << 8) & 0x00FF00;
    red = r & 0x0000FF;

    color = blue + green + red;

    return (color);
}

void adjustColor(void) {
    short crossfade, i, j;
    unsigned long color;

    crossfade = arrPots[3];
    if (crossfade > 255) crossfade = 255;
    color = getRGBColor(crossfade, 255);

    for (j = 0; j < MAXCOL; j++) {
        for (i = 0; i < (MAXROW); i++) {
            matrix[i][j] = color;
        }
    }
    updateOutputBuffer();
}

int main(void) {
    unsigned long i;
    for (i = 0; i < (NUMWRITES * COLORDEPTH * MAXLINE); i++) matrixOutData[i] = (unsigned short) WHITE;
    panelColorBars();
    InitializeSystem();

    // TESTupdateOutputPanel(0);
    // ColorBars();

    DelayMs(100);
    printf("\r\rSTARTING LED CONTROLLER CDC VERSION");

    while (1) {
        if (HOSTRxLength) {
            printf("\rRECEIVED: %s", HOSTRxBuffer);
            HOSTRxLength = 0;
        }
#if defined(USB_INTERRUPT)
        if (USB_BUS_SENSE && (USBGetDeviceState() == DETACHED_STATE)) {
            USBDeviceAttach();
        }
#endif

#if defined(USB_POLLING)
        // Check bus status and service USB interrupts.
        USBDeviceTasks(); // Interrupt or polling method.  
#endif
        // Application-specific tasks.
        // Application related code may be added here, or in the ProcessIO() function.
        ProcessIO();
    }//end while
}//end main

/********************************************************************
 * Function:        static void InitializeSystem(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        InitializeSystem is a centralize initialization
 *                  routine. All required USB initialization routines
 *                  are called from here.
 *
 *                  User application initialization routine should
 *                  also be called from here.                  
 *
 * Note:            None
 *******************************************************************/
static void InitializeSystem(void) {
    SYSTEMConfigPerformance(80000000); // was 60000000

#if defined(USE_USB_BUS_SENSE_IO)
    tris_usb_bus_sense = INPUT_PIN; // See HardwareProfile.h
#endif
#if defined(USE_SELF_POWER_SENSE_IO)
    tris_self_power = INPUT_PIN; // See HardwareProfile.h
#endif

    UserInit();

    USBDeviceInit(); //usb_device.c.  Initializes USB module SFRs and firmware
    //variables to known states.
}//end InitializeSystem

void UserInit(void) {
    PORTSetPinsDigitalOut(IOPORT_E, BIT_0 | BIT_1 | BIT_2 | BIT_3);

    // Turn off JTAG so we get the pins back
    mJTAGPortEnable(false);

#ifdef REV3BOARD 
    PORTSetPinsDigitalOut(IOPORT_A, BIT_0);
#else
    PORTSetPinsDigitalOut(IOPORT_A, BIT_6 | BIT_7); // `B
#endif

    PORTD = 0x0000; // Set matrix output enable high. All other outputs low.
#ifdef REV3BOARD
    OEpin = 1;
#else    
    OEpin = 0;
#endif    

#ifdef PANEL32X32    
    PORTSetPinsDigitalOut(IOPORT_D, BIT_0 | BIT_1 | BIT_2 | BIT_3); // For 32x32 matrix
#else
    PORTSetPinsDigitalOut(IOPORT_D, BIT_0 | BIT_1 | BIT_2); // For 16x32 matrix
#endif    



    // Set up Timer 2/3 as a single 32 bit timer with interrupt priority of 2
    // Use internal clock, 1:1 prescale
    // If Postscale = 1600 interrupts occur about every 33 uS
    // This yields a refresh rate of about 59 hz for entire display
    // The flicker seems pretty minimal at this rate
    // 1 / 33 us x 8 x 2^6 = 59
    // 8 lines x 32 columns x 6 panels x 6 bit resolution = 9216 writes to PORT D for each refresh!       
    ConfigIntTimer23(T2_INT_ON | T2_INT_PRIOR_2);
    OpenTimer23(T23_ON | T23_SOURCE_INT | T23_PS_1_1, TIMER_ROLLOVER);


    //ConfigIntTimer4(T4_INT_ON | T4_INT_PRIOR_2);
    //OpenTimer4(T4_ON | T4_SOURCE_INT | T4_PS_1_1, 1000);

    PORTSetPinsDigitalOut(IOPORT_B, BIT_0 | BIT_1 | BIT_2 | BIT_15);
    PORTSetBits(IOPORT_B, BIT_0 | BIT_1 | BIT_2);
    XBEE_SLEEP_OFF;


    //PORTSetPinsDigitalIn(IOPORT_B, BIT_4 | BIT_5);
    //mCNOpen(CN_ON, CN6_ENABLE | CN7_ENABLE, CN6_PULLUP_ENABLE | CN7_PULLUP_ENABLE);
    //ConfigIntCN(CHANGE_INT_ON | CHANGE_INT_PRI_2);

    // Set up Port C outputs:
#ifdef REV3BOARD
    // RtccShutdown();
    PORTSetPinsDigitalOut(IOPORT_C, BIT_3 | BIT_13 | BIT_14 | BIT_4);
#else
    PORTSetPinsDigitalOut(IOPORT_C, BIT_3 | BIT_4);
#endif
    // Set up Port G outputs:
    PORTSetPinsDigitalOut(IOPORT_G, BIT_0);
    RS485_CTRL = 0;

#ifdef REV3BOARD
    LATC = OE_HIGH_LATCH_LOW;
#else    
    PORTA = OE_HIGH_LATCH_LOW;
#endif    

#define PMP_CONTROL	(PMP_ON | PMP_MUX_OFF | PMP_READ_WRITE_EN | PMP_WRITE_POL_HI) 

#define PMP_MODE        (PMP_IRQ_READ_WRITE | PMP_MODE_MASTER1 | PMP_DATA_BUS_16 | PMP_WAIT_BEG_1 | PMP_WAIT_MID_0 | PMP_WAIT_END_1)
#define PMP_PORT_PINS	PMP_PEN_ALL
#define PMP_INTERRUPT	PMP_INT_OFF

    // setup the PMP
    mPMPOpen(PMP_CONTROL, PMP_MODE, PMP_PORT_PINS, PMP_INTERRUPT);


    // CONFIGURE DMA CHANNEL 1 - SEND DATA
    DmaChnOpen(DMA_CHANNEL1, 3, DMA_OPEN_DEFAULT);

    // Set the transfer event control: what event is to start the DMA transfer
    DmaChnSetEventControl(DMA_CHANNEL1, DMA_EV_START_IRQ(_PMP_IRQ));

    // Set up transfer destination: PMDIN Parallel Port Data Input Register
    DmaChnSetTxfer(DMA_CHANNEL1, &matrixOutData[0], (void*) &PMDIN, NUMWRITES, 2, 2);
    //  DmaChnSetTxfer(DMA_CHANNEL1, &matrixOutData[0], (void*)&PMDIN, sizeof(matrixOutData), 2, NUMWRITES*2);

    // Enable the transfer done event flag:
    DmaChnSetEvEnableFlags(DMA_CHANNEL1, DMA_EV_BLOCK_DONE);


    // Once we configured the DMA channel we can enable it
    DmaChnEnable(DMA_CHANNEL1);

    DmaChnForceTxfer(DMA_CHANNEL1);

    // CONFIGURE DMA CHANNEL 2 - LATCH DATA
    DmaChnOpen(DMA_CHANNEL2, 2, DMA_OPEN_DEFAULT);

    // Set the transfer event control: what event is to start the DMA transfer
    DmaChnSetEventControl(DMA_CHANNEL2, DMA_EV_START_IRQ(_DMA1_IRQ));

    // Set up transfer: source & destination, source & destination size, number of bytes trasnferred per event
#ifdef REV3BOARD
    DmaChnSetTxfer(DMA_CHANNEL2, latchHigh, (void*) &LATC, sizeof (latchHigh), 2, 2);
#else
    DmaChnSetTxfer(DMA_CHANNEL2, latchHigh, (void*) &LATA, sizeof (latchHigh), 2, 2);
#endif
    // Enable the transfer done event flag:
    DmaChnSetEvEnableFlags(DMA_CHANNEL2, DMA_EV_CELL_DONE);

    // Once we configured the DMA channel we can enable it
    DmaChnEnable(DMA_CHANNEL2);

    // Set up main UART
    UARTConfigure(HOSTuart, UART_ENABLE_HIGH_SPEED | UART_ENABLE_PINS_TX_RX_ONLY);
    UARTSetFifoMode(HOSTuart, UART_INTERRUPT_ON_RX_NOT_EMPTY); //  | UART_INTERRUPT_ON_TX_DONE  
    UARTSetLineControl(HOSTuart, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
#define HOSTuart UART2
#define SYS_FREQ 80000000
    UARTSetDataRate(HOSTuart, SYS_FREQ, 57600); // `A
    UARTEnable(HOSTuart, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));

    // Configure UART #2 Interrupts
    INTEnable(INT_U2TX, INT_DISABLED);
    INTEnable(INT_SOURCE_UART_RX(HOSTuart), INT_ENABLED);
    INTSetVectorPriority(INT_VECTOR_UART(HOSTuart), INT_PRIORITY_LEVEL_2);
    // INTSetVectorSubPriority(INT_VECTOR_UART(HOSTuart), INT_SUB_PRIORITY_LEVEL_0);        

    // Turn on the interrupts
    INTEnableSystemMultiVectoredInt();

}//end UserInit

/********************************************************************
 * Function:        void BlinkUSBStatus(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        BlinkUSBStatus turns on and off LEDs 
 *                  corresponding to the USB device state.
 *
 * Note:            mLED macros can be found in HardwareProfile.h
 *                  USBDeviceState is declared and updated in
 *                  usb_device.c.
 *******************************************************************/
void BlinkUSBStatus(void) {
    static WORD led_count = 0;

    if (led_count == 0)led_count = 10000U;
    led_count--;

#define mLED_Both_Off()         {mLED_1_Off();mLED_2_Off();}
#define mLED_Both_On()          {mLED_1_On();mLED_2_On();}
#define mLED_Only_1_On()        {mLED_1_On();mLED_2_Off();}
#define mLED_Only_2_On()        {mLED_1_Off();mLED_2_On();}

    if (USBSuspendControl == 1) {
        if (led_count == 0) {
            mLED_1_Toggle();
            if (mGetLED_1()) {
                mLED_2_On();
            } else {
                mLED_2_Off();
            }
        }//end if
    } else {
        if (USBDeviceState == DETACHED_STATE) {
            mLED_Both_Off();
        } else if (USBDeviceState == ATTACHED_STATE) {
            mLED_Both_On();
        } else if (USBDeviceState == POWERED_STATE) {
            mLED_Only_1_On();
        } else if (USBDeviceState == DEFAULT_STATE) {
            mLED_Only_2_On();
        } else if (USBDeviceState == ADDRESS_STATE) {
            if (led_count == 0) {
                mLED_1_Toggle();
                mLED_2_Off();
            }//end if
        } else if (USBDeviceState == CONFIGURED_STATE) {
            if (led_count == 0) {
                mLED_1_Toggle();
                if (mGetLED_1()) {
                    mLED_2_Off();
                } else {
                    mLED_2_On();
                }
            }//end if
        }//end if(...)
    }//end if(UCONbits.SUSPND...)

}//end BlinkUSBStatus

void USBCBSuspend(void) {
}

void USBCBWakeFromSuspend(void) {
}

void USBCB_SOF_Handler(void) {
    // No need to clear UIRbits.SOFIF to 0 here.
    // Callback caller is already doing that.

    //This is reverse logic since the pushbutton is active low
    if (buttonPressed == sw2) {
        if (buttonCount != 0) {
            buttonCount--;
        } else {
            //This is reverse logic since the pushbutton is active low
            buttonPressed = !sw2;

            //Wait 100ms before the next press can be generated
            buttonCount = 100;
        }
    } else {
        if (buttonCount != 0) {
            buttonCount--;
        }
    }
}

void USBCBErrorHandler(void) {
}

void USBCBCheckOtherReq(void) {
    USBCheckCDCRequest();
}//end

void USBCBStdSetDscHandler(void) {
    // Must claim session ownership if supporting this request
}//end

void USBCBInitEP(void) {
    //Enable the CDC data endpoints
    CDCInitEP();
}

void USBCBSendResume(void) {
    static WORD delay_count;
    if (USBGetRemoteWakeupStatus() == TRUE) {
        //Verify that the USB bus is in fact suspended, before we send
        //remote wakeup signalling.
        if (USBIsBusSuspended() == TRUE) {
            USBMaskInterrupts();

            //Clock switch to settings consistent with normal USB operation.
            USBCBWakeFromSuspend();
            USBSuspendControl = 0;
            USBBusIsSuspended = FALSE; //So we don't execute this code again, 
            //until a new suspend condition is detected.

            delay_count = 3600U;
            do {
                delay_count--;
            } while (delay_count);

            //Now drive the resume K-state signalling onto the USB bus.
            USBResumeControl = 1; // Start RESUME signaling
            delay_count = 1800U; // Set RESUME line for 1-13 ms
            do {
                delay_count--;
            } while (delay_count);
            USBResumeControl = 0; //Finished driving resume signalling

            USBUnmaskInterrupts();
        }
    }
}

#if defined(ENABLE_EP0_DATA_RECEIVED_CALLBACK)

void USBCBEP0DataReceived(void) {
}
#endif

BOOL USER_USB_CALLBACK_EVENT_HANDLER(int event, void *pdata, WORD size) {
    switch (event) {
        case EVENT_TRANSFER:
            //Add application specific callback task or callback function here if desired.
            break;
        case EVENT_SOF:
            USBCB_SOF_Handler();
            break;
        case EVENT_SUSPEND:
            USBCBSuspend();
            break;
        case EVENT_RESUME:
            USBCBWakeFromSuspend();
            break;
        case EVENT_CONFIGURED:
            USBCBInitEP();
            break;
        case EVENT_SET_DESCRIPTOR:
            USBCBStdSetDscHandler();
            break;
        case EVENT_EP0_REQUEST:
            USBCBCheckOtherReq();
            break;
        case EVENT_BUS_ERROR:
            USBCBErrorHandler();
            break;
        case EVENT_TRANSFER_TERMINATED:
            //Add application specific callback task or callback function here if desired.
            //The EVENT_TRANSFER_TERMINATED event occurs when the host performs a CLEAR
            //FEATURE (endpoint halt) request on an application endpoint which was 
            //previously armed (UOWN was = 1).  Here would be a good place to:
            //1.  Determine which endpoint the transaction that just got terminated was 
            //      on, by checking the handle value in the *pdata.
            //2.  Re-arm the endpoint if desired (typically would be the case for OUT 
            //      endpoints).
            break;
        default:
            break;
    }
    return TRUE;
}



// HOST UART interrupt handler it is set at priority level 2

void __ISR(HOST_VECTOR, ipl2) IntHostUartHandler(void) {
    static unsigned short HOSTRxIndex = 0;
    static unsigned char TxIndex = 0;
    unsigned char ch;

    if (HOSTbits.OERR || HOSTbits.FERR) {
        if (UARTReceivedDataIsAvailable(HOSTuart))
            ch = UARTGetDataByte(HOSTuart);
        HOSTbits.OERR = 0;
        HOSTRxIndex = 0;
    } else if (INTGetFlag(INT_SOURCE_UART_RX(HOSTuart))) {
        INTClearFlag(INT_SOURCE_UART_RX(HOSTuart));
        if (UARTReceivedDataIsAvailable(HOSTuart)) {
            ch = UARTGetDataByte(HOSTuart);
            if (ch != 0 && ch != '\n') {
                if (HOSTRxIndex < MAXHOSTBUFFER - 2)
                    HOSTRxBuffer[HOSTRxIndex++] = ch;
                if (ch == '\r') {
                    HOSTRxBuffer[HOSTRxIndex] = '\0';
                    HOSTRxLength = HOSTRxIndex;
                    HOSTRxIndex = 0;
                }
            }
            // UARTtimeout=UART_TIMEOUT;
        }
    }
    if (INTGetFlag(INT_SOURCE_UART_TX(HOSTuart))) {
        INTClearFlag(INT_SOURCE_UART_TX(HOSTuart));
        if (HOSTTxLength) {
            if (TxIndex < MAXHOSTBUFFER) {
                ch = HOSTTxBuffer[TxIndex++];
                if (TxIndex <= HOSTTxLength) {
                    while (!UARTTransmitterIsReady(HOSTuart));
                    UARTSendDataByte(HOSTuart, ch);
                } else {
                    while (!UARTTransmitterIsReady(HOSTuart));
                    HOSTTxLength = false;
                    TxIndex = 0;
                }
            } else {
                TxIndex = 0;
                HOSTTxLength = false;
                INTEnable(INT_SOURCE_UART_TX(HOSTuart), INT_DISABLED);
            }
        } else INTEnable(INT_SOURCE_UART_TX(HOSTuart), INT_DISABLED);
    }
}

unsigned long getLongInteger(unsigned char b0, unsigned char b1, unsigned char b2, unsigned char b3) {

    union {
        unsigned char byte[4];
        unsigned long lngInteger;
    } convert;

    convert.byte[0] = b0;
    convert.byte[1] = b1;
    convert.byte[2] = b2;
    convert.byte[3] = b3;

    return (convert.lngInteger);
}

unsigned short getShort(unsigned char b0, unsigned char b1) {

    union {
        unsigned char byte[2];
        unsigned short integer;
    } convert;

    convert.byte[0] = b0;
    convert.byte[1] = b1;
    return (convert.integer);
}

unsigned char loadPixels(unsigned char *bufferPtr) {
    unsigned short row = 0;
    unsigned short col = 0;
    unsigned char colorByte, mask, red, blue, green;
    short i = 0;
    i = 0;
    for (row = 0; row < MAXROW; row++) {
        for (col = 0; col < MAXCOL; col++) {
            colorByte = bufferPtr[i++];
            blue = gammaTable[colorByte];
            colorByte = bufferPtr[i++];
            green = gammaTable[colorByte];
            colorByte = bufferPtr[i++];
            red = gammaTable[colorByte];
            // matrix0[row][col] = getLongInteger(red, green, blue, 0x00);
        }
    }
    return (TRUE);
}

void loadMatrix(void) {
    unsigned short row, col;
    unsigned long pixel, lngRed, lngGreen, lngBlue;
    unsigned char Red, Blue, Green;

    for (row = 0; row < MAXROW; row++) {
        for (col = 0; col < MAXCOL; col++) {
            // pixel = matrix0[row][col];

            lngRed = (pixel & (0x000000FF));
            lngGreen = (pixel & (0x0000FF00)) >> 8;
            lngBlue = (pixel & (0x00FF0000)) >> 16;

            Red = (unsigned char) ((lngRed * BALANCE) >> 16);
            Green = (unsigned char) ((lngGreen * BALANCE) >> 16);
            Blue = (unsigned char) ((lngBlue * BALANCE) >> 16);

            lngRed = (unsigned long) gammaTable[Red];
            lngGreen = (unsigned long) gammaTable[Green];
            lngBlue = (unsigned long) gammaTable[Blue];

            lngGreen = lngGreen << 8;
            lngBlue = lngBlue << 16;

            pixel = lngRed | lngGreen | lngBlue;
            matrix[row][col] = pixel;
        }
    }
}

unsigned short decodePacket(unsigned char *ptrPacket, unsigned char *ptrBitmap) {
    unsigned short i, j;
    static unsigned char escapeFlag = FALSE;
    unsigned char ch;

    j = 0;
    for (i = 0; i < USBPACKETSIZE; i++) {
        ch = ptrPacket[i];
        if (USBstatus == USB_STANDBY) {
            if (ch == STX) {
                USBstatus = USB_INCOMING;
                escapeFlag = FALSE;
            }
        } else {
            if (!escapeFlag) {
                if (ch == STX) {
                    USBstatus = USB_PACKETERROR;
                    escapeFlag = FALSE;
                    return (0);
                } else if (ch == ETX) {
                    USBstatus = USB_DONE;
                    return (j);
                } else if (ch == DLE)
                    escapeFlag = TRUE;
                else if (j < MAXBITMAP)
                    ptrBitmap[j++] = ch;
                else {
                    USBstatus = USB_PACKETERROR;
                    return (0);
                }
            } else {
                escapeFlag = FALSE;
                if (j < MAXBITMAP)
                    ptrBitmap[j++] = ch;
                else {
                    USBstatus = USB_PACKETERROR;
                    j = 0;
                }
            }
        }
    }

    return (j);
}

void ConfigAd(void) {

    mPORTBSetPinsAnalogIn(BIT_0);
    mPORTBSetPinsAnalogIn(BIT_1);
    mPORTBSetPinsAnalogIn(BIT_2);
    mPORTBSetPinsAnalogIn(BIT_3);

    // ---- configure and enable the ADC ----

    // ensure the ADC is off before setting the configuration
    CloseADC10();

    // define setup parameters for OpenADC10
    //                 Turn module on | ouput in integer | trigger mode auto | enable autosample
#define PARAM1  ADC_MODULE_ON | ADC_FORMAT_INTG | ADC_CLK_AUTO | ADC_AUTO_SAMPLING_ON

    // ADC ref external    | disable offset test    | enable scan mode | perform  samples | use dual buffers | use only mux A
#define PARAM2  ADC_VREF_AVDD_AVSS | ADC_OFFSET_CAL_DISABLE | ADC_SCAN_ON | ADC_SAMPLES_PER_INT_4 | ADC_ALT_BUF_ON | ADC_ALT_INPUT_OFF

    //                   use ADC internal clock | set sample time
#define PARAM3  ADC_CONV_CLK_INTERNAL_RC | ADC_SAMPLE_TIME_31

    //  set inputs to analog
#define PARAM4    ENABLE_AN0_ANA | ENABLE_AN1_ANA| ENABLE_AN2_ANA | ENABLE_AN3_ANA

    // Only scan AN0, AN1, AN2, AN3 for now
#define PARAM5   SKIP_SCAN_AN4 |SKIP_SCAN_AN5 |SKIP_SCAN_AN6 |SKIP_SCAN_AN7 |\
                    SKIP_SCAN_AN8 |SKIP_SCAN_AN9 |SKIP_SCAN_AN10 |\
                      SKIP_SCAN_AN11 | SKIP_SCAN_AN12 |SKIP_SCAN_AN13 |SKIP_SCAN_AN14 |SKIP_SCAN_AN15

    // set negative reference to Vref for Mux A
    SetChanADC10(ADC_CH0_NEG_SAMPLEA_NVREF);

    // open the ADC
    OpenADC10(PARAM1, PARAM2, PARAM3, PARAM4, PARAM5);

    ConfigIntADC10(ADC_INT_PRI_2 | ADC_INT_SUB_PRI_2 | ADC_INT_ON);

    // clear the interrupt flag
    mAD1ClearIntFlag();

    // Enable the ADC
    EnableADC10();

}

void __ISR(_ADC_VECTOR, ipl6) AdcHandler(void) {
    unsigned short offSet;
    unsigned char i;

    mAD1IntEnable(INT_DISABLED);
    mAD1ClearIntFlag();

    // Determine which buffer is idle and create an offset
    offSet = 8 * ((~ReadActiveBufferADC10() & 0x01));

    for (i = 0; i < MAXPOTS; i++)
        arrPots[i] = (unsigned char) (ReadADC10(offSet + i) / 4); // read the result of channel 0 conversion from the idle buffer

}

void __ISR(_TIMER_23_VECTOR, ipl2) Timer23Handler(void) {
    static unsigned char line = 0;
    static unsigned char colorPlane = 0;
    static unsigned long Delaymultiplier = TIMER_ROLLOVER;
    static unsigned short dataOffset = 0;
    static unsigned char testFlag = FALSE;

    mT23ClearIntFlag(); // Clear interrupt flag    

    WritePeriod23(Delaymultiplier);
    if (colorPlane == 0 || colorPlane == 2 || colorPlane == 4) TEST_OUT = 1;
    else TEST_OUT = 0;
    //#ifdef REV2BOARD    
    PORTD = line;
    //#else
    //    PORTD = ~line;
    //#endif    

#ifdef REV3BOARD
    LATC = OE_HIGH_LATCH_LOW;
#else
    PORTA = OE_HIGH_LATCH_LOW;
#endif

    DmaChnSetTxfer(DMA_CHANNEL1, &matrixOutData[dataOffset], (void*) &PMDIN, NUMWRITES * 2, 2, 2);

    DmaChnEnable(DMA_CHANNEL1);
    DmaChnEnable(DMA_CHANNEL2);
    // DmaChnDisable(DMA_CHANNEL2);
    dataOffset = dataOffset + NUMWRITES;
    Delaymultiplier = Delaymultiplier << 1;
    colorPlane++;
    if (colorPlane >= COLORDEPTH) {
        colorPlane = 0;
        Delaymultiplier = TIMER_ROLLOVER;
        line++;
        if (line >= MAXLINE) {
            line = 0;
            dataOffset = 0;
        }
    }
}

/*
unsigned short processUSBpanelData(unsigned char *ptrUSBdata, unsigned short dataLength) {
    unsigned char lowByte, highByte, command, subCommand;
    unsigned short lengthCheck, packetCheck;

    packetCheck = dataLength;

    command = ptrUSBdata[0];
    subCommand = ptrUSBdata[1];
    lowByte = ptrUSBdata[2];
    highByte = ptrUSBdata[3];

    lengthCheck = getShort(lowByte, highByte);
    if (lengthCheck != (dataLength - 4))
        return (USB_LENGTH_ERROR);

    if (subCommand > 3) return (USB_COMMAND_ERROR);

    uncompressData(&ptrUSBdata[4], panelUpdateBytes);
    updateOutputPanelPtr(subCommand, panelUpdateBytes);
    return (USB_PACKET_OK);
}
 */
// NO COMPRESSION VERSION

unsigned short processUSBpanelData(unsigned char *ptrUSBdata, unsigned short dataLength) {
    unsigned char lowByte, highByte, command, subCommand;
    unsigned short lengthCheck, packetCheck;
    unsigned short i, j;

    packetCheck = dataLength;

    command = ptrUSBdata[0];
    subCommand = ptrUSBdata[1];
    lowByte = ptrUSBdata[2];
    highByte = ptrUSBdata[3];

    lengthCheck = getShort(lowByte, highByte);
    if (lengthCheck != (dataLength - 4))
        return (USB_LENGTH_ERROR);

    if (subCommand >= NUMPANELS) return (USB_COMMAND_ERROR);
    
#ifdef PANEL32X32    
    if (!command){
        j = 4;
        for (i=0; i<(PANELSIZE/2); i++) tempPanel[i] = ptrUSBdata[j++];
    }
    else {
        j = 4;
        for (i=(PANELSIZE/2); i<PANELSIZE; i++) tempPanel[i] = ptrUSBdata[j++];        
        updateOutputPanelPtr(subCommand, &tempPanel[0]);
    }
#else   
    updateOutputPanelPtr(subCommand, &ptrUSBdata[4]);
#endif 
    return (USB_PACKET_OK);
}

void updateOutputBuffer(void) {
    unsigned short line;
    unsigned char PWMbit;
    unsigned short outDataIndex;
    unsigned short outputData;
    unsigned long matrixData, REDmask, GREENmask, BLUEmask;
    short row, col;
    outDataIndex = 0;

    for (line = 0; line < MAXLINE; line++) {
        REDmask = RED_LSB;
        GREENmask = GREEN_LSB;
        BLUEmask = BLUE_LSB;
        for (PWMbit = 0; PWMbit < COLORDEPTH; PWMbit++) {
#ifdef DUALMATRIX                  
            for (row = line; row < (MAXROW); row = row + (MAXLINE * 4)) {
#else
            for (row = line; row < (MAXROW); row = row + (MAXLINE * 2)) {
#endif                
                for (col = 0; col < MAXCOL; col++) {
                    outputData = 0x00;
#ifdef DUALMATRIX                  
                    matrixData = matrix[row][col];
                    if (REDmask & matrixData) outputData = outputData | R1bit;
                    if (GREENmask & matrixData) outputData = outputData | G1bit;
                    if (BLUEmask & matrixData) outputData = outputData | B1bit;

                    matrixData = matrix[row + MAXLINE][col];
                    if (REDmask & matrixData) outputData = outputData | R2bit;
                    if (GREENmask & matrixData) outputData = outputData | G2bit;
                    if (BLUEmask & matrixData) outputData = outputData | B2bit;

                    matrixData = matrix[row + MAXLINE * 2][col];
                    if (REDmask & matrixData) outputData = outputData | M2R1bit;
                    if (GREENmask & matrixData) outputData = outputData | M2G1bit;
                    if (BLUEmask & matrixData) outputData = outputData | M2B1bit;

                    matrixData = matrix[row + MAXLINE * 3][col];
                    if (REDmask & matrixData) outputData = outputData | M2R2bit;
                    if (GREENmask & matrixData) outputData = outputData | M2G2bit;
                    if (BLUEmask & matrixData) outputData = outputData | M2B2bit;

#else                  

                    matrixData = matrix[row][col];
                    if (REDmask & matrixData) outputData = outputData | R1bit;
                    if (GREENmask & matrixData) outputData = outputData | G1bit;
                    if (BLUEmask & matrixData) outputData = outputData | B1bit;

                    matrixData = matrix[row + MAXLINE][col];
                    if (REDmask & matrixData) outputData = outputData | R2bit;
                    if (GREENmask & matrixData) outputData = outputData | G2bit;
                    if (BLUEmask & matrixData) outputData = outputData | B2bit;


#endif                  

                    //#ifdef REV2BOARD
                    matrixOutData[outDataIndex] = outputData;
                    // if (outputData) printf("\rData: %x, index: %d", outputData, outDataIndex);
                    //#else
                    //                    matrixOutData[outDataIndex] = ~outputData;
                    //#endif                    
                    outDataIndex++;
                }
            }

            REDmask = REDmask << 1;
            GREENmask = GREENmask << 1;
            BLUEmask = BLUEmask << 1;
        }
    }
}

/********************************************************************
 * Function:        void ProcessIO(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is a place holder for other user
 *                  routines. It is a mixture of both USB and
 *                  non-USB tasks.
 *
 * Note:            None
 *******************************************************************/
void ProcessIO(void) {
    short i;
    unsigned char USBdata[USBPACKETSIZE];
    unsigned char dataByte;
    short numBytes, numUSBBytes;
    static short inBufferIndex = 0;
    unsigned char USBpacket[MAX_PANEL_DATASIZE];
    unsigned char error;

    //Blink the LEDs according to the USB device status
    BlinkUSBStatus();
    // User Application USB tasks
    if ((USBDeviceState < CONFIGURED_STATE) || (USBSuspendControl == 1)) return;

    if (USBUSARTIsTxTrfReady()) {
        numUSBBytes = getsUSBUSART(USBpacket, 64);
        if (numUSBBytes) {
            if (!USBstatus) inBufferIndex = 0;
            numBytes = decodePacket(USBpacket, USBdata);
            if (USBstatus) {
                for (i = 0; i < numBytes; i++) {
                    dataByte = USBdata[i];
                    if (inBufferIndex < MAX_PANEL_DATASIZE)
                        USBpanelData[inBufferIndex++] = dataByte;
                }
                numUSBBytes = 0;
                if (USBstatus == USB_DONE) {
                    USBstatus = USB_STANDBY;
                    error = processUSBpanelData(USBpanelData, inBufferIndex);
                    if (error == USB_LENGTH_ERROR) printf("\rLENGTH ERR: %d", inBufferIndex);
                    else if (error == USB_COMMAND_ERROR) printf("\rSUBCOMMAND > RANGE");
                    else printf("\rOK: %d", inBufferIndex);
                    putUSBUSART(USBreplyBuffer, USBPACKETSIZE);
                    inBufferIndex = 0;
                } else if (USBstatus == USB_PACKETERROR) {
                    USBstatus = USB_STANDBY;
                    printf("\rUSB PACKET ERR: %d", inBufferIndex);
                    putUSBUSART(USBreplyBuffer, USBPACKETSIZE);
                    inBufferIndex = 0;
                }
            }
        }
    }
    CDCTxService();
} //end ProcessIO

unsigned char updateOutputPanel(unsigned char panelNumber) {
    unsigned short line, col;
    unsigned char PWMbit;
    unsigned short outDataIndex, offset;
    unsigned short outputData;
    unsigned long matrixData, REDmask, GREENmask, BLUEmask;
    unsigned char evenFlag;

    if ((panelNumber % 2) == 0) evenFlag = TRUE;
    else evenFlag = FALSE;

    offset = (PANELS_ACROSS - 1) * PANELCOLS;
    outDataIndex = (panelNumber / 2) * PANELCOLS;

    for (line = 0; line < MAXLINE; line++) {
        REDmask = RED_LSB;
        GREENmask = GREEN_LSB;
        BLUEmask = BLUE_LSB;
        for (PWMbit = 0; PWMbit < COLORDEPTH; PWMbit++) {
            for (col = 0; col < PANELCOLS; col++) {

                if (evenFlag) {
                    outputData = matrixOutData[outDataIndex] & EVEN_MASK;

                    matrixData = panelData[panelNumber][line][col];
                    if (REDmask & matrixData) outputData = outputData | R1bit;
                    if (GREENmask & matrixData) outputData = outputData | G1bit;
                    if (BLUEmask & matrixData) outputData = outputData | B1bit;

                    matrixData = panelData[panelNumber][line + MAXLINE][col];
                    if (REDmask & matrixData) outputData = outputData | R2bit;
                    if (GREENmask & matrixData) outputData = outputData | G2bit;
                    if (BLUEmask & matrixData) outputData = outputData | B2bit;

                    matrixOutData[outDataIndex] = outputData;
                } else {
                    outputData = matrixOutData[outDataIndex] & ODD_MASK;

                    matrixData = panelData[panelNumber][line][col];
                    if (REDmask & matrixData) outputData = outputData | M2R1bit;
                    if (GREENmask & matrixData) outputData = outputData | M2G1bit;
                    if (BLUEmask & matrixData) outputData = outputData | M2B1bit;

                    matrixData = panelData[panelNumber][line + MAXLINE][col];
                    if (REDmask & matrixData) outputData = outputData | M2R2bit;
                    if (GREENmask & matrixData) outputData = outputData | M2G2bit;
                    if (BLUEmask & matrixData) outputData = outputData | M2B2bit;

                    matrixOutData[outDataIndex] = outputData;
                }
                outDataIndex++;
            }
            outDataIndex = outDataIndex + offset;
            REDmask = REDmask << 1;
            GREENmask = GREENmask << 1;
            BLUEmask = BLUEmask << 1;
        }
    }
}

#define LSB (0x01<<(8-COLORDEPTH))
#define HALFSIZE (PANELSIZE/2)

#define BRED_LSB   0b00100000 // $$$$
#define BGREEN_LSB 0b00000100
#define BBLUE_LSB  0b00000001

unsigned char updateOutputPanelPtr(unsigned char panelNumber, unsigned char *ptrPanelData) {
    unsigned short line, col;
    unsigned char PWMbit;
    unsigned short outDataIndex, offset, ptrIndex;
    unsigned short outputData;
    unsigned char lowMatrixByte, hiMatrixByte;
    unsigned char evenFlag = FALSE;
    unsigned char mask, redMask, greenMask, blueMask;  // $$$$

    if ((panelNumber % 2) == 0) evenFlag = TRUE;
    offset = (PANELS_ACROSS - 1) * PANELCOLS;
    outDataIndex = (panelNumber / 2) * PANELCOLS;

    for (line = 0; line < MAXLINE; line++) {
        mask = LSB;
        redMask = BRED_LSB;
        greenMask = BGREEN_LSB;
        blueMask = BBLUE_LSB;        
        for (PWMbit = 0; PWMbit < COLORDEPTH; PWMbit++) {
            ptrIndex = line * PANELCOLS * NUMCHANNELS;
            for (col = 0; col < PANELCOLS; col++) {
                if (evenFlag) {
                    outputData = matrixOutData[outDataIndex] & EVEN_MASK;
                    lowMatrixByte = ptrPanelData[ptrIndex];
                    hiMatrixByte = ptrPanelData[ptrIndex + HALFSIZE];
                    mask = redMask;
                    if (mask & lowMatrixByte) outputData = outputData | R1bit;
                    if (mask & hiMatrixByte) outputData = outputData | R2bit;
                    // ptrIndex++; $$$$

                    lowMatrixByte = ptrPanelData[ptrIndex];
                    hiMatrixByte = ptrPanelData[ptrIndex + HALFSIZE];
                    mask = greenMask;
                    if (mask & lowMatrixByte) outputData = outputData | G1bit;
                    if (mask & hiMatrixByte) outputData = outputData | G2bit;
                    // ptrIndex++; $$$$

                    lowMatrixByte = ptrPanelData[ptrIndex];
                    hiMatrixByte = ptrPanelData[ptrIndex + HALFSIZE];
                    mask = blueMask;
                    if (PWMbit > 0){
                        if (mask & lowMatrixByte) outputData = outputData | B1bit;
                        if (mask & hiMatrixByte) outputData = outputData | B2bit;
                    }
                    ptrIndex++;

                } else {
                    outputData = matrixOutData[outDataIndex] & ODD_MASK;
                    lowMatrixByte = ptrPanelData[ptrIndex];
                    hiMatrixByte = ptrPanelData[ptrIndex + HALFSIZE];
                    mask = redMask;
                    if (mask & lowMatrixByte) outputData = outputData | M2R1bit;
                    if (mask & hiMatrixByte) outputData = outputData | M2R2bit;
                    // ptrIndex++; $$$$

                    lowMatrixByte = ptrPanelData[ptrIndex];
                    hiMatrixByte = ptrPanelData[ptrIndex + HALFSIZE];
                    mask = greenMask;
                    if (mask & lowMatrixByte) outputData = outputData | M2G1bit;
                    if (mask & hiMatrixByte) outputData = outputData | M2G2bit;
                    // ptrIndex++; $$$$

                    lowMatrixByte = ptrPanelData[ptrIndex];
                    hiMatrixByte = ptrPanelData[ptrIndex + HALFSIZE];
                    mask = blueMask;
                    if (PWMbit > 0){
                        if (mask & lowMatrixByte) outputData = outputData | M2B1bit;
                        if (mask & hiMatrixByte) outputData = outputData | M2B2bit;
                    }
                    ptrIndex++;
                }
                matrixOutData[outDataIndex++] = outputData;
            }
            outDataIndex = outDataIndex + offset;
            mask = mask << 1;
            redMask = redMask << 1;
            greenMask = greenMask << 1;
            if (PWMbit > 0) blueMask = blueMask << 1;
        }
    }
    return(1);
}

unsigned char uncompressData(unsigned char *ptrCompressedData, unsigned char *ptrPanelData) {
    unsigned short i, j;
    unsigned short Red, Blue, Green;
    unsigned short compressedInt;

    union {
        unsigned char byte[2];
        unsigned short integer;
    } convert;


    if (ptrCompressedData == NULL || ptrPanelData == NULL) return (FALSE);

    j = 0;
    i = 0;
    do {
        convert.byte[0] = ptrCompressedData[i++];
        convert.byte[1] = ptrCompressedData[i++];
        compressedInt = convert.integer;
        Red = (compressedInt & 0b1111100000000000) >> 8;
        Green = (compressedInt & 0b0000011111100000) >> 3;
        Blue = (compressedInt & 0b0000000000011111) << 3;

        ptrPanelData[j++] = (unsigned char) Red;
        ptrPanelData[j++] = (unsigned char) Green;
        ptrPanelData[j++] = (unsigned char) Blue;

    } while (i < COMPRESSED_SIZE);
    return (TRUE);
}

/** EOF main.c *************************************************/

