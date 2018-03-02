/**/
/*===========================================================================*/
/**
 **      @file    main.c
 **
 **      @brief   acts bootloader main module
 **/
/*===========================================================================*/
/**/

/** CONFIGURATION **************************************************/
// PIC24FJ64GA704 Configuration Bit Settings
// 'C' source line config statements
// FSEC
#pragma config BWRP = OFF               // Boot Segment Write-Protect bit (Boot Segment may be written)
#pragma config BSS = DISABLED           // Boot Segment Code-Protect Level bits (No Protection (other than BWRP))
#pragma config BSEN = OFF               // Boot Segment Control bit (No Boot Segment)
//#pragma config BSEN = ON    // Boot Segment Control bit->Boot Segment size determined by FBSLIM
#pragma config GWRP = OFF               // General Segment Write-Protect bit (General Segment may be written)
#pragma config GSS = DISABLED           // General Segment Code-Protect Level bits (No Protection (other than GWRP))
#pragma config CWRP = OFF               // Configuration Segment Write-Protect bit (Configuration Segment may be written)
#pragma config CSS = DISABLED           // Configuration Segment Code-Protect Level bits (No Protection (other than CWRP))
//#pragma config AIVTDIS = ON             // Alternate Interrupt Vector Table bit (Enabled AIVT)
#pragma config AIVTDIS = OFF             // Alternate Interrupt Vector Table bit (Disabeld AIVT)

// FBSLIM
#pragma config BSLIM = 0x1FFF           // Boot Segment Flash Page Address Limit bits (Boot Segment Flash page address  limit)

// FSIGN

// FOSCSEL
#pragma config FNOSC = PRIPLL           // Oscillator Source Selection (Primary Oscillator with PLL module (XT + PLL, HS + PLL, EC + PLL))
// We can use 96MHZ PLL OR PLLX4: both are correct
#pragma config PLLMODE = PLL96DIV2      // PLL Mode Selection (96 MHz PLL. Oscillator input is divided by 2 (8 MHz input))
//#pragma config PLLMODE = PLL4X          // PLL Mode Selection->4x PLL selected
//#pragma config IESO = OFF             // Two-speed Oscillator Start-up Enable bit (Start up with user-selected oscillator source)
#pragma config IESO = ON                // Two-speed Oscillator Start-up Enable bit->Start up device with FRC, then switch to user-selected oscillator source

// FOSC
#pragma config POSCMD = XT              // Primary Oscillator Mode Select bits (XT Crystal Oscillator Mode)
#pragma config OSCIOFCN = ON            // OSC2 Pin Function bit (OSC2 is general purpose digital I/O pin)
#pragma config SOSCSEL = OFF            // SOSC Power Selection Configuration bits->Digital (SCLKI) mode

#pragma config PLLSS = PLL_PRI          // PLL Secondary Selection Configuration bit (PLL is fed by the Primary oscillator)
//#pragma config IOL1WAY = ON             // Peripheral pin select configuration bit->Allow only one reconfiguration
#pragma config FCKSM = CSECMD           // Clock Switching Mode bits (Clock switching is enabled,Fail-safe Clock Monitor is disabled)

// FWDT
// 2sec
//#pragma config WDTPS = PS128          // Watchdog Timer Postscaler bits->1:128
//#pragma config FWPSA = PR512          // Watchdog Timer Prescaler bit->1:512
// 512msec
#pragma config WDTPS = PS128             // Watchdog Timer Postscaler bits->1:128
#pragma config FWPSA = PR128            // Watchdog Timer Prescaler bit->1:128
//#pragma config FWDTEN = ON            // Watchdog Timer Enable bits->WDT Enabled
#pragma config FWDTEN = ON_SWDTEN       // Watchdog Timer Enable bits->WDT Enabled/Disabled (controlled using SWDTEN bit)
#pragma config WINDIS = OFF             // Watchdog Timer Window Enable bit->Watchdog Timer in Non-Window mode
#pragma config WDTWIN = WIN25           // Watchdog Timer Window Select bits->WDT Window is 25% of WDT period
#pragma config WDTCMX = WDTCLK          // WDT MUX Source Select bits->WDT clock source is determined by the WDTCLK Configuration bits
#pragma config WDTCLK = SYSCLK          // WDT Clock Source Select bits->WDT uses system clock when active, LPRC while in Sleep mode

// FPOR
#pragma config BOREN = OFF              // Brown Out Enable bit (Brown Out Disabled)
#pragma config LPCFG = OFF              // Low power regulator control (No Retention Sleep)
#pragma config DNVPEN = ENABLE          // Downside Voltage Protection Enable bit (Downside protection enabled using ZPBOR when BOR is inactive)

// FICD
#pragma config ICS = PGD3               // ICD Communication Channel Select bits (Communicate on PGEC3 and PGED3)
#pragma config JTAGEN = OFF             // JTAG Enable bit (JTAG is disabled)

// FDEVOPT1
#pragma config ALTCMPI = DISABLE        // Alternate Comparator Input Enable bit (C1INC, C2INC, and C3INC are on their standard pin locations)
#pragma config TMPRPIN = OFF            // Tamper Pin Enable bit (TMPRN pin function is disabled)
#pragma config SOSCHP = ON              // SOSC High Power Enable bit (valid only when SOSCSEL = 1 (Enable SOSC high power mode (default))
#pragma config ALTI2C1 = ALTI2CEN       // Alternate I2C pin Location (SDA1 and SCL1 on RB9 and RB8)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

/*============================================================================*/
#include "GenericTypeDefs.h"
#include "Compiler.h"

#include "p24FJ64GA704.h"
#include "serialCom.h"

#include "define.h"
#include "Macro.h"
#include "mem.h"
#include "ram.h"
#include "const.h"
#include "main.h"
#include "BL_UART_ServerMg.h"

#include "Bootloader.h"
#include "TimerMg.h"

#include "mem.h"
#include "progMemFunctions.h"
#include "ram.h"

/* Macro per la definizione degli Array dei filtri */
#define FILTER_WINDOW           3

/* Larghezza della finestra del filtro */
#define FILTER_WINDOW_LENGTH    (FILTER_WINDOW-1)
#define FILTER_WINDOW_LOOP      (FILTER_WINDOW-2)
#define MAX_CHANGE              (FILTER_WINDOW/2)
#define MIN_COUNT               (FILTER_WINDOW*3/4)
#define LOW_FILTER              0
#define HIGH_FILTER             1
#define ERRORE_FILTRO           2
#define COUNT_RESET             0
#define INPUT_ARRAY             6

/* Bootloader struct lives @ 0x200 */
const BootloaderPointers_T __attribute__ ((space(prog), address (0x200))) Bootloader = {
  BL_CRCarea,
  BL_CRCareaFlash
};

const unsigned long __attribute__ ((space(psv), address (__BL_SW_VERSION)))
dummy0 = BL_SW_VERSION; /* 0x0170 */

const unsigned short __attribute__ ((space(psv), address (__BL_CODE_CRC)))
dummy1 = 0; /* this will be changed in the .hex file by the CRC helper */

__psv__ const unsigned short *ptrBLCRCFlash = (unsigned short *) __BL_CODE_CRC;

static void BL_Init(void);
void BL_UserInit(void);
char CheckApplicationPresence(DWORD address);

void jump_to_appl();

void BL_ServerMg(void);
static void BL_IORemapping(void);

void BL_inputManager(void);
//static unsigned char BL_FilterSensorInput(unsigned char InputFilter);
static void BL_ReadDigInStatus(void);

// -----------------------------------------------------------------------------
//                      APPLICATION PROGRAM Service Routine
void APPLICATION_T1_InterruptHandler(void);
void APPLICATION_U1TX_InterruptHandler(void);
void APPLICATION_U1RX_InterruptHandler(void);
void APPLICATION_MI2C1_InterruptHandler(void);
void APPLICATION_SPI1_InterruptHandler(void);
void APPLICATION_SPI1TX_InterruptHandler(void);
void APPLICATION_SPI1RX_InterruptHandler(void);
//                      BOOT Service Routine NOT USED
void BOOT_MI2C1_InterruptHandler(void);
void BOOT_SPI1_InterruptHandler(void);
void BOOT_SPI1TX_InterruptHandler(void);
void BOOT_SPI1RX_InterruptHandler(void);

void Pippo(void);
// -----------------------------------------------------------------------------

/** T Y P E D E F S ******************************************************************* */
typedef union {
  unsigned char byte;
  struct {
    unsigned char  StatusType0: 1;
    unsigned char  StatusType1: 1;
    unsigned char  StatusType2: 1;
    unsigned char  StatusType3: 1;
    unsigned char  StatusType4: 1;
    unsigned char  StatusType5: 1;
    unsigned char  StatusType6: 1;
    unsigned char  StatusType7: 1;
  } Bit;
} DigInStatusType;

DWORD_VAL ReadFlashMemory;

/** D A T A ****************************************************************** */
static DigInStatusType BL_DigInStatus, BL_DigInNotFiltered;

/** T E X T ****************************************************************** */
void BL_GestStandAlone(void)
/*
**=============================================================================
**
**      Oggetto        : Gestione accensione e spegnimento apparecchio
**                                       attraverso il tasto ON/OFF
**
**      Parametri      : void
**
**      Ritorno        : void
**
**      Vers. - autore : 1.0  nuovo  G.Comai
**
**=============================================================================
*/
{
  if (BLState.livello == INIT) {
    if ((StatusTimer(T_WAIT_FORCE_BL) == T_RUNNING) && isUART_Force_Slave_BL_Cmd()) {
      BL_ForceStandAlone();
      resetNewProcessingMsg();
    }
    else if (StatusTimer(T_WAIT_FORCE_BL) == T_ELAPSED &&
             BL_StandAlone == BL_WAIT_CHECK_STAND_ALONE)

      BL_StandAlone = CheckApplicationPresence(BL_STAND_ALONE_CHECK);

    if (StatusTimer(T_WAIT_FORCE_BL) == T_ELAPSED)
      StopTimer(T_WAIT_FORCE_BL);

  } /* (BLState.level == INIT) */
}

/*
**=============================================================================
**
**      Oggetto        : Funzione di servizio degli Interrupt NON usati dal 
**                       BootLoader
**                                    
**      Parametri      : void
**
**      Ritorno        : void
**
**      Vers. - autore : 1.0 Michele Abelli
**
**=============================================================================
*/
void Pippo(void)
{
   unsigned int a;
   for (a = 0; a < 10; a++){}
}
 
 
unsigned short BL_CRCarea(unsigned char *pointer, unsigned short n_char,unsigned short CRCinit)
/**=============================================================================
**
**      Oggetto        : Calcola CRC di una zona di byte specificata
**                       dai parametri di ingresso
**
**      Parametri      : pointer      Indirizzo iniziale dell'area
**                                    da controllare
**                       n_char       Numero dei bytes da includere nel calcolo
**                       CRCinit      Valore iniziale di CRC ( = 0 se n_char
**                                    copre l'intera zona da verificare,
**                                    = CRCarea della zona precedente se
**                                    si sta procedendo a blocchi
**
**      Ritorno        : CRCarea      Nuovo valore del CRC calcolato
**
**      Vers. - autore : 1.0  nuovo   G. Comai
**
**=============================================================================*/
{
  /* La routine proviene dalla dispensa "CRC Fundamentals", pagg. 196, 197. */

  /* Nota sull'algoritmo:
     dato un vettore, se ne calcoli il CRC_16: se si accodano i 2 bytes del
     CRC_16 a tale vettore (low byte first!!!), il CRC_16 calcolato
     sul vettore così ottenuto DEVE valere zero.
     Tale proprietà può essere sfruttata nelle comunicazione seriali
     per verificare che un messaggio ricevuto,
     contenente in coda i 2 bytes del CRC_16 (calcolati dal trasmettitore),
     sia stato inviato correttamente: il CRC_16, calcolato dal ricevente
     sul messaggio complessivo deve valere zero. */

  unsigned long i;
  unsigned short index;
//  unsigned char psv_shadow;

  /* save the PSVPAG */
//  psv_shadow = PSVPAG;

  /* set the PSVPAG for accessing CRC_TABLE[] */
//  PSVPAG = __builtin_psvpage (BL_CRC_TABLE);

  for (i = 0; i < n_char; i++) {
    index = ( (CRCinit ^ ( (unsigned short) *pointer & 0x00FF) ) & 0x00FF);
    CRCinit = ( (CRCinit >> 8) & 0x00FF) ^ BL_CRC_TABLE[index];
    pointer = pointer + 1;

#if ! defined __DEBUG	
    /* Reset Watchdog*/
    ClrWdt();
#endif	
  } /* end for */

  /* restore the PSVPAG for the compiler-managed PSVPAG */
//  PSVPAG = psv_shadow;

  return CRCinit;
} /* end CRCarea */

unsigned short BL_CRCareaFlash(unsigned long address, unsigned long n_word,unsigned short CRCinit)
/**=============================================================================
**
**      Oggetto        : Calcola CRC di una zona di byte specificata
**                       dai parametri di ingresso
**
**      Parametri      : pointer      Indirizzo iniziale dell'area
**                                    da controllare
**                       n_char       Numero dei bytes da includere nel calcolo
**                       CRCinit      Valore iniziale di CRC ( = 0 se n_char
**                                    copre l'intera zona da verificare,
**                                    = CRCarea della zona precedente se
**                                    si sta procedendo a blocchi
**
**      Ritorno        : CRCarea      Nuovo valore del CRC calcolato
**
**      Vers. - autore : 1.0  nuovo   G. Comai
**
**=============================================================================*/
{
  DWORD_VAL dwvResult;
  /* La routine proviene dalla dispensa "CRC Fundamentals", pagg. 196, 197. */

  /* Nota sull'algoritmo:
     dato un vettore, se ne calcoli il CRC_16: se si accodano i 2 bytes del
     CRC_16 a tale vettore (low byte first!!!), il CRC_16 calcolato
     sul vettore così ottenuto DEVE valere zero.
     Tale proprietà può essere sfruttata nelle comunicazione seriali
     per verificare che un messaggio ricevuto,
     contenente in coda i 2 bytes del CRC_16 (calcolati dal trasmettitore),
     sia stato inviato correttamente: il CRC_16, calcolato dal ricevente
     sul messaggio complessivo deve valere zero. */

  unsigned long i;
  unsigned char j;
  unsigned short index;
//  unsigned char psv_shadow;

  WORD wTBLPAGSave;

  /* save the PSVPAG */
//  psv_shadow = PSVPAG;
  /* set the PSVPAG for accessing CRC_TABLE[] */
//  PSVPAG = __builtin_psvpage (BL_CRC_TABLE);

  for (i = 0; i < n_word; i++) {

#if ! defined __DEBUG	
    /* Reset Watchdog*/
    ClrWdt();
#endif	
  
    wTBLPAGSave = TBLPAG;
    TBLPAG = ((DWORD_VAL*)&address)->w[1];

    dwvResult.w[1] = __builtin_tblrdh((WORD)address);
    dwvResult.w[0] = __builtin_tblrdl((WORD)address);
    TBLPAG = wTBLPAGSave;
    for (j=0; j<4; j++) {
      index = ( (CRCinit ^ ( (unsigned short) dwvResult.v[j] & 0x00FF) ) & 0x00FF);
      CRCinit = ( (CRCinit >> 8) & 0x00FF) ^ BL_CRC_TABLE[index];
    }
    address+=2;
  } /* end for */

  /* restore the PSVPAG for the compiler-managed PSVPAG */
//  PSVPAG = psv_shadow;

  return CRCinit;
} /* end CRCarea */

/********************************************************************
 * Function:        static void BL_Init(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        BL_Init is a centralize initialization
 *                  routine. All required USB initialization routines
 *                  are called from here.
 *
 *                  User application initialization routine should
 *                  also be called from here.
 *
 * Note:            None
 *******************************************************************/
static void BL_Init(void)
{ 
  /* Use Alternate Vector Table */
  //INTCON2bits.AIVTEN = 1;
    
  // Enable nested interrupts
  INTCON1bits.NSTDIS = 0;

  BL_IORemapping();
  BL_TimerInit();
  BL_initSerialCom();
  BL_UserInit();
}

/********************************************************************
 * Function:        void BL_UserInit(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        BL_User_Init is a centralize initialization
 *                  routine. All required USB initialization routines
 *                  are called from here.
 *
 *                  User application initialization routine should
 *                  also be called from here.
 *
 * Note:            None
 *******************************************************************/
void BL_UserInit(void)
{
  BLState.livello = INIT;
  BL_StandAlone = BL_WAIT_CHECK_STAND_ALONE;

  StartTimer(T_FIRST_WINDOW);
  StartTimer(T_WAIT_FORCE_BL);
}

static void BL_IORemapping(void)
{
    /****************************************************************************
     * Setting the Output Latch SFR(s)
     ***************************************************************************/
    LATA = 0x0000;
    LATB = 0x0000;
    LATC = 0x0000;

    /****************************************************************************
     * Setting the Weak Pull Up and Weak Pull Down SFR(s)
     ***************************************************************************/
    IOCPDA = 0x0000;
    IOCPDB = 0x0000;
    IOCPDC = 0x0000;
    IOCPUA = 0x0000;
    IOCPUB = 0x0000;
    IOCPUC = 0x0000;

    /****************************************************************************
     * Setting the Open Drain SFR(s)
     ***************************************************************************/
    ODCA = 0x0000;
    ODCB = 0x0000;
    ODCC = 0x0000;

    /****************************************************************************
     * Setting the Analog/Digital Configuration SFR(s)
     ***************************************************************************/
    // Set AN0 and AN1
    ANSA = 0x0003;
    ANSB = 0x0000;
    ANSC = 0x0000;

    // Set as Digital I/O
    ANSBbits.ANSB2 = 0; // RB2
    ANSBbits.ANSB3 = 0; // RB3
    // Set as Digital I/O
    ANSCbits.ANSC0 = 0; // RC0        
    ANSCbits.ANSC1 = 0; // RC1      
    ANSCbits.ANSC2 = 0; // RC2       
    
    /****************************************************************************
     * Setting the GPIO Direction SFR(s)
     ***************************************************************************/
    // Initialization function to define IO
    //TRISA = 0x069F;
    //TRISB = 0xA2E4;
    //TRISC = 0x03FF;
	TRISBbits.TRISB0 = OUTPUT; // UART_DE
	TRISBbits.TRISB1 = OUTPUT; // TMP_RESET
	TRISBbits.TRISB2 = INPUT;  // TMP_ALERT 
	TRISBbits.TRISB3 = OUTPUT; // PUMP
	TRISBbits.TRISB4 = OUTPUT; // NEB
	TRISBbits.TRISB7 = INPUT;  // LEVEL
	TRISBbits.TRISB8 = OUTPUT; // I2C0_SDL
	TRISBbits.TRISB9 = INPUT;  // I2C0_SDA  
	TRISBbits.TRISB10 = OUTPUT;// SPI_SS
	TRISBbits.TRISB11 = OUTPUT;// SPI_SCK
	TRISBbits.TRISB12 = OUTPUT;// SPI_SD0
	TRISBbits.TRISB13 = INPUT; // SPI_SDI
  	TRISBbits.TRISB14 = OUTPUT;// UART_TX
	TRISBbits.TRISB15 = INPUT; // UART_RX
    
	TRISCbits.TRISC0 = INPUT; // Dip Switch 1
	TRISCbits.TRISC1 = INPUT; // Dip Switch 2
	TRISCbits.TRISC2 = INPUT; // Dip Switch 3
	TRISCbits.TRISC3 = INPUT; // Dip Switch 4
	TRISCbits.TRISC4 = INPUT; // Dip Switch 5
	TRISCbits.TRISC5 = INPUT; // Dip Switch 6
    
	TRISAbits.TRISA0 = INPUT; // AN0
	TRISAbits.TRISA1 = INPUT; // AN1
	TRISAbits.TRISA2 = INPUT; // OSC0
	TRISAbits.TRISA3 = INPUT; // OSC1
	TRISAbits.TRISA8 = OUTPUT;// LED	

}/* end IORemapping() */

/********************************************************************
 * Function:        char CheckApplPres(DWORD address)
 *
 * PreCondition:    None
 *
 * Input:           Long -> First Application Flash Address
 *
 * Output:          Char -> BootLoader Stand Alone = TRUE
 *
 * Side Effects:    None
 *
 * Overview:        CheckApplPres is a centralize initialization
 *                  routine. All required USB initialization routines
 *                  are called from here.
 *
 *                  User application initialization routine should
 *                  also be called from here.
 *
 * Note:            None
 *******************************************************************/
//static char CheckApplicationPresence(DWORD address)
char CheckApplicationPresence(DWORD address)
{
//  if (ReadProgramMemory(address) == APPL_FLASH_MEMORY_ERASED_VALUE)
//    return BL_STAND_ALONE;

  return BL_NO_STAND_ALONE;
} /* end CheckApplicationPresence() */

static void BL_ReadDigInStatus(void)
/*
*//*=====================================================================*//**
**
**      @brief Reads digital input
**
**      @param void
**
**      @retval void
**
**
*//*=====================================================================*//**
*/
{
#if defined FORCE_SLAVE_0
  BL_DigInNotFiltered.byte = 0;

#elif defined FORCE_SLAVE_1
  BL_DigInNotFiltered.byte = 1;

#elif defined FORCE_SLAVE_2
  BL_DigInNotFiltered.byte = 2;

#elif defined FORCE_SLAVE_3
  BL_DigInNotFiltered.byte = 3;

#elif defined FORCE_SLAVE_4
  BL_DigInNotFiltered.byte = 4;

#elif defined FORCE_SLAVE_5
  BL_DigInNotFiltered.byte = 5;

#elif defined FORCE_SLAVE_6
  BL_DigInNotFiltered.byte = 6;

#elif defined FORCE_SLAVE_7
  BL_DigInNotFiltered.byte = 7;

#elif defined FORCE_SLAVE_8
  BL_DigInNotFiltered.byte = 8;

#elif defined FORCE_SLAVE_9
  BL_DigInNotFiltered.byte = 9;

#elif defined FORCE_SLAVE_10
  BL_DigInNotFiltered.byte = 10;

#elif defined FORCE_SLAVE_11
  BL_DigInNotFiltered.byte = 11;

#elif defined FORCE_SLAVE_12
  BL_DigInNotFiltered.byte = 12;

#elif defined FORCE_SLAVE_13
  BL_DigInNotFiltered.byte = 13;

#elif defined FORCE_SLAVE_14
  BL_DigInNotFiltered.byte = 14;

#elif defined FORCE_SLAVE_15
  BL_DigInNotFiltered.byte = 15;

#elif defined FORCE_SLAVE_16
  BL_DigInNotFiltered.byte = 16;

#elif defined FORCE_SLAVE_17
  BL_DigInNotFiltered.byte = 17;

#elif defined FORCE_SLAVE_18
  BL_DigInNotFiltered.byte = 18;

#elif defined FORCE_SLAVE_19
  BL_DigInNotFiltered.byte = 19;

#elif defined FORCE_SLAVE_20
  BL_DigInNotFiltered.byte = 20;

#elif defined FORCE_SLAVE_21
  BL_DigInNotFiltered.byte = 21;

#elif defined FORCE_SLAVE_22
  BL_DigInNotFiltered.byte = 22;

#elif defined FORCE_SLAVE_23
  BL_DigInNotFiltered.byte = 23;

#elif defined FORCE_SLAVE_24
  BL_DigInNotFiltered.byte = 24;

#elif defined FORCE_SLAVE_25
  BL_DigInNotFiltered.byte = 25;

#elif defined FORCE_SLAVE_26
  BL_DigInNotFiltered.byte = 26;

#elif defined FORCE_SLAVE_27
  BL_DigInNotFiltered.byte = 27;

#elif defined FORCE_SLAVE_28
  BL_DigInNotFiltered.byte = 28;

#elif defined FORCE_SLAVE_29
  BL_DigInNotFiltered.byte = 29;

#elif defined FORCE_SLAVE_30
  BL_DigInNotFiltered.byte = 30;

#elif defined FORCE_SLAVE_31
  BL_DigInNotFiltered.byte = 31;

#elif defined FORCE_SLAVE_32
  BL_DigInNotFiltered.byte = 32;

#elif defined FORCE_SLAVE_33
  BL_DigInNotFiltered.byte = 33;

#elif defined FORCE_SLAVE_34
  BL_DigInNotFiltered.byte = 34;

#elif defined FORCE_SLAVE_35
  BL_DigInNotFiltered.byte = 35;

#elif defined FORCE_SLAVE_36
  BL_DigInNotFiltered.byte = 36;

#elif defined FORCE_SLAVE_37
  BL_DigInNotFiltered.byte = 37;

#elif defined FORCE_SLAVE_38
  BL_DigInNotFiltered.byte = 38;

#elif defined FORCE_SLAVE_39
  BL_DigInNotFiltered.byte = 39;

#elif defined FORCE_SLAVE_40
  BL_DigInNotFiltered.byte = 40;

#elif defined FORCE_SLAVE_41
  BL_DigInNotFiltered.byte = 41;

#elif defined FORCE_SLAVE_42
  BL_DigInNotFiltered.byte = 42;

#elif defined FORCE_SLAVE_43
  BL_DigInNotFiltered.byte = 43;

#elif defined FORCE_SLAVE_44
  BL_DigInNotFiltered.byte = 44;

#elif defined FORCE_SLAVE_45
  BL_DigInNotFiltered.byte = 45;

#elif defined FORCE_SLAVE_46
  BL_DigInNotFiltered.byte = 46;

#elif defined FORCE_SLAVE_47
  BL_DigInNotFiltered.byte = 47;

#else
	/* Read 485 address bits from dip-switch on S1 */
	BL_DigInNotFiltered.Bit.StatusType0 = ~SW3;
	BL_DigInNotFiltered.Bit.StatusType1 = ~SW2;
	BL_DigInNotFiltered.Bit.StatusType2 = ~SW1;
	BL_DigInNotFiltered.Bit.StatusType3 = ~SW4;
	BL_DigInNotFiltered.Bit.StatusType4 = ~SW5;
	BL_DigInNotFiltered.Bit.StatusType5 = ~SW6;
	BL_DigInNotFiltered.Bit.StatusType6 = 0;
	BL_DigInNotFiltered.Bit.StatusType7 = 0;  
#endif
}

void BL_inputManager(void)
/*
*//*=====================================================================*//**
**
**      @brief Sequencer of the INPUT module
**
**      @param void
**
**      @retval void
**
**
*//*=====================================================================*//**
*/
{
  BL_ReadDigInStatus();
  BL_DigInStatus.byte = BL_DigInNotFiltered.byte;
} /* end inputManager() */

static void hw_init(void)
{
    // Manually generated
// -----------------------------------------------------------------------------        
    // We can use 96MHZ PLL OR PLLX4: both are correct
	// 1. 96MHz PLL
    // POSTSCALER Clock Division = 1 --> Clock Frequency = 32MHZ - 16MIPS
    CLKDIVbits.CPDIV0 = 0;

    CLKDIVbits.CPDIV1 = 0;
    
	// unlock OSCCON register: 'NOSC' = primary oscillator with PLL module - 
    // 'OSWEN' = 1 initiate an oscillator switch to the clock source specified by 'NOSC' 
	__builtin_write_OSCCONH(0x03);
	__builtin_write_OSCCONL(0x01);

	/* wait for clock to stabilize: Primary Oscillator with PLL module (XTPLL, HSPLL))*/
	while (OSCCONbits.COSC != 0b011)
	  ;	
	/* wait for PLL to lock: PLL module is in lock, PLL start-up timer is satisfied */
	while (OSCCONbits.LOCK != 1)
	  ;
    // Auto generate initialization
// -----------------------------------------------------------------------------    
/*
	// 2. PLLX4
    // CF no clock failure; NOSC PRIPLL; SOSCEN disabled; POSCEN disabled; CLKLOCK unlocked; OSWEN Switch is Complete; IOLOCK not-active; 
    __builtin_write_OSCCONL((uint8_t) (0x0300 & 0x00FF));
    // CPDIV 1:1; PLLEN disabled; DOZE 1:8; RCDIV FRC; DOZEN disabled; ROI disabled; 
    CLKDIV = 0x3000;
    // STOR disabled; STORPOL Interrupt when STOR is 1; STSIDL disabled; STLPOL Interrupt when STLOCK is 1; STLOCK disabled; STSRC SOSC; STEN disabled; TUN Center frequency; 
    OSCTUN = 0x0000;
    // ROEN disabled; ROSEL FOSC; ROSIDL disabled; ROSWEN disabled; ROOUT disabled; ROSLP disabled; 
    REFOCONL = 0x0000;
    // RODIV 0; 
    REFOCONH = 0x0000;
    // ROTRIM 0; 
    REFOTRIML = 0x0000;
    // DCOTUN 0; 
    DCOTUN = 0x0000;
    // DCOFSEL 8; DCOEN disabled; 
    DCOCON = 0x0700;
    // DIV 0; 
    OSCDIV = 0x0000;
    // TRIM 0; 
    OSCFDIV = 0x0000;
*/
// -----------------------------------------------------------------------------    
}
//static void jump_to_appl()
void jump_to_appl()
{
  /* Write slave address into reserved data location before
   * relinquishing control to the application. */
  slave_index = (unsigned short) BL_slave_id ;

  /* Use standard vector table */
  //INTCON2bits.AIVTEN = 0;

  // Active Program: APPLICATION_PROGRAM
  program_active = APPLICATION_PROGRAM;
  
  /* jump to app code, won't return */
  __asm__ volatile ("goto " __APPL_GOTO_ADDR);
}

int main(void)
{
	// Hardware initialization 
	hw_init();

	// BootLoader app initialization 
	BL_Init();

	BL_inputManager();
    
	// Lettura dei Dip Switch
	BL_slave_id = 0x00FF & BL_DigInStatus.byte;
	//BL_slave_id = 43;
    
    // Active Program: BOOT
    program_active = BOOT;

#if ! defined __DEBUG	    
    ENABLE_WDT();
#endif	
    //EraseFlashPage(9);
    //WriteFlashWord(0x2400, 0x000F0F0FL);
    //ReadFlashMemory.Val = ReadProgramMemory((DWORD) (0x2400));   
	do {
#if ! defined __DEBUG	
		/* Reset Watchdog*/
		ClrWdt();
#endif	

    if(StatusTimer(T_FIRST_WINDOW) == T_ELAPSED) {
			BL_GestStandAlone();
			BL_serialCommManager();
			BL_UART_ServerMg();
		}
	    BL_TimerMg();
	} while(BL_StandAlone != BL_NO_STAND_ALONE);

    Nop();
	Nop();

	jump_to_appl(); /* goodbye */

  return 0; // unreachable, just get rid of compiler warning 
} // end main 

// -----------------------------------------------------------------------------
// INTERRUPT Service Routine
// TIMER 1
void __attribute__((__interrupt__,auto_psv)) _T1Interrupt(void)
{
    // Clear Timer 1 Interrupt Flag
    IFS0bits.T1IF = 0;                         
    
    // If BOOT is Active it runs 1 function, otherwise if APPLICATION_PROGRAM is acive it runs another
    if (program_active == BOOT)
      BOOT_T1_InterruptHandler();
    else
      APPLICATION_T1_InterruptHandler();
}
// UART1 RX 
void __attribute__((__interrupt__, no_auto_psv)) _U1RXInterrupt(void)
{
    // If BOOT is Active it runs 1 function, otherwise if APPLICATION_PROGRAM is acive it runs another
    if (program_active == BOOT)
      BOOT_U1RX_InterruptHandler();
    else
      APPLICATION_U1RX_InterruptHandler();
}
// UART1 TX 
void __attribute__((__interrupt__, no_auto_psv)) _U1TXInterrupt(void)
{
    // If BOOT is Active it runs 1 function, otherwise if APPLICATION_PROGRAM is acive it runs another
    if (program_active == BOOT)
      BOOT_U1TX_InterruptHandler();
    else
      APPLICATION_U1TX_InterruptHandler();
}
// I2C1
void __attribute__ ( ( interrupt, no_auto_psv ) ) _MI2C1Interrupt ( void )
{
    IFS1bits.MI2C1IF = 0;

    // If BOOT is Active it runs 1 function, otherwise if APPLICATION_PROGRAM is acive it runs another
    if (program_active == BOOT)
      BOOT_MI2C1_InterruptHandler();
    else
      APPLICATION_MI2C1_InterruptHandler();    
}
// SPI1 GENERAL
void __attribute__ ( ( interrupt, no_auto_psv ) ) _SPI1Interrupt ( void )
{
    // If BOOT is Active it runs 1 function, otherwise if APPLICATION_PROGRAM is acive it runs another
    if (program_active == BOOT)
      BOOT_SPI1_InterruptHandler();
    else
      APPLICATION_SPI1_InterruptHandler();    
}
// SPI1 TX
void __attribute__ ( ( interrupt, no_auto_psv ) ) _SPI1TXInterrupt ( void )
{
    // If BOOT is Active it runs 1 function, otherwise if APPLICATION_PROGRAM is acive it runs another
    if (program_active == BOOT)
      BOOT_SPI1TX_InterruptHandler();
    else
      APPLICATION_SPI1TX_InterruptHandler();    
}
// SPI1 RX
void __attribute__ ( ( interrupt, no_auto_psv ) ) _SPI1RXInterrupt ( void )
{
    // If BOOT is Active it runs 1 function, otherwise if APPLICATION_PROGRAM is acive it runs another
    if (program_active == BOOT)
      BOOT_SPI1RX_InterruptHandler();
    else
      APPLICATION_SPI1RX_InterruptHandler();    
}
// -----------------------------------------------------------------------------
//                      APPLICATION PROGRAM Service Routine
// Timer 1 Interrupt handler 
void __attribute__((address(__APPL_T1))) APPLICATION_T1_InterruptHandler(void)
{
    Pippo();
}
// UART1 RX Interrupt handler 
void __attribute__((address(__APPL_U1RX1))) APPLICATION_U1RX_InterruptHandler(void)
{
    Pippo();
}
// UART1 TX Interrupt handler 
void __attribute__((address(__APPL_U1TX1))) APPLICATION_U1TX_InterruptHandler(void)
{
    Pippo();
}
// I2C1 Interrupt handler 
void __attribute__((address(__APPL_MI2C1))) APPLICATION_MI2C1_InterruptHandler(void)
{
    Pippo();
}
// SPI1 Interrupt handler 
void __attribute__((address(__APPL_SPI1))) APPLICATION_SPI1_InterruptHandler(void)
{
    Pippo();
}
// SPI1TX Interrupt handler 
void __attribute__((address(__APPL_SPI1TX))) APPLICATION_SPI1TX_InterruptHandler(void)
{
    Pippo();
}
// SPI1RX Interrupt handler 
void __attribute__((address(__APPL_SPI1RX))) APPLICATION_SPI1RX_InterruptHandler(void)
{
    Pippo();
}
// -----------------------------------------------------------------------------
//                      BOOT Service Routine NOT USED
// I2C1 Interrupt handler 
void BOOT_MI2C1_InterruptHandler(void)
{
    Pippo();
}
// SPI1 GENERAL Interrupt handler 
void BOOT_SPI1_InterruptHandler(void)
{
    Pippo();
}
// SPI1TX Interrupt handler 
void BOOT_SPI1TX_InterruptHandler(void)
{
    Pippo();
}
// SPI1RX Interrupt handler 
void BOOT_SPI1RX_InterruptHandler(void)
{
    Pippo();
}
// -----------------------------------------------------------------------------