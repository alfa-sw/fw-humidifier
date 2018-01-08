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
#pragma config GWRP = OFF               // General Segment Write-Protect bit (General Segment may be written)
#pragma config GSS = DISABLED           // General Segment Code-Protect Level bits (No Protection (other than GWRP))
#pragma config CWRP = OFF               // Configuration Segment Write-Protect bit (Configuration Segment may be written)
#pragma config CSS = DISABLED           // Configuration Segment Code-Protect Level bits (No Protection (other than CWRP))
#pragma config AIVTDIS = ON             // Alternate Interrupt Vector Table bit (Enabled AIVT)

// FBSLIM
#pragma config BSLIM = 0x1FFF           // Boot Segment Flash Page Address Limit bits (Boot Segment Flash page address  limit)

// FSIGN

// FOSCSEL
#pragma config FNOSC = PRIPLL           // Oscillator Source Selection (Primary Oscillator with PLL module (XT + PLL, HS + PLL, EC + PLL))
#pragma config PLLMODE = PLL96DIV2      // PLL Mode Selection (96 MHz PLL. Oscillator input is divided by 2 (8 MHz input))
#pragma config IESO = OFF               // Two-speed Oscillator Start-up Enable bit (Start up with user-selected oscillator source)

// FOSC
#pragma config POSCMD = XT              // Primary Oscillator Mode Select bits (XT Crystal Oscillator Mode)
#pragma config OSCIOFCN = ON            // OSC2 Pin Function bit (OSC2 is general purpose digital I/O pin)
#pragma config SOSCSEL = ON             // SOSC Power Selection Configuration bits (SOSC is used in crystal (SOSCI/SOSCO) mode)
#pragma config PLLSS = PLL_PRI          // PLL Secondary Selection Configuration bit (PLL is fed by the Primary oscillator)
#pragma config IOL1WAY = OFF            // Peripheral pin select configuration bit (Allow multiple reconfigurations)
#pragma config FCKSM = CSECMD           // Clock Switching Mode bits (Clock switching is enabled,Fail-safe Clock Monitor is disabled)

// FWDT
#pragma config WDTPS = PS16             // Watchdog Timer Postscaler bits (1:16)
#pragma config FWPSA = PR32             // Watchdog Timer Prescaler bit (1:32)
#pragma config FWDTEN = ON              // Watchdog Timer Enable bits (WDT Enabled)
#pragma config WINDIS = OFF             // Watchdog Timer Window Enable bit (Watchdog Timer in Non-Window mode)
#pragma config WDTWIN = WIN25           // Watchdog Timer Window Select bits (WDT Window is 25% of WDT period)
#pragma config WDTCMX = WDTCLK          // WDT MUX Source Select bits (WDT clock source is determined by the WDTCLK Configuration bits)
#pragma config WDTCLK = LPRC            // WDT Clock Source Select bits (WDT uses LPRC)

// FPOR
#pragma config BOREN = OFF              // Brown Out Enable bit (Brown Out Disabled)
#pragma config LPCFG = OFF              // Low power regulator control (No Retention Sleep)
#pragma config DNVPEN = ENABLE          // Downside Voltage Protection Enable bit (Downside protection enabled using ZPBOR when BOR is inactive)

// FICD
#pragma config ICS = PGD2               // ICD Communication Channel Select bits (Communicate on PGEC2 and PGED2)
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
dummy0 = BL_SW_VERSION; /* 0x0150 */

const unsigned short __attribute__ ((space(psv), address (__BL_CODE_CRC)))
dummy1 = 0; /* this will be changed in the .hex file by the CRC helper */

__psv__ const unsigned short *ptrBLCRCFlash = (unsigned short *) __BL_CODE_CRC;

static void BL_Init(void);
void BL_UserInit(void);
//static char CheckApplicationPresence(DWORD address);
char CheckApplicationPresence(DWORD address);

void jump_to_appl();

void BL_ServerMg(void);
static void BL_IORemapping(void);

void BL_inputManager(void);
//static unsigned char BL_FilterSensorInput(unsigned char InputFilter);
static void BL_ReadDigInStatus(void);

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

/** D A T A *************************************************************************** */
//static signed char index_0, index_1;
//static unsigned char n_filter;
//static unsigned char FILTRAGGIO_LOW[FILTER_WINDOW];
//static unsigned char DummyOutput_low,shift;
//static unsigned char zero_counter, one_counter, ChangeStatus, Out_Status;
static DigInStatusType BL_DigInStatus, BL_DigInNotFiltered;

/** T E X T *************************************************************************** */
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

    /* Reset Watchdog*/
    ClrWdt();
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

    /* Reset Watchdog*/
    ClrWdt();

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
  INTCON2bits.AIVTEN = 1;

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
    // Set as Digital I/O
    ANSBbits.ANSB2 = 0; // RB2
    ANSBbits.ANSB3 = 0; // RB3
    // Set as Digital I/O
    ANSCbits.ANSC0 = 0; // RC0        
    ANSCbits.ANSC1 = 0; // RC1      
    ANSCbits.ANSC2 = 0; // RC2       
    
    // Initialization function to define IO
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

    // UART_RX --> RP15 
    RPINR18bits.U1RXR = 15;
    // UART_TX --> RP14   
    _RP14R = 3;
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
  if (ReadProgramMemory(address) == APPL_FLASH_MEMORY_ERASED_VALUE)
    return BL_STAND_ALONE;

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
	BL_DigInNotFiltered.Bit.StatusType0 = SW1;
	BL_DigInNotFiltered.Bit.StatusType1 = SW2;
	BL_DigInNotFiltered.Bit.StatusType2 = SW3;
	BL_DigInNotFiltered.Bit.StatusType3 = SW4;
	BL_DigInNotFiltered.Bit.StatusType4 = SW5;
	BL_DigInNotFiltered.Bit.StatusType5 = SW6;
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
  //BL_DigInStatus.byte = BL_FilterSensorInput(BL_DigInNotFiltered.byte);
  BL_DigInStatus.byte = BL_DigInNotFiltered.byte;
} /* end inputManager() */

//static unsigned char BL_FilterSensorInput(unsigned char InputFilter)
/*
*//*=====================================================================*//**
**
**      @brief Filter of the keys state
**
**      @param InputFilter input
**
**      @retval ouput filtered
**
**
**
*//*=====================================================================*//**
*/
/*
{
  const unsigned char MASK_BIT_8[]={0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80};

  unsigned char temp_uc;
  signed char temp_sc;

  // n_filter = Finestra campioni del filtro
  if (n_filter < FILTER_WINDOW_LENGTH)   
  {
    n_filter++;
  }
  else
  {
    n_filter = 0;
  }

  FILTRAGGIO_LOW[n_filter] = InputFilter;

  // INPUT_ARRAY = N° di ingressi da filtrare (8*2)
  for(temp_uc = 0 ; temp_uc < INPUT_ARRAY ; temp_uc++) 
  {
    shift = MASK_BIT_8[temp_uc];

    //ByteLow
    for(temp_sc = FILTER_WINDOW_LOOP ; temp_sc >= 0 ; temp_sc--)
    {
      // Indice 0
      index_0 = n_filter - temp_sc;
      if (index_0 < 0)
      {
        index_0 += FILTER_WINDOW;
      }
      // Indice 1
      index_1 = n_filter - temp_sc - 1;
      if (index_1 < 0)
      {
        index_1 += FILTER_WINDOW;
      }

      if ( (FILTRAGGIO_LOW[index_0] ^ FILTRAGGIO_LOW[index_1]) & shift)
      {
        ChangeStatus++;
      }

      if ( FILTRAGGIO_LOW[index_0] & shift)
      {
        one_counter++;
      }

      else
      {
        zero_counter++;
      }

      if (temp_sc == 0)
      {
        if (FILTRAGGIO_LOW[index_1] & shift)
        {
          one_counter++;
        }
        else
        {
          zero_counter++;
        }
      }
    }

    if (ChangeStatus > MAX_CHANGE)
    {
      if (zero_counter >= MIN_COUNT)
      {
        Out_Status = LOW_FILTER;
      }
      else if (one_counter >= MIN_COUNT)
      {
        Out_Status = HIGH_FILTER;
      }
      else
      {
        Out_Status = ERRORE_FILTRO;
      }
    }
    else
    {
      if (zero_counter > one_counter)
      {
        Out_Status = LOW_FILTER;
      }
      else
      {
        Out_Status = HIGH_FILTER;
      }
    }

    zero_counter = COUNT_RESET;
    one_counter  = COUNT_RESET;
    ChangeStatus = COUNT_RESET;

    // Segnale d'ingresso filtrato
    if (Out_Status != ERRORE_FILTRO)
    {
      if (!temp_uc)
      {
        DummyOutput_low = Out_Status;
      }
      else
      {
        DummyOutput_low |= (Out_Status << temp_uc);
      }
    }
  }
  return (DummyOutput_low);
} // end FilterSensorInput 
*/
static void hw_init(void)
{
	// Use 96MHz PLL	
    // POSTSCALER Clock Division = 1 --> Clock Frequency = 32MHZ - 16MIPS
    CLKDIVbits.CPDIV0 = 0;
    CLKDIVbits.CPDIV1 = 0;
    
	/* unlock OSCCON register: 'NOSC' = primary oscillator with PLL module - 
    'OSWEN' = 1 initiate an oscillator switch to the clock source specified by 'NOSC' */
	__builtin_write_OSCCONH(0x03);
	__builtin_write_OSCCONL(0x01);
	
	/* wait for clock to stabilize: Primary Oscillator with PLL module (XTPLL, HSPLL))*/
	while (OSCCONbits.COSC != 0b011)
	  ;
	
	/* wait for PLL to lock: PLL module is in lock, PLL start-up timer is satisfied */
	while (OSCCONbits.LOCK != 1)
	  ;
}

/** HALTs if a memory error is detected */
static void memtest(void)
{
#define BIT_PATTERN_1(x)                        \
  ((unsigned short)(x) % 4 ? 0xAA55 : 0x55AA)

#define BIT_PATTERN_2(x)                        \
  ((unsigned short)(x) % 4 ? 0x55AA : 0xAA55)

  /* This memory check covers also the final locations of the BL
     memory, to ensure slave index will be stored in a fully
     functional memory. */
#define __MEMCHK_BASE_PTR ((unsigned short *)(__SLAVE_INDEX_ADDR))
#define __MEMCHK_END_PTR  ((unsigned short *)(__APPL_DATA_END ))
  /* -- end of local macros ------------------------------------------------- */

  //unsigned short *p;

  /* Memory check, bit pattern 1, fwd sweeping */
  //for (p = __MEMCHK_BASE_PTR; p < __MEMCHK_END_PTR; ++ p) {
  //    (*p) = BIT_PATTERN_1(p);
  //}

  //for (p = __MEMCHK_BASE_PTR; p < __MEMCHK_END_PTR; ++ p) {
  //  if ((*p) != BIT_PATTERN_1(p))
  //    HALT();
  //}

  /* Memory check, bit pattern 2, fwd sweeping */
  //for (p = __MEMCHK_BASE_PTR; p < __MEMCHK_END_PTR; ++ p) {
  //  (*p) = BIT_PATTERN_2(p);
  //}

  //for (p = __MEMCHK_BASE_PTR; p < __MEMCHK_END_PTR; ++ p) {
  //  if ((*p) != BIT_PATTERN_2(p))
  //    HALT();
  //}
 /* memtest() */
}
//static void jump_to_appl()
void jump_to_appl()
{
  /* Write slave address into reserved data location before
   * relinquishing control to the application. */
  slave_index = (unsigned short) BL_slave_id ;

  /* Use standard vector table */
  INTCON2bits.AIVTEN = 0;

  /* jump to app code, won't return */
  __asm__ volatile ("goto " __APPL_GOTO_ADDR);
}

int main(void)
{
	/* Hardware initialization */
	hw_init();

	/* Memory test */
	memtest();

	/* BootLoader app initialization */
	BL_Init();

	/* Compare calculated CRC16 for BootLoader application executable
	* with the expected value, stored at @ptrBLCRCFlash. This check is
	*only possible when running the release build. */
	/*
	#ifndef __DEBUG
		if ((*ptrBLCRCFlash) != BL_CRCareaFlash(__BL_CODE_BASE,
                                          BYTES2WORDS(__BL_CODE_END -
                                                      __BL_CODE_BASE), 0))
    HALT();
	#endif
	*/
	BL_inputManager();
	// Lettura dei Dip Switch
	BL_slave_id = 0x00FF & BL_DigInStatus.byte;
	//BL_slave_id = 43;

    ENABLE_WDT();

	do {
		/* Kick the dog */
		ClrWdt();

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

  return 0; /* unreachable, just get rid of compiler warning */
} /* end main */
