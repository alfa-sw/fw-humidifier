/**/
/*============================================================================*/
/**
**      @file      MAIN.C
**
**      @brief     Main
**
**      @version   Alfa HUTBRD
**/
/*============================================================================*/
/**/

/* ===== SETTING CONFIGURATION BITS================================================== */
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
//#pragma config AIVTDIS = ON             // Alternate Interrupt Vector Table bit (Enabled AIVT)
#pragma config AIVTDIS = OFF             // Alternate Interrupt Vector Table bit (Enabled AIVT)

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

// include file definition
#include <xc.h>
#include "p24FJ64GA704.h"
#include "timerMg.h"
#include "gestio.h"
#include "statusManager.h"
#include "mem.h"
#include "serialcom.h"
#include "ram.h"
#include "define.h"
#include "i2c1.h"
#include "gestIO.h"
#include "spi.h"

volatile const unsigned short *PtrTestResults = (unsigned short *) (__BL_TEST_RESULTS_ADDR);
volatile const unsigned long *BootPtrTestResults = (unsigned long *) (__BL_SW_VERSION);
// -----------------------------------------------------------------------------
//                      APPLICATION PROGRAM Service Routine
void APPLICATION_T1_InterruptHandler(void);
void APPLICATION_U1TX_InterruptHandler(void);
void APPLICATION_U1RX_InterruptHandler(void);
void APPLICATION_MI2C1_InterruptHandler(void);
void APPLICATION_SPI1_InterruptHandler(void);
void APPLICATION_SPI1TX_InterruptHandler(void);
void APPLICATION_SPI1RX_InterruptHandler(void);

void SPI1_InterruptHandler(void);
void SPI1TX_InterruptHandler(void);
void SPI1RX_InterruptHandler(void);

void Pippo(void);
// -----------------------------------------------------------------------------
/** T Y P E D E F S ******************************************************************* */
#if defined NO_BOOTLOADER

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
} DigInMicroSwitch;

static DigInMicroSwitch DigInMSwitch;

#else
#endif

/*
**=============================================================================
**
**      Oggetto        : Funzione di servizio degli Interrupt NON usati dal 
**                       Programma Applicativo
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

int main(void)
{
    uint8_t stato_led;

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
	InitTMR();
	initIO();
    INTERRUPT_Initialize();
	initStatusManager();
	initParam();
	initSerialCom();
    I2C1_Initialize();
    SPI1_Initialize();
    
#if defined NO_BOOTLOADER
  // if NON HARDCODED address is defined and BootLoader is NOT present, 
  // Slave Addres is read directly from dip switches)
  if (slave_id == UNIVERSAL_ID) {  
	/* Read 485 address bits from dip-switch on S1 */
	DigInMSwitch.Bit.StatusType0 = ~SW3;
	DigInMSwitch.Bit.StatusType1 = ~SW2;
	DigInMSwitch.Bit.StatusType2 = ~SW1;
	DigInMSwitch.Bit.StatusType3 = ~SW4;
	DigInMSwitch.Bit.StatusType4 = ~SW5;
	DigInMSwitch.Bit.StatusType5 = ~SW6;
	DigInMSwitch.Bit.StatusType6 = 0;
	DigInMSwitch.Bit.StatusType7 = 0;

  	slave_id = 0x00FF & DigInMSwitch.byte;	
	//slave_id = 43;  
	}
#else
  // if NON HARDCODED address is defined and BootLoader is present, Slave Addres is read from BootLoader (= from dip switches)
 if (slave_id == UNIVERSAL_ID)
	slave_id = SLAVE_ADDR();
#endif

#ifndef DEBUG_SLAVE
    /* enable 16msec watchdog */
    ENABLE_WDT();
#else
#endif	

    StartTimer(T_LED_DURATION_ON);
    StartTimer(T_ERROR_STATUS);
    while (1)
	{
#ifndef DEBUG_SLAVE
        /* kicking the dog ;-) */
        ClrWdt();
#else
#endif			
        // main loop
		humidifierStatusManager();
		TimerMg();
		gestioneIO();
		serialCommManager();
        I2C_Manager();
        SPI_Manager();
        // LED management
        // ---------------------------------------------------------------------
        if (isColorCmdSetupOutput() )
        {
            if (PeripheralAct.Peripheral_Types.humidifier_20_led == ON) 
            {
                Status.level = HUMIDIFIER_PAR_RX;
                StopHumidifier();
                NextStatus.level = HUMIDIFIER_NEBULIZER_PUMP_LED_RISCALDATORE_ON_ST;
                    
                if (AnalyzeSetupOutputs() == FALSE)
                    HumidifierAct.command.cmd = CMD_IDLE;
                else
                {        
                    if (PeripheralAct.Action == OUTPUT_ON) 
                    {
                        LED = ON;
                        HumidifierAct.Led_state = ON;
                    }     
                    else 
                    {
                        LED = OFF;
                        HumidifierAct.Led_state = OFF;
                    } 
                    HumidifierAct.command.cmd = CMD_IDLE;
                }
            }	            
        }        
        // LED ON - OFF
       if (StatusTimer(T_LED_DURATION_ON) == T_ELAPSED)
        {
            StopTimer(T_LED_DURATION_ON);
            StartTimer(T_LED_DURATION_ON);
            if (stato_led == OFF)
            {
                stato_led = ON;
                LED = ON;
                HumidifierAct.Led_state = ON;
//                AIR_PUMP_ON();
            }
            else
            {
                stato_led = OFF;
                LED = OFF;
                HumidifierAct.Led_state = OFF;                
//                AIR_PUMP_OFF();
            }                
        }
        // ---------------------------------------------------------------------                    
 	}
}

// -----------------------------------------------------------------------------
//                      APPLICATION PROGRAM Service Routine
// ISR used when BOOT and APPLICATION PROGRAMS are both present
#ifndef NO_BOOTLOADER

// Timer 1 Interrupt handler 
void __attribute__((address(__APPL_T1))) APPLICATION_T1_InterruptHandler(void)
{
    T1_InterruptHandler();
}
// UART1 RX Interrupt handler 
void __attribute__((address(__APPL_U1RX1))) APPLICATION_U1RX_InterruptHandler(void)
{
    U1RX_InterruptHandler();
}
// UART1 TX Interrupt handler 
void __attribute__((address(__APPL_U1TX1))) APPLICATION_U1TX_InterruptHandler(void)
{
    U1TX_InterruptHandler();
}
// I2C1 Interrupt handler 
void __attribute__((address(__APPL_MI2C1))) APPLICATION_MI2C1_InterruptHandler(void)
{
    MI2C1_InterruptHandler();
}
// SPI1 Interrupt handler 
void __attribute__((address(__APPL_SPI1))) APPLICATION_SPI1_InterruptHandler(void)
{
    SPI1_InterruptHandler();
}
// SPI1TX Interrupt handler 
void __attribute__((address(__APPL_SPI1TX))) APPLICATION_SPI1TX_InterruptHandler(void)
{
    SPI1TX_InterruptHandler();
}
// SPI1RX Interrupt handler 
void __attribute__((address(__APPL_SPI1RX))) APPLICATION_SPI1RX_InterruptHandler(void)
{
   SPI1RX_InterruptHandler();
}
// -----------------------------------------------------------------------------
// ISR used when only Application Program runs
#else
// Default Interrupt
void __attribute__((__interrupt__,auto_psv)) _DefaultInterrupt(void);
void _DefaultInterrupt(void)
{
    Nop();
    Nop();
    while(1);
}
void __attribute__((__interrupt__,auto_psv)) _T1Interrupt(void)
{
   T1_InterruptHandler();
}
void __attribute__((__interrupt__, no_auto_psv)) _U1RXInterrupt(void)
{
   U1RX_InterruptHandler();
}
void __attribute__((__interrupt__, no_auto_psv)) _U1TXInterrupt(void)
{
   U1TX_InterruptHandler();
}
void __attribute__ ( ( interrupt, no_auto_psv ) ) _MI2C1Interrupt ( void )
{
   MI2C1_InterruptHandler();
} 
void __attribute__ ( ( interrupt, no_auto_psv ) ) _SPI1Interrupt ( void )
{
    SPI1_InterruptHandler();
}
void __attribute__ ( ( interrupt, no_auto_psv ) ) _SPI1TXInterrupt ( void )
{
    SPI1TX_InterruptHandler();
}
void __attribute__ ( ( interrupt, no_auto_psv ) ) _SPI1RXInterrupt ( void )
{
   SPI1RX_InterruptHandler();
}
// -----------------------------------------------------------------------------
#endif
//                      APPLICATION PROGRAM Service Routine NOT USED
// SPI1 GENERAL Interrupt handler 
void SPI1_InterruptHandler(void)
{
    Pippo();
}
// SPI1TX Interrupt handler 
void SPI1TX_InterruptHandler(void)
{
    Pippo();
}
// SPI1RX Interrupt handler 
void SPI1RX_InterruptHandler(void)
{
    Pippo();
}
// -----------------------------------------------------------------------------