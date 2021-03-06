/**/
/*============================================================================*/
/**
**      @file     gestIO.c
**
**      @brief     Campionamento ingressi digitali
**
**      @version   Alfa HUTBRD
**/
/*============================================================================*/
/**/

#include "gestIO.h"
#include "p24FJ64GA704.h"
#include "timerMg.h"

#define FILTER_WINDOW           5
//#define INPUT_ARRAY				4
#define INPUT_ARRAY				16
#define FILTER_WINDOW_LENGTH    (FILTER_WINDOW-1)
#define FILTER_WINDOW_LOOP      (FILTER_WINDOW-2)
#define MAX_CHANGE              (FILTER_WINDOW/2)
#define MIN_COUNT               (FILTER_WINDOW*3/4)
#define ERRORE_FILTRO 	        2
#define LOW_FILTER              0
#define HIGH_FILTER             1
#define COUNT_RESET             0
#define MASK_FILTER_OFF         0x0000

const unsigned char MASK_BIT_8[]={0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80};
const unsigned short MASK_BIT_16[]={0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80,0x100,0x200,0x400,0x800,0x1000,0x2000,
		0x4000,0x8000};

DigInStatusType DigInStatus, DigInNotFiltered;

static unsigned char  n_filter;
static unsigned char zero_counter, one_counter, ChangeStatus, Out_Status;
static signed char index_0, index_1;
static unsigned char DummyOutput_low, DummyOutput_high, shift;
static unsigned char  FILTRAGGIO_LOW[FILTER_WINDOW];
static unsigned char  FILTRAGGIO_HIGH[FILTER_WINDOW];
static DigInStatusType OutputFilter;


static unsigned short FilterSensorInput(DigInStatusType InputFilter);
void readIn(void);


void initIO(void)
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
}

void gestioneIO(void)
{
	// Check the IO value
	if (StatusTimer(T_READ_IO)==T_HALTED)
	{
		StartTimer(T_READ_IO);

	}

	if (StatusTimer(T_READ_IO)==T_ELAPSED)
	{
		StartTimer(T_READ_IO);
		readIn();
		DigInStatus.word=FilterSensorInput(DigInNotFiltered);
	}

}

static unsigned short  FilterSensorInput(DigInStatusType InputFilter)
	
/*=====================================================================*//**
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
{
  unsigned char temp_uc;
  signed char temp_sc;

  /* n_filter = Finestra campioni del filtro */
  if (n_filter < FILTER_WINDOW_LENGTH)
  {
    n_filter++;
  }
  else
  {
    n_filter = 0;
  }

  FILTRAGGIO_LOW[n_filter] = InputFilter.byte.low;
  FILTRAGGIO_HIGH[n_filter] = InputFilter.byte.high;

  for(temp_uc = 0 ; temp_uc < (INPUT_ARRAY / 2); temp_uc++) // INPUT_ARRAY = N� di ingressi da filtrare
  {
    shift = 0x1 << temp_uc;

    //ByteLow
    for(temp_sc = FILTER_WINDOW_LOOP; temp_sc >= 0; temp_sc--)
    {
      //Indice 0
      index_0 = n_filter - temp_sc;
      if (index_0 < 0)
      {
        index_0 += FILTER_WINDOW;
      }
      //Indice 1
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
    one_counter = COUNT_RESET;
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

    /*Byte High*/
    for(temp_sc = FILTER_WINDOW_LOOP; temp_sc >= 0 ; temp_sc--)
    {
      /*Indice 0*/
      index_0 = n_filter - temp_sc;
      if (index_0 < 0)
      {
        index_0 += FILTER_WINDOW;
      }
      /*Indice 1*/
      index_1 = n_filter - temp_sc - 1;
      if (index_1 < 0)
      {
        index_1 += FILTER_WINDOW;
      }

      if ( (FILTRAGGIO_HIGH[index_0] ^ FILTRAGGIO_HIGH[index_1]) & shift)
      {
        ChangeStatus++;
      }

      if (FILTRAGGIO_HIGH[index_0] & shift)
      {
        one_counter++;
      }
      else
      {
        zero_counter++;
      }

      if (temp_sc == 0)
      {
        if (FILTRAGGIO_HIGH[index_1] & shift)
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
    one_counter = COUNT_RESET;
    ChangeStatus = COUNT_RESET;

    /* Segnale d'ingresso filtrato */
    if (Out_Status != ERRORE_FILTRO)
    {
      if (!temp_uc)
      {
        DummyOutput_high = Out_Status;
      }
      else
      {
        DummyOutput_high |= (Out_Status << temp_uc);
      }
    }
  }

  OutputFilter.byte.low = DummyOutput_low;
  OutputFilter.byte.high = DummyOutput_high;

  return (OutputFilter.word);

} /*end FilterSensorInput*/

unsigned char getWaterLevel(void)
{
	return DigInStatus.Bit.StatusType0;
}

void readIn(void)
{	
	DigInNotFiltered.Bit.StatusType0 = LEVEL;
	
	DigInNotFiltered.Bit.StatusType1 = SW1;
	DigInNotFiltered.Bit.StatusType2 = SW2;
	DigInNotFiltered.Bit.StatusType3 = SW3;
	DigInNotFiltered.Bit.StatusType4 = SW4;
	DigInNotFiltered.Bit.StatusType5 = SW5;
	DigInNotFiltered.Bit.StatusType6 = SW6;
}

