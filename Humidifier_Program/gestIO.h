
#ifndef _GEST_IO_H
#define _GEST_IO_H

#define INPUT 1
#define OUTPUT 0

#define UART_DE  PORTBbits.RB0  // Output UART
#define TMP_RESET PORTBbits.RB1 // Output SEMSOR RESET
#define TMP_ALERT PORTBbits.RB2 // Input SEMSOR ALERT
#define PUMP      PORTBbits.RB3 // Output PUMP
#define NEB       PORTBbits.RB4 // Output NEBULIZER
#define LEVEL     PORTBbits.RB7 // Input LEVEL
#define I2C0_SDL  PORTBbits.RB8 // Ouput I2C
#define I2C0_SDA  PORTBbits.RB9 // Input I2C
#define SPI_SS    PORTBbits.RB10// Output SPI
#define SPI_SCK   PORTBbits.RB11// Output SPI
#define SPI_SD0   PORTBbits.RB12// Output SPI
#define SPI_SDI   PORTBbits.RB13// Input SPI
#define UART_TX   PORTBbits.RB14// Output UART
#define UAR_RX    PORTBbits.RB15// Input UART

#define SW1	PORTCbits.RC0  // Input Dip Switch 1
#define SW2	PORTCbits.RC1  // Input Dip Switch 2
#define SW3	PORTCbits.RC2  // Input Dip Switch 3
#define SW4	PORTCbits.RC3  // Input Dip Switch 4
#define SW5	PORTCbits.RC4  // Input Dip Switch 5
#define SW6 PORTCbits.RC5  // Input Dip Switch 6

#define AN0  PORTCbits.RA0 // Input Analog 0
#define AN1	 PORTCbits.RA1 // Input Analog 1
#define OSC0 PORTCbits.RA2 // Input Oscillator 1
#define OSC1 PORTCbits.RA3 // Input Oscillator 0
#define LED  PORTCbits.RA8 // Output LED

typedef union {
  unsigned char bytes[2];
  unsigned short word;

  struct {
    /* User-operated inputs */
    unsigned short  StatusType0       : 1;
    unsigned short  StatusType1       : 1;
    unsigned short  StatusType2       : 1;
    unsigned short  StatusType3       : 1;
    unsigned short  StatusType4       : 1;
    unsigned short  StatusType5       : 1;
    unsigned short  StatusType6       : 1;
    unsigned short  StatusType7       : 1;
    unsigned short  StatusType8       : 1;
    unsigned short  StatusType9       : 1;
    unsigned short  StatusType10       : 1;
    unsigned short  StatusType11       : 1;
    unsigned short  StatusType12       : 1;
    unsigned short  StatusType13       : 1;
    unsigned short  StatusType14       : 1;
    unsigned short  StatusType15       : 1;
  } Bit;

  struct {
    unsigned char low;
    unsigned char high;
  } byte;
} DigInStatusType;


extern void initIO(void);
extern void gestioneIO(void);
extern unsigned char getWaterLevel(void);

#endif

