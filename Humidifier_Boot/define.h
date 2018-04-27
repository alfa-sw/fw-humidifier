/**/
/*============================================================================*/
/**
 **      @file    define.h
 **
 **      @brief   File di definizioni
  **/
/*============================================================================*/
/**/
#ifndef __DEFINE_H
#define __DEFINE_H

#include "Macro.h"

/* Parametri Flash */
#define FLASH_PAGE_WORD_LENGTH 1024
#define FLASH_PAGE_BYTE_LENGTH 1536

#define START_APPL_ADDRESS 0x00002000L
#define END_APPL_ADDRESS   0x0000AF00L

// 0 - 400 --> PG0 / 400 - 800 --> PG1 / 800 - C00 --> PG2 / C00 - 1000 --> PG3 / 1000 - 1400 --> PG4 / 1400 - 1800 --> PG5 / 1800 - 1C00 --> PG6
// 1C00 - 2000 --> PG7 / 2000 - 2400 --> PG8
#define EXPECTED_FIRST_PAGE_APPL (8) /* starts @ 0x2000 */
#define FIRST_PG_APPL (START_APPL_ADDRESS / FLASH_PAGE_WORD_LENGTH)
#if FIRST_PG_APPL != EXPECTED_FIRST_PAGE_APPL
#  warning First application page does not not match expected value. Check define.h
#endif
//#define FIRST_PG_APPL (8)

//#define EXPECTED_LAST_PAGE_APPL (43) /* ends @ 0xAF00 */
//#define LAST_PG_APPL  (END_APPL_ADDRESS / FLASH_PAGE_WORD_LENGTH)
//#if LAST_PG_APPL != EXPECTED_LAST_PAGE_APPL
//#  warning Last application page does not not match expected value. Check define.h
//#endif
#define LAST_PG_APPL  (41)

/******************************************************************************************/
/***************************************** Main *******************************************/
/******************************************************************************************/
#define BL_STAND_ALONE_CHECK            (START_APPL_ADDRESS + 4)
#define APPL_FLASH_MEMORY_ERASED_VALUE  0x00FFFFFFL

typedef enum {
  PROC_OK,             /* Procedura corretta      */
  PROC_ERROR,          /* Valore scritto e letto non coincidono  */
  PROC_PROTECTED,      /* La flash è protetta in scrittura       */
  PROC_ERROR_PROTECT,  /* La flash si trova nello stato protetto */
  PROC_ERROR_12V,
  PROC_ADDRESS_ERROR,  /* Indirizzo errato */
  PROC_MAX_FAILS,
  PROC_CHK_ERROR
} PROC_RES;

typedef struct __attribute__ ((packed)) {
  unsigned char livello;
  unsigned char fase;
  unsigned char step;
  PROC_RES ProcRes;
} Stato;

/* Enum per BLState.livello */
enum {
  POWER_OFF = 0,
  INIT,
  USB_CONNECT_EXECUTION,  /* supported if BOOTLOADER_USB macro is uncommented */
  UART_FW_UPLOAD,         /* firmware upload */
  UART_FW_UPLOAD_FAILED,  /* firmware upload failed */
};

/* Enum per BLState.step per BLState.fase=SAT_FW_UPLOAD --- Utilizzati
   sia per USB che per comunicazione seriale */
enum {
  ERASE_DEVICE = 0,
  WAIT_DATA_PACKET,
  PROGRAM_DEVICE,
  PROGRAM_END,
  GET_DATA,
  RESET_SYSTEM,
  DO_RESET
};

#define INPUT 1
#define OUTPUT 0

#define BOOT                0
#define APPLICATION_PROGRAM 1

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

#endif /* __DEFINE_H */
