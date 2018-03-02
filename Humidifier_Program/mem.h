/********************************************************************************
 **
 **      Filename     : mem.h
 **
 **      Description  : Memory addresses header file
 **
 **   ===========================================================================
 */
#ifndef MEM_H_DEFINED
#define MEM_H_DEFINED

/* -- Program memory macros -------------------------------------------------- */
#define __APPL_CODE_BASE (0x2000L)
#define __APPL_CODE_END  (0xAF00L)

#define __APPL_CODE_CRC   (__APPL_CODE_BASE + 0x2)
#define __APPL_CODE_BEGIN (__APPL_CODE_BASE + 0x4)

/* Interrupt vector addresses */
#define  __APPL_T1     (__APPL_CODE_BASE + 0x1A)  // 0x201A 
#define  __APPL_U1RX1  (__APPL_CODE_BASE + 0x24)  // 0x2024 
#define  __APPL_U1TX1  (__APPL_CODE_BASE + 0x2E)  // 0x202E
#define  __APPL_MI2C1  (__APPL_CODE_BASE + 0x38)  // 0x2038 
#define  __APPL_SPI1   (__APPL_CODE_BASE + 0x42)  // 0x2042
#define  __APPL_SPI1TX (__APPL_CODE_BASE + 0x4C)  // 0x204C
#define  __APPL_SPI1RX (__APPL_CODE_BASE + 0x56)  // 0x2056
/* -- Data memory macros -------------------------------------------------- */
#define __APPL_DATA_BASE 0x1010
/* Data dwelling in the BL memory, not altered by app init */
#define __BL_TEST_RESULTS_ADDR (__APPL_DATA_BASE - 0x12)

#define __BOOT_GOTO_ADDR "0x0204"

#define __BL_CODE_END  (0x1FFEL)
#define __BL_SW_VERSION (__BL_CODE_END - 0x4)

#define SLAVE_ADDR() (*(PtrTestResults))

//extern volatile const unsigned short *PtrTestResults;

#endif /* MEM_H_DEFINED */
