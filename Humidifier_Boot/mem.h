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
#define __BL_CODE_BASE (0x0000L)
#define __BL_CODE_END  (0x1FFEL)

#define __APPL_CODE_BASE (0x2000L)
#define __APPL_GOTO_ADDR "0x2004"

#define __BL_SW_VERSION (__BL_CODE_END - 0x4)
#define __BL_CODE_CRC   (__BL_CODE_END)

/* -- Data memory macros ----------------------------------------------------- */

#define __APPL_DATA_BASE (0x1000)

#define __APPL_DATA_END  (0x4800)

/* This location is used to forward the 485 index to the application. */
#define __SLAVE_INDEX_ADDR (__APPL_DATA_BASE - 0x2)

/* -- Interrupt handlers ----------------------------------------------------- */
#define  __APPL_T1     (__APPL_CODE_BASE + 0x1A)  // 0x201A 
#define  __APPL_U1RX1  (__APPL_CODE_BASE + 0x24)  // 0x2024 
#define  __APPL_U1TX1  (__APPL_CODE_BASE + 0x2E)  // 0x202E
#define  __APPL_MI2C1  (__APPL_CODE_BASE + 0x38)  // 0x2038 
#endif /* MEM_H_DEFINED */
