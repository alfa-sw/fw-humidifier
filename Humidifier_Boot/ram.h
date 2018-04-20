/**/
/*============================================================================*/
/**
**      @file    ram.h
**
**      @brief   Global data module
**/
/*============================================================================*/
/**/
#include "define.h"
#include "mem.h"
#include "serialCom.h"

#ifdef RAM_EXTERN_DISABLE
#define RAM_EXTERN
#else
#define RAM_EXTERN extern
#endif
/*****************************************************************************/

RAM_EXTERN char BL_StandAlone
#ifdef RAM_EXTERN_DISABLE
 = TRUE
#endif
;

RAM_EXTERN Stato BLState;

/* Reserved 2 bytes for communication with the application. */
RAM_EXTERN volatile short slave_index __attribute__((space(data),
                                                     address(__SLAVE_INDEX_ADDR)));

/* Reserved 2 bytes for set which program is active: BOOT or APPLICATION PROGRAM */
RAM_EXTERN volatile short program_active __attribute__((space(data),
                                                     address(__PROGRAM_ACTIVE_ADDR)));
													 
RAM_EXTERN volatile long boot_fw_version __attribute__((space(data),
                                                     address(__BL_SW_VERSION)));													 

#ifdef HARDCODED_ADDRESS
/*B1_BASE*/#ifdef B1_BASE
/*B1_BASE*/RAM_EXTERN unsigned char BL_slave_id
/*B1_BASE*/#ifdef RAM_EXTERN_DISABLE
/*B1_BASE*/ = B1_BASE_ID
/*B1_BASE*/#endif
/*B1_BASE*/;
/*B1_BASE*/#endif

/*B2_BASE*/#ifdef B2_BASE
/*B2_BASE*/RAM_EXTERN unsigned char BL_slave_id
/*B2_BASE*/#ifdef RAM_EXTERN_DISABLE
/*B2_BASE*/ = B2_BASE_ID
/*B2_BASE*/#endif
/*B2_BASE*/;
/*B2_BASE*/#endif

/*B3_BASE*/#ifdef B3_BASE
/*B3_BASE*/RAM_EXTERN unsigned char BL_slave_id
/*B3_BASE*/#ifdef RAM_EXTERN_DISABLE
/*B3_BASE*/ = B3_BASE_ID
/*B3_BASE*/#endif
/*B3_BASE*/;
/*B3_BASE*/#endif

/*B4_BASE*/#ifdef B4_BASE
/*B4_BASE*/RAM_EXTERN unsigned char BL_slave_id
/*B4_BASE*/#ifdef RAM_EXTERN_DISABLE
/*B4_BASE*/ = B4_BASE_ID
/*B4_BASE*/#endif
/*B4_BASE*/;
/*B4_BASE*/#endif

/*B5_BASE*/#ifdef B5_BASE
/*B5_BASE*/RAM_EXTERN unsigned char BL_slave_id
/*B5_BASE*/#ifdef RAM_EXTERN_DISABLE
/*B5_BASE*/ = B5_BASE_ID
/*B5_BASE*/#endif
/*B5_BASE*/;
/*B5_BASE*/#endif


/*B6_BASE*/#ifdef B6_BASE
/*B6_BASE*/RAM_EXTERN unsigned char BL_slave_id
/*B6_BASE*/#ifdef RAM_EXTERN_DISABLE
/*B6_BASE*/ = B6_BASE_ID
/*B6_BASE*/#endif
/*B6_BASE*/;
/*B6_BASE*/#endif

/*B7_BASE*/#ifdef B7_BASE
/*B7_BASE*/RAM_EXTERN unsigned char BL_slave_id
/*B7_BASE*/#ifdef RAM_EXTERN_DISABLE
/*B7_BASE*/ = B7_BASE_ID
/*B7_BASE*/#endif
/*B7_BASE*/;
/*B7_BASE*/#endif

/*B8_BASE*/#ifdef B8_BASE
/*B8_BASE*/RAM_EXTERN unsigned char BL_slave_id
/*B8_BASE*/#ifdef RAM_EXTERN_DISABLE
/*B8_BASE*/ = B8_BASE_ID
/*B8_BASE*/#endif
/*B8_BASE*/;
/*B8_BASE*/#endif

/*C1_COLOR*/#ifdef C1_COLOR
/*C1_COLOR*/RAM_EXTERN unsigned char BL_slave_id
/*C1_COLOR*/#ifdef RAM_EXTERN_DISABLE
/*C1_COLOR*/ = C1_COLOR_ID
/*C1_COLOR*/#endif
/*C1_COLOR*/;
/*C1_COLOR*/
/*C1_COLOR*/#endif

/*C2_COLOR*/#ifdef C2_COLOR
/*C2_COLOR*/RAM_EXTERN unsigned char BL_slave_id
/*C2_COLOR*/#ifdef RAM_EXTERN_DISABLE
/*C2_COLOR*/ = C2_COLOR_ID
/*C2_COLOR*/#endif
/*C2_COLOR*/;
/*C2_COLOR*/#endif

/*C3_COLOR*/#ifdef C3_COLOR
/*C3_COLOR*/RAM_EXTERN unsigned char BL_slave_id
/*C3_COLOR*/#ifdef RAM_EXTERN_DISABLE
/*C3_COLOR*/ = C3_COLOR_ID
/*C3_COLOR*/#endif
/*C3_COLOR*/;
/*C3_COLOR*/#endif

/*C4_COLOR*/#ifdef C4_COLOR
/*C4_COLOR*/RAM_EXTERN unsigned char BL_slave_id
/*C4_COLOR*/#ifdef RAM_EXTERN_DISABLE
/*C4_COLOR*/ = C4_COLOR_ID
/*C4_COLOR*/#endif
/*C4_COLOR*/;
/*C4_COLOR*/#endif

/*C5_COLOR*/#ifdef C5_COLOR
/*C5_COLOR*/RAM_EXTERN unsigned char BL_slave_id
/*C5_COLOR*/#ifdef RAM_EXTERN_DISABLE
/*C5_COLOR*/ = C5_COLOR_ID
/*C5_COLOR*/#endif
/*C5_COLOR*/;
/*C5_COLOR*/#endif

/*C6_COLOR*/#ifdef C6_COLOR
/*C6_COLOR*/RAM_EXTERN unsigned char BL_slave_id
/*C6_COLOR*/#ifdef RAM_EXTERN_DISABLE
/*C6_COLOR*/ = C6_COLOR_ID
/*C6_COLOR*/#endif
/*C6_COLOR*/;
/*C6_COLOR*/#endif

/*C7_COLOR*/#ifdef C7_COLOR
/*C7_COLOR*/RAM_EXTERN unsigned char BL_slave_id
/*C7_COLOR*/#ifdef RAM_EXTERN_DISABLE
/*C7_COLOR*/ = C7_COLOR_ID
/*C7_COLOR*/#endif
/*C7_COLOR*/;
/*C7_COLOR*/#endif

/*C8_COLOR*/#ifdef C8_COLOR
/*C8_COLOR*/RAM_EXTERN unsigned char BL_slave_id
/*C8_COLOR*/#ifdef RAM_EXTERN_DISABLE
/*C8_COLOR*/ = C8_COLOR_ID
/*C8_COLOR*/#endif
/*C8_COLOR*/;
/*C8_COLOR*/#endif

/*C9_COLOR*/#ifdef C9_COLOR
/*C9_COLOR*/RAM_EXTERN unsigned char BL_slave_id
/*C9_COLOR*/#ifdef RAM_EXTERN_DISABLE
/*C9_COLOR*/ = C9_COLOR_ID
/*C9_COLOR*/#endif
/*C9_COLOR*/;
/*C9_COLOR*/#endif

/*C10_COLOR*/#ifdef C10_COLOR
/*C10_COLOR*/RAM_EXTERN unsigned char BL_slave_id
/*C10_COLOR*/#ifdef RAM_EXTERN_DISABLE
/*C10_COLOR*/ = C10_COLOR_ID
/*C10_COLOR*/#endif
/*C10_COLOR*/;
/*C10_COLOR*/#endif

/*C11_COLOR*/#ifdef C11_COLOR
/*C11_COLOR*/RAM_EXTERN unsigned char BL_slave_id
/*C11_COLOR*/#ifdef RAM_EXTERN_DISABLE
/*C11_COLOR*/ = C11_COLOR_ID
/*C11_COLOR*/#endif
/*C11_COLOR*/;
/*C11_COLOR*/#endif

/*C12_COLOR*/#ifdef C12_COLOR
/*C12_COLOR*/RAM_EXTERN unsigned char BL_slave_id
/*C12_COLOR*/#ifdef RAM_EXTERN_DISABLE
/*C12_COLOR*/ = C12_COLOR_ID
/*C12_COLOR*/#endif
/*C12_COLOR*/;
/*C12_COLOR*/#endif

/*C13_COLOR*/#ifdef C13_COLOR
/*C13_COLOR*/RAM_EXTERN unsigned char BL_slave_id
/*C13_COLOR*/#ifdef RAM_EXTERN_DISABLE
/*C13_COLOR*/ = C13_COLOR_ID
/*C13_COLOR*/#endif
/*C13_COLOR*/;
/*C13_COLOR*/
/*C13_COLOR*/#endif

/*C14_COLOR*/#ifdef C14_COLOR
/*C14_COLOR*/RAM_EXTERN unsigned char BL_slave_id
/*C14_COLOR*/#ifdef RAM_EXTERN_DISABLE
/*C14_COLOR*/ = C14_COLOR_ID
/*C14_COLOR*/#endif
/*C14_COLOR*/;
/*C14_COLOR*/#endif

/*C15_COLOR*/#ifdef C15_COLOR
/*C15_COLOR*/RAM_EXTERN unsigned char BL_slave_id
/*C15_COLOR*/#ifdef RAM_EXTERN_DISABLE
/*C15_COLOR*/ = C15_COLOR_ID
/*C15_COLOR*/#endif
/*C15_COLOR*/;
/*C15_COLOR*/#endif

/*C16_COLOR*/#ifdef C16_COLOR
/*C16_COLOR*/RAM_EXTERN unsigned char BL_slave_id
/*C16_COLOR*/#ifdef RAM_EXTERN_DISABLE
/*C16_COLOR*/ = C16_COLOR_ID
/*C16_COLOR*/#endif
/*C16_COLOR*/;
/*C16_COLOR*/#endif

/*C17_COLOR*/#ifdef C17_COLOR
/*C17_COLOR*/RAM_EXTERN unsigned char BL_slave_id
/*C17_COLOR*/#ifdef RAM_EXTERN_DISABLE
/*C17_COLOR*/ = C17_COLOR_ID
/*C17_COLOR*/#endif
/*C17_COLOR*/;
/*C17_COLOR*/#endif

/*C18_COLOR*/#ifdef C18_COLOR
/*C18_COLOR*/RAM_EXTERN unsigned char BL_slave_id
/*C18_COLOR*/#ifdef RAM_EXTERN_DISABLE
/*C18_COLOR*/ = C18_COLOR_ID
/*C18_COLOR*/#endif
/*C18_COLOR*/;
/*C18_COLOR*/#endif

/*C19_COLOR*/#ifdef C19_COLOR
/*C19_COLOR*/RAM_EXTERN unsigned char BL_slave_id
/*C19_COLOR*/#ifdef RAM_EXTERN_DISABLE
/*C19_COLOR*/ = C19_COLOR_ID
/*C19_COLOR*/#endif
/*C19_COLOR*/;
/*C19_COLOR*/#endif

/*C20_COLOR*/#ifdef C20_COLOR
/*C20_COLOR*/RAM_EXTERN unsigned char BL_slave_id
/*C20_COLOR*/#ifdef RAM_EXTERN_DISABLE
/*C20_COLOR*/ = C20_COLOR_ID
/*C20_COLOR*/#endif
/*C20_COLOR*/;
/*C20_COLOR*/#endif

/*C21_COLOR*/#ifdef C21_COLOR
/*C21_COLOR*/RAM_EXTERN unsigned char BL_slave_id
/*C21_COLOR*/#ifdef RAM_EXTERN_DISABLE
/*C21_COLOR*/ = C21_COLOR_ID
/*C21_COLOR*/#endif
/*C21_COLOR*/;
/*C21_COLOR*/#endif

/*C22_COLOR*/#ifdef C22_COLOR
/*C22_COLOR*/RAM_EXTERN unsigned char BL_slave_id
/*C22_COLOR*/#ifdef RAM_EXTERN_DISABLE
/*C22_COLOR*/ = C22_COLOR_ID
/*C22_COLOR*/#endif
/*C22_COLOR*/;
/*C22_COLOR*/#endif

/*C23_COLOR*/#ifdef C23_COLOR
/*C23_COLOR*/RAM_EXTERN unsigned char BL_slave_id
/*C23_COLOR*/#ifdef RAM_EXTERN_DISABLE
/*C23_COLOR*/ = C23_COLOR_ID
/*C23_COLOR*/#endif
/*C23_COLOR*/;
/*C23_COLOR*/#endif

/*C24_COLOR*/#ifdef C24_COLOR
/*C24_COLOR*/RAM_EXTERN unsigned char BL_slave_id
/*C24_COLOR*/#ifdef RAM_EXTERN_DISABLE
/*C24_COLOR*/ = C24_COLOR_ID
/*C24_COLOR*/#endif
/*C24_COLOR*/;
/*C24_COLOR*/#endif


/*MOVE_X_AXIS*/#ifdef MOVE_X_AXIS
/*MOVE_X_AXIS*/RAM_EXTERN unsigned char BL_slave_id
/*MOVE_X_AXIS*/#ifdef RAM_EXTERN_DISABLE
/*MOVE_X_AXIS*/ = MOVE_X_AXIS_ID
/*MOVE_X_AXIS*/#endif
/*MOVE_X_AXIS*/;
/*MOVE_X_AXIS*/#endif

/*MOVE_Y_AXIS*/#ifdef MOVE_Y_AXIS
/*MOVE_Y_AXIS*/RAM_EXTERN unsigned char BL_slave_id
/*MOVE_Y_AXIS*/#ifdef RAM_EXTERN_DISABLE
/*MOVE_Y_AXIS*/ = MOVE_Y_AXIS_ID
/*MOVE_Y_AXIS*/#endif
/*MOVE_Y_AXIS*/;
/*MOVE_Y_AXIS*/#endif

/*STORAGE_CONTAINER1*/#ifdef STORAGE_CONTAINER1
/*STORAGE_CONTAINER1*/RAM_EXTERN unsigned char BL_slave_id
/*STORAGE_CONTAINER1*/#ifdef RAM_EXTERN_DISABLE
/*STORAGE_CONTAINER1*/ = STORAGE_CONTAINER1_ID
/*STORAGE_CONTAINER1*/#endif
/*STORAGE_CONTAINER1*/;
/*STORAGE_CONTAINER1*/#endif

/*STORAGE_CONTAINER2*/#ifdef STORAGE_CONTAINER2
/*STORAGE_CONTAINER2*/RAM_EXTERN unsigned char BL_slave_id
/*STORAGE_CONTAINER2*/#ifdef RAM_EXTERN_DISABLE
/*STORAGE_CONTAINER2*/ = STORAGE_CONTAINER2_ID
/*STORAGE_CONTAINER2*/#endif
/*STORAGE_CONTAINER2*/;
/*STORAGE_CONTAINER2*/#endif

/*STORAGE_CONTAINER3*/#ifdef STORAGE_CONTAINER3
/*STORAGE_CONTAINER3*/RAM_EXTERN unsigned char BL_slave_id
/*STORAGE_CONTAINER3*/#ifdef RAM_EXTERN_DISABLE
/*STORAGE_CONTAINER3*/ = STORAGE_CONTAINER3_ID
/*STORAGE_CONTAINER3*/#endif
/*STORAGE_CONTAINER3*/;
/*STORAGE_CONTAINER3*/#endif

/*STORAGE_CONTAINER4*/#ifdef STORAGE_CONTAINER4
/*STORAGE_CONTAINER4*/RAM_EXTERN unsigned char BL_slave_id
/*STORAGE_CONTAINER4*/#ifdef RAM_EXTERN_DISABLE
/*STORAGE_CONTAINER4*/ = STORAGE_CONTAINER4_ID
/*STORAGE_CONTAINER4*/#endif
/*STORAGE_CONTAINER4*/;
/*STORAGE_CONTAINER4*/#endif

/*PLUG_COVER_1*/#ifdef PLUG_COVER1
/*PLUG_COVER_1*/RAM_EXTERN unsigned char BL_slave_id
/*PLUG_COVER_1*/#ifdef RAM_EXTERN_DISABLE
/*PLUG_COVER_1*/ = PLUG_COVER_1_ID
/*PLUG_COVER_1*/#endif
/*PLUG_COVER_1*/;
/*PLUG_COVER_1*/#endif

/*PLUG_COVER_2*/#ifdef PLUG_COVER2
/*PLUG_COVER_2*/RAM_EXTERN unsigned char BL_slave_id
/*PLUG_COVER_2*/#ifdef RAM_EXTERN_DISABLE
/*PLUG_COVER_2*/ = PLUG_COVER_2_ID
/*PLUG_COVER_2*/#endif
/*PLUG_COVER_2*/;
/*PLUG_COVER_2*/#endif

/*AUTOCAP*/#ifdef AUTOCAP
/*AUTOCAP*/RAM_EXTERN unsigned char BL_slave_id
/*AUTOCAP*/#ifdef RAM_EXTERN_DISABLE
/*AUTOCAP*/ = AUTOCAP_ID
/*AUTOCAP*/#endif
/*AUTOCAP*/;
/*AUTOCAP*/#endif

/*CANLIFTER*/#ifdef CANLIFTER
/*CANLIFTER*/RAM_EXTERN unsigned char BL_slave_id
/*CANLIFTER*/#ifdef RAM_EXTERN_DISABLE
/*CANLIFTER*/ = CANLIFTER_ID
/*CANLIFTER*/#endif
/*CANLIFTER*/;
/*CANLIFTER*/#endif

/*HUMIDIFIER*/#ifdef HUMIDIFIER
/*HUMIDIFIER*/RAM_EXTERN unsigned char BL_slave_id
/*HUMIDIFIER*/#ifdef RAM_EXTERN_DISABLE
/*HUMIDIFIER*/ = HUMIDIFIER_ID
/*HUMIDIFIER*/#endif
/*HUMIDIFIER*/;
/*HUMIDIFIER*/#endif

#else
RAM_EXTERN unsigned char BL_slave_id;
#endif

RAM_EXTERN unsigned short Set_Reset;
