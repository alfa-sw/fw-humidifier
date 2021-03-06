#include "typedef.h"

#ifdef RAM_EXTERN_DISABLE
#   define RAM_EXTERN
#else
#   define RAM_EXTERN extern
#endif
/*****************************************************************************/

/* ***************************** ADDRESS SETUP ***************************** */
#if ! defined RAM_EXTERN_DISABLE
/* decl */ extern unsigned char slave_id;

/* Assigning a fixed ID to begin with */
#else
/* def */  unsigned char slave_id

#    if defined COLOR_1
= B1_BASE_ID;

#  elif defined COLOR_2
= B2_BASE_ID;

#  elif defined COLOR_3
= B3_BASE_ID;

#  elif defined COLOR_4
= B4_BASE_ID;

#  elif defined COLOR_5
= B5_BASE_ID;

#  elif defined COLOR_6
= B6_BASE_ID;

#  elif defined COLOR_7
= B7_BASE_ID;

#  elif defined COLOR_8
= B8_BASE_ID;

#  elif defined COLOR_9
= C1_COLOR_ID;

#  elif defined COLOR_10
= C2_COLOR_ID;

#  elif defined COLOR_11
= C3_COLOR_ID;

#  elif defined COLOR_12
= C4_COLOR_ID;

#  elif defined COLOR_13
= C5_COLOR_ID;

#  elif defined COLOR_14
= C6_COLOR_ID;

#  elif defined COLOR_15
= C7_COLOR_ID;

#  elif defined COLOR_16
= C8_COLOR_ID;

#  elif defined COLOR_17
= C9_COLOR_ID;

#  elif defined COLOR_18
= C10_COLOR_ID;

#  elif defined COLOR_19
= C11_COLOR_ID;

#  elif defined COLOR_20
= C12_COLOR_ID;

#  elif defined COLOR_21
= C13_COLOR_ID;

#  elif defined COLOR_22
= C14_COLOR_ID;

#  elif defined COLOR_23
= C15_COLOR_ID;

#  elif defined COLOR_24
= C16_COLOR_ID;

#  elif defined AXIS_X
= MOVE_X_AXIS_ID;

#  elif defined AXIS_Y
= MOVE_Y_AXIS_ID;

#  elif defined CONTAINER_1
= STORAGE_CONTAINER1_ID;

#  elif defined CONTAINER_2
= STORAGE_CONTAINER2_ID;

#  elif defined CONTAINER_3
= STORAGE_CONTAINER3_ID;

#  elif defined CONTAINER_4
= STORAGE_CONTAINER4_ID;

#  elif defined COVER_1
= PLUG_COVER_1_ID;

#  elif defined COVER_2
= PLUG_COVER_2_ID;

#  elif (defined AUTOCAP_LIFTER || defined AUTOCAP_NOLIFTER)
= AUTOCAP_ID;
# elif defined _SGABELLO
= SGABELLO;
# elif defined _HUMIDIFIER
= HUMIDIFIER;

#  else
//#  error Universal address not yet supported.
= UNIVERSAL_ID; /* 0 is the universal address, the actuator will take
                 * the address from the dip-switches in R1. R0 does
                 * not support universal addresses.*/
#  endif

#endif /* ! defined RAM_EXTERN_DISABLE */

RAM_EXTERN status_t Status;
RAM_EXTERN status_t OldStatus;
RAM_EXTERN PeripheralAct_t PeripheralAct;
RAM_EXTERN HumidifierAct_t HumidifierAct;


