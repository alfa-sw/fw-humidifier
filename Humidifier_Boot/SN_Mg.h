/**/
/*============================================================================*/
/**
**      @file    SN_Mg.h
**
**      @version Alfa Color Tester
**/
/*============================================================================*/
/**/
#ifndef __SN_MG_H__
#define __SN_MG_H__

/*===== MACRO LOCALI =========================================================*/
#define SNLEN  ((BYTE)0x06)   /* hw silicon serial number length */

/*===== PROTOTIPI FUNZIONI ==================================================*/
extern BOOL BL_BHwSnRead(void);
extern void BL_ReadSSN (void);
extern unsigned char BL_sn_arr[SNLEN];
extern void BL_SSN_GetValue(unsigned char * number_buffer);

#endif /* __SN_MG_H__ */
