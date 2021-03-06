#ifndef _STATUS_MANAGER_H
#define _STATUS_MANAGER_H

enum {
	/* 0 */   HUMIDIFIER_INIT_ST=0,
	/* 1 */   HUMIDIFIER_READY_ST,
	/* 2 */   HUMIDIFIER_RUN_ST,
	/* 3 */   HUMIDIFIER_NEBULIZER_ON_ST,
	/* 4 */   HUMIDIFIER_PUMP_ON_ST,
	/* 5 */   HUMIDIFIER_NEBULIZER_PUMP_ON_ST,
	/* 6 */   HUMIDIFIER_PAR_RX,
	/* 7 */   

  /** Errors */
    /*  8 */   HUMIDIFIER_BAD_PAR_ERROR = 8,
    /*  9 */   HUMIDIFIER_TOO_LOW_WATER_LEVEL,
    /* 10 */   HUMIDIFIER_TOUT_ERROR,
    /* 11 */   HUMIDIFIER_RH_ERROR,
    /* 12 */   HUMIDIFIER_TEMPERATURE_ERROR,
};

enum {
	CMD_IDLE = 0x00,
	CMD_STOP = 0x01,
	CMD_SETUP_PARAM   = 0x02,
	CMD_SETUP_OUTPUT  = 0x04,
	CMD_STOP_PROCESDS = 0x08,	
  };

  
enum {
    CONTROLLO_PRESENZA = 0x01,
    SETUP_PARAMETRI_UMIDIFICATORE = 0x13,
    IMPOSTA_USCITE_UMIDIFICATORE  = 0x14
  };

enum {
  STEP_0,  STEP_1,  STEP_2,  STEP_3,  STEP_4,
  STEP_5,  STEP_6,  STEP_7,  STEP_8,  STEP_9,
  STEP_10, STEP_11, STEP_12, STEP_13, STEP_14,
  STEP_15, STEP_16, STEP_17, STEP_18, STEP_19,
  STEP_20, STEP_21, STEP_22, STEP_23, STEP_24,
  STEP_25, STEP_26, STEP_27, STEP_28, STEP_29,
  STEP_30, STEP_31, STEP_32, STEP_33, STEP_34,
  STEP_35, STEP_36, STEP_37, STEP_38, STEP_39,
  STEP_40, STEP_41, STEP_42, STEP_43, STEP_44,
  STEP_45, STEP_46, STEP_47, STEP_48, STEP_49,
};

extern void humidifierStatusManager(void);
extern void initStatusManager(void);
extern void initParam(void);
extern void StopHumidifier(void);
extern int AnalyzeParam(void);
extern int AnalyzeSetupOutputs(void);
extern int AcquireTemperature(unsigned char Temp_Type, unsigned long *Temp);
extern int AcquireHumidityTemperature(unsigned char Hum_Type, unsigned long *Temp, unsigned long *Humidity);
extern void HumidifierProcessCalculation(unsigned long Multiplier, unsigned long RH, unsigned long Temperature, 
														unsigned long *Period, unsigned long *Pump_Duration, unsigned long *Neb_Duration);
#endif
