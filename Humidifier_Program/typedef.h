#ifndef _TYPEDEF_H_
#define _TYPEDEF_H_

/*========================== DEFINIZIONI GENERALI DI TIPO ================== */

/*!  Union   */
typedef union __attribute__ ((packed))
{
  unsigned short allflags;
  struct
  {
    unsigned short unused:16;
  } bit;
} statusFlags_t;


typedef union __attribute__ ((packed))
{
  unsigned short uword;
  signed short   sword;
  unsigned char  byte[2];
} unionWord_t;


typedef union __attribute__ ((packed))
{
  unsigned long  udword;
  signed long    sdword;
  unsigned short word[2];
  unsigned char  byte[4];
} unionDWord_t;

typedef struct
{
  unsigned char level;
  unsigned char phase;
  unsigned char step;
  unsigned char errorCode;
} status_t;

typedef struct
{
  unsigned char Action;
  union __attribute__ ((packed)) PeripheralTypes_t
  {
	unsigned char bytePeripheral;
	
	struct {
      unsigned char humidifier_20_nebulizer     : 1;
      unsigned char humidifier_20_pump			: 1;
      unsigned char humidifier_20_led			: 1;
      unsigned char humidifier_20_riscaldatore	: 1;
      unsigned char unused    					: 4;
	};	  
  } Peripheral_Types;
} PeripheralAct_t;

typedef struct
{
  unsigned char typeMessage;

  // humidifier command 
  union __attribute__ ((packed))
  {
    unsigned char cmd;
    struct
    {
      unsigned char stop      	: 1;
      unsigned char setup_param	: 1;
      unsigned char setup_output: 1;
      unsigned char stop_process: 1;
      unsigned char unused    	: 4;
    };
  } command;

  union __attribute__ ((packed)) HumidifierFlags_t
  {
    unsigned long allFlags;
    unsigned char byteFlags[4];
    unsigned short wordFlags[2];

    struct {
      /* octet 1 */
      unsigned char stopped                : 1;
      unsigned char ready                  : 1;
      unsigned char run			           : 1;
      unsigned char nebulizer_on           : 1;
      unsigned char pump_on                : 1;
      unsigned char nebulizer_pump_on      : 1;
	  unsigned char hum_20_par_rx		   : 1;
      unsigned char unused_byte_1          : 1;

      /* octet 2 */
	  unsigned char hum_20_bad_par_error   : 1;	  
	  unsigned char hum_20_too_low_water_level : 1;	  
      unsigned char tout_error             : 1;
      unsigned char hum_20_RH_error        : 1;
      unsigned char hum_20_Temperature_error   : 1;     
	  unsigned char unused_byte_2          : 3;

      /* octet 3 */
      unsigned char unused_byte_3          : 8;

      /* octet 4 */
      unsigned char unused_byte_4          : 8;
    };
  } HumidifierFlags;
  
  unsigned char Autocap_Status;
  
  unsigned char Humidifier_Enable;
  unsigned char Humdifier_Type;
  unsigned char Humidifier_PWM;
  unsigned long Humidifier_Period;
  unsigned long Humidifier_Multiplier;
  unsigned long AutocapOpen_Duration;
  unsigned long AutocapOpen_Period;
  unsigned char Temp_Enable;
  unsigned char Temperature_Type;
  unsigned long Temp_Period;
  unsigned char Temp_T_LOW;
  unsigned char Temp_T_HIGH; 
  unsigned char Heater;
  unsigned char Heater_Hysteresis;

  unsigned long Temperature;
  unsigned long RH;  
  unsigned long Dosing_Temperature;
  unsigned char Nebulizer_state;
  unsigned char Pump_state;  
  unsigned char Led_state;  
  unsigned char Riscaldatore_state;    
} HumidifierAct_t;

/*===== PROTEZIONE PER UNICA INCLUSIONE (FINE FILE) =========================*/
#endif

