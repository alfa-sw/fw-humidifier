/*
*//*=====================================================================*//**
**
**      Nome del file  : TIMERMG.H
**
**      Descrizione    : Inclusioni relative al modulo TIMERMG.C
**
**      Progetto       : alfa color tester
**
*//*=====================================================================*//**
*/

#ifndef _TIMER_MG_H
#define _TIMER_MG_H

/**
 * Timebase is 2 ms
 * e.g. 1s = 500 ticks
 */
#define  T_BASE 2

/**
 * Timers management
 */
#define NOT_RUNNING -2
#define START_TIMER 1
#define STOP_TIMER  0
#define T_CLEAR_IF_ELAPSED 1
#define T_START_IF_ELAPSED 2
#define T_READ 0
#define T_ELAPSED      -1
#define T_RUNNING       2
#define T_HALTED        0
#define T_STARTED       1
#define T_NOT_RUNNING  -2

/**
 * Conversion to seconds
 * 1 sec = 1000 ms, 1 count each 2 ms ->1000/2 = 500
 */
#define  CONV_SEC_COUNT  500L

/**
 * Timers
 */
 enum {
   /* 1 */ T_READ_IO,
   /* 2 */ T_DELAY_INTRA_FRAMES,
   /* 3 */ T_SLAVE_WAIT_TIMER,	
   /* 4 */ T_SLAVE_WAIT_LINK_TIMER,
   /* 5 */ T_HUM_CAP_OPEN_ON,
   /* 6 */ T_HUM_CAP_OPEN_PERIOD,	
   /* 7 */ T_HUM_CAP_CLOSED_ON,	
   /* 8 */ T_HUM_CAP_CLOSED_PERIOD,
   /* 9 */ T_DOS_PERIOD,
   /* 10 */T_HARD_RESET,
   /* 11 */T_SHT30_MEASUREMENT,
   /* 12 */T_SHT30_WRITE_TIMEOUT,
   /* 13 */T_SHT30_HEATER,
   /* 14 */
   N_TIMERS
 };

/* 1 */ #define DELAY_READ_IO 10
/* 2 */ #define DELAY_INTRA_FRAMES 2
/* 3 */ #define DELAY_WAIT_SLAVE   3000 // 3sec
/* 4 */ #define DELAY_SLAVE_WAIT_LINK_TIMER  10000 // 10 sec
// Default Activation Duration with Autocap Open: 20"
/* 5 */ # define DELAY_HUM_CAP_OPEN_ON 10000	
// Base Timer Activation Period with Autocap Open: 1"
/* 6 */ # define DELAY_HUM_CAP_OPEN_PERIOD 500
// Default Activation Duration with Autocap Closed: 5"
/* 7 */ # define DELAY_HUM_CAP_CLOSED_ON 2500
// Base Timer Activation Period with Autocap Closed: 1"
/* 8 */ # define DELAY_HUM_CAP_CLOSED_PERIOD 500
// Base Timer Dosing Temperature Activation Period : 1"
/* 9 */	# define DELAY_DOS_PERIOD	500			
// Waiting Reset Time: 2 msec
/* 10 */# define DELAY_HARD_RESET 1  
// Waiting SHT30 Measurement: 8 msec
/* 11 */# define DELAY_SHT30_MEASUREMENT 4
// Timeout on SHT30 Write Command: 4 msec
/* 12 */# define DELAY_SHT30_TIMEOUT 2
// Wait 10" with HEATER ON
 /* 13 */# define DELAY_WAIT_HEATER 5000
/* 14 */
/* 15 */	

typedef struct {
  signed char Flg;
  unsigned short InitBase;
} timerstype;

extern unsigned short TimeBase;
extern timerstype TimStr[N_TIMERS];
extern unsigned short Durata[N_TIMERS];
extern void TimerMg (void);
extern unsigned short ReadTimer(unsigned char timer);
extern void StartTimer(unsigned char Timer);
extern void StopTimer(unsigned char Timer);
extern unsigned char NotRunningTimer(unsigned char Timer);
extern signed char StatusTimer(unsigned char Timer);
extern void InitTMR(void);

#endif

