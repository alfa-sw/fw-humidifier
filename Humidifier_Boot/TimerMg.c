/**/
/*============================================================================*/
/**
**      @file    timeMg.c
**
**      @brief   Timers Manager module
**/
/*============================================================================*/
/**/
#include "Compiler.h"
#include "TimerMg.h"
#include "Macro.h"
#include "mem.h"

timerstype BL_TimStr[N_TIMERS];
unsigned short BL_Durata[N_TIMERS] = {
/*  0 */    DELAY_FORCE_STAND_ALONE,
/*  1 */    DELAY_INTRA_FRAMES,
/*  2 */    DELAY_T_FIRST_WINDOW,
};

static volatile unsigned short BL_TimeBase;

/* Timer Manager sequencer */
void BL_TimerMg(void)
{
  static unsigned short BL_MonTimeBase;
  unsigned char temp;

  /* Mirror time base */
  _T1IE=0;
  BL_MonTimeBase = BL_TimeBase;
  _T1IE=1;

  for (temp = 0; temp < N_TIMERS; temp++) {

    if (BL_TimStr[temp].Flg == T_RUNNING) {
      if ((BL_MonTimeBase - BL_TimStr[temp].InitBase) >= BL_Durata[temp])
        BL_TimStr[temp].Flg = T_ELAPSED;
    }

    if (BL_TimStr[temp].Flg == T_STARTED) {
      BL_TimStr[temp].InitBase = BL_MonTimeBase;
      BL_TimStr[temp].Flg = T_RUNNING;
    }
  }
} /* BL_TimerMg() */

/* Timer1 initialization for TimerMg */
void BL_TimerInit (void)
{
  //Timer 1 controls position/speed controller sample time
  TMR1 = 0;                       // Resetting TIMER
  PR1 = SPEED_CONTROL_RATE_TIMER; // speed controller rate
  T1CON = 0x0000;                 // reset timer configuration
  T1CONbits.TCKPS = 1;            // 1 = 1:8 prescaler

  IPC0bits.T1IP = 3;              // Set Timer 1 Interrupt Priority Level
  IFS0bits.T1IF = 0;              // Clear Timer1 Interrupt Flag
  IEC0bits.T1IE = 1;              // Enable Timer1 interrupt
  T1CONbits.TON = 1;              // Enable Timer1
}

/* Timer 1 Interrupt handler */
void __attribute__((__interrupt__,auto_psv)) _AltT1Interrupt(void)
{
  IFS0bits.T1IF = 0;                          //Clear Timer 1 Interrupt Flag
  ++ BL_TimeBase;
}
