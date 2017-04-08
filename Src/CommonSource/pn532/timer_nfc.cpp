/*
 * \file timer.h
 *
 * Created on: Apr 27, 2016
 * \author Remko Welling, 541858
 *          remko@rfsee.nl
 *
 *
 * \version 0.8	Initial release for the HACOX project team for code review
 *
 *
 */

#include "timer.h"
#include "stm32f1xx_hal.h"

char isTimerRunning = 0;

void TimerFinishing(void const *n) {
    isTimerRunning = 0;
}

osTimerDef(timer_nfc_0, TimerFinishing);
osTimerId timer_nfc;

void TimerNFC_Init()
{
  timer_nfc = osTimerCreate(osTimer(timer_nfc_0), osTimerOnce, (void *)0);
}

void TimerNFC_Delay(const uint32_t time)
{
  osDelay(time);
}
 
void TimerNFC_setTimeOut(const uint32_t timeout)
{
  osTimerStart(timer_nfc, timeout);
}

char TimerNFC_isTimeOut(void)
{
  return isTimerRunning;
}

