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

#include "timer_nfc.h"
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"


Timer::Timer() {
  _isTimerNfc0_Running = false;
  _timer_nfc = osTimerCreate(osTimer(timer_nfc_0), osTimerOnce, (void *)0);
}
Timer::~Timer() {
  
}

void Timer::TimerFinishing(void const *n) {
    _isTimerNfc0_Running = 0;
}

void Timer::delay(const uint32_t time)
{
  osDelay(time);
}
 
void Timer::setTimeOut(const uint32_t timeout)
{
  _isTimerNfc0_Running = true;
  osTimerStart(_timer_nfc, timeout);
}

bool Timer::isTimeOut(void)
{
  return _isTimerNfc0_Running;
}

