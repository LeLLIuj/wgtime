/*
 * \file timer.h
 *
 * Created on: Apr 27, 2016
 * \author Remko Welling, 541858
 *          remko@rfsee.nl
 *
 * To make this timer work the interrupt handler function inside stm32f4xx_it.c shall be modified
 * to allow the Timing delay variable to be incremented at system tick:
\verbatim

	__IO uint32_t TimmingDelay;
	void SysTick_Handler(void)
	{
	  if(TimmingDelay !=0)
	  {
		TimmingDelay --; 	///< Variable for timer class.
	   }
	}

 \endverbatim
 *
 *
 *
 * \version 0.8	Initial release for the HACOX project team for code review
 *
 *
 */


#ifndef TIMER_H
#define TIMER_H

#include <stdint.h>
#include "cmsis_os.h"

class Timer {
private:
  osTimerDef_t os_timer_def_timer_nfc_0;
  bool _isTimerNfc0_Running;
  osTimerId _timer_nfc;
  
  void TimerFinishing(void const *n);
public:
  Timer();
  virtual ~Timer();
  
  /// \brief delay function
  /// wait a specified time in mS
  /// \param time time in mS to wait
  void delay(const uint32_t time);

  /// \brief Set timeout
  /// \param timeout timeout in mS
  void setTimeOut(const uint32_t timeout);

  /// \brief test if timeout is expired.
  /// \return true if timeout is expired.
  bool isTimeOut(void);
};


#endif
