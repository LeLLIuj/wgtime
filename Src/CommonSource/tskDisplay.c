#include "FreeRTOS.h"
#include "cmsis_os.h"

#include "rtc.h"
#include "display.h"


/* taskStartDisplay function */
void taskStartDisplay(void const * argument)
{
  // TODO Init RTC to 0 for test
  setRtcDateInSecondsTime(0);

  /* USER CODE BEGIN taskStartDisplay */
  /* Infinite loop */
  for(;;)
  {
    Display_ShowTime(DISPLAY_LIMIT_SHOW - (uint32_t)getRtcDateInSecondsTime());
    
    osDelay(1000);
  }
  /* USER CODE END taskStartDisplay */
}


// Real time priority task
void taskStartDisplay_Stopwatch(void const * argument)
{

  /* USER CODE BEGIN taskStartDisplay */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1000);
  }
  /* USER CODE END taskStartDisplay */
}