#include "FreeRTOS.h"
#include "cmsis_os.h"

#include "rtc.h"
#include "display.h"


/* taskStartDisplay function */
void taskStartDisplay(void const * argument)
{
  // TODO Init RTC to 0 for test
  setRtcDateInSeconds(0);

  /* USER CODE BEGIN taskStartDisplay */
  /* Infinite loop */
  for(;;)
  {
    Display_ShowTime(DISPLAY_LIMIT_SHOW - (uint32_t)getRtcDateInSeconds() );
    
    osDelay(1);
  }
  /* USER CODE END taskStartDisplay */
}

//