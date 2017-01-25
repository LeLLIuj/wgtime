#include "FreeRTOS.h"
#include "cmsis_os.h"

/* taskStartGyroscope function */
void taskStartGyroscope(void const * argument)
{

  /* USER CODE BEGIN taskStartGyroscope */
  /* Infinite loop */
  for(;;)
  {
    osDelay(10);
  }
  /* USER CODE END taskStartGyroscope */
}