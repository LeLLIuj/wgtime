#include "FreeRTOS.h"
#include "cmsis_os.h"

/* taskStartDisplay function */
void taskStartDisplay(void const * argument)
{

  /* USER CODE BEGIN taskStartDisplay */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END taskStartDisplay */
}