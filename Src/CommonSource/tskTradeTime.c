#include "FreeRTOS.h"
#include "cmsis_os.h"

/* taskStartTradeTime function */
void taskStartTradeTime(void const * argument)
{

  /* USER CODE BEGIN taskStartTradeTime */
  /* Infinite loop */
  for(;;)
  {
    osDelay(10);
  }
  /* USER CODE END taskStartTradeTime */
}