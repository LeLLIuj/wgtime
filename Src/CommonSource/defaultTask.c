#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "gpio.h"

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{
  int ledSignal = 0;

  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    // Led Controll
    if (ledSignal) {
      BoardLedOn();
    } else {
      BoardLedOff();
    }
    
    ledSignal = ledSignal ? 0 : 1;
    osDelay(1000);
  }
  /* USER CODE END StartDefaultTask */
}
