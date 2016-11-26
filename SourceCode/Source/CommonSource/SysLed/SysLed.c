#include "stm32f10x.h"
#include "core_cm3.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"

#include "sysLed.h"

//==============================================================================
#define LED_SYSTEM_PIN  GPIO_Pin_13
#define LED_SYSTEM_PORT GPIOC

//==============================================================================

BitAction ledSignal = Bit_RESET;
volatile uint32 time = 0;


//-----------------------------------------------------------
//------------------- initSysLed ----------------------------
//-----------------------------------------------------------
void initSysLed(void) {
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);// Clock PORTC Enable
  
  GPIO_InitTypeDef GPIO_InitStructure;
  
  // Init System Led
  GPIO_InitStructure.GPIO_Pin =  LED_SYSTEM_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(LED_SYSTEM_PORT, &GPIO_InitStructure);
}


void sysLedHandler(void) {
  GPIO_WriteBit(LED_SYSTEM_PORT, LED_SYSTEM_PIN, (BitAction)ledSignal);

  if (time == 0) {
    time = 5000;
    ledSignal = ledSignal ? Bit_RESET : Bit_SET;
  }
  
  time--;
}