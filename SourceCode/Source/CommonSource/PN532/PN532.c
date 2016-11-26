#include "stm32f10x.h"
#include "core_cm3.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_i2c.h"



#define RTC_reg_address   0  //adress of the register within ds1307 or RTC
#define RTC_address      (0X77 & 0x7f) << 1 // Device address i.e RTC address
#define mpu6050_FLAG_TIMEOUT         ((uint32_t)0x1000)
#define mpu6050_LONG_TIMEOUT         ((uint32_t)(500 * mpu6050_FLAG_TIMEOUT))



uint8_t sec = 0;

void initPN532(void) {
  GPIO_InitTypeDef GPIO_InitStructure;
  I2C_InitTypeDef  I2C_InitStructure;
  
  // Включаем тактирование I2C
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

  // Настраиваем порты SCL and SDA
  GPIO_PinRemapConfig(GPIO_Remap_I2C1, ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  I2C_InitStructure.I2C_OwnAddress1 = 0x00; // MPU6050 7-bit adress = 0x68, 8-bit adress = 0xD0;
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;

  I2C_Init(I2C2, &I2C_InitStructure);

  I2C_Cmd(I2C2, ENABLE);
}


uint8_t PN532_WRiteReg(uint8_t data,uint8_t RegAdd) {
  
  int mpu6050_Timeout = 0;

  
  /* Test on BUSY Flag */
  mpu6050_Timeout = mpu6050_LONG_TIMEOUT;
  while(I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY) != RESET) {
    if((mpu6050_Timeout--) == 0) {
      //USART_print(" error1 ");
    }
  }

  /* Send Start */
  I2C_GenerateSTART(I2C2, ENABLE);



  /* Send Device address address of the DS1307*/
  I2C_SendData(I2C1, RTC_address);


  /* Send the address of register within device DS1307*/
  I2C_SendData(I2C1, (uint8_t)RegAdd);

  
  /* Wait until TC flag is set */
  mpu6050_Timeout = mpu6050_LONG_TIMEOUT;
  while(I2C_GetFlagStatus(I2C2, I2C_FLAG_TC) == RESET) {
    if((mpu6050_Timeout--) == 0) {
      //USART_print(" error3 ");
    }
  }

  
  /* Send the data to write in internal register*/
  I2C_SendData(I2C2, (uint8_t)data);
  
  
  /* Wait until TC flag is set */
  mpu6050_Timeout = mpu6050_LONG_TIMEOUT;
  while(I2C_GetFlagStatus(I2C2, I2C_FLAG_TC) == RESET) {
    if((mpu6050_Timeout--) == 0) {
      //USART_print(" error3 ");
    }
  }


  I2C_GenerateSTOP(I2C2, ENABLE);
  /*stop bit flag*/
  while(I2C_GetFlagStatus(I2C2, I2C_FLAG_STOPF));
  
  return 1;
}


uint8_t PN532_ReadReg(uint8_t RegAdd) {
  uint8_t mpu6050_BufferRX[2] ={0,0};
  uint8_t temp;
  uint32_t DataNum = 0;
  int mpu6050_Timeout = 0;

  /* Test on BUSY Flag */
#if 1
  mpu6050_Timeout = mpu6050_LONG_TIMEOUT;
  while(I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY) != RESET) {
    if((mpu6050_Timeout--) == 0) {
      //USART_print(" error1 ");
    }
  }
#endif
  
  /* Configure slave address, nbytes, reload, end mode and start or stop generation */
  I2C_TransferHandling(I2C2, RTC_address, 1, I2C_SoftEnd_Mode, I2C_Generate_Start_Write);

  /* Wait until TXIS flag is set */
  mpu6050_Timeout = mpu6050_LONG_TIMEOUT;
  while(I2C_GetFlagStatus(I2C2, I2C_FLAG_TXIS) == RESET) {
    if((mpu6050_Timeout--) == 0) {
      //USART_print(" error2 ");
    }
  }

  /* Send Register address */
  I2C_SendData(I2C2, (uint8_t)RegAdd);

  /* Wait until TC flag is set */
  mpu6050_Timeout = mpu6050_LONG_TIMEOUT;
  while(I2C_GetFlagStatus(I2C2, I2C_FLAG_TC) == RESET) {
    if((mpu6050_Timeout--) == 0) {
      //USART_print(" error3 ");
    }
  }

  /* Configure slave address, nbytes, reload, end mode and start or stop generation */
  I2C_TransferHandling(I2C2, RTC_address, 1, I2C_AutoEnd_Mode, I2C_Generate_Start_Read);

  /* Reset local variable */
  DataNum = 0;

  /* Wait until all data are received */
  while (DataNum != 1) {
    /* Wait until RXNE flag is set */
      mpu6050_Timeout = mpu6050_LONG_TIMEOUT;
    while(I2C_GetFlagStatus(I2C2, I2C_FLAG_RXNE) == RESET) {
      if((mpu6050_Timeout--) == 0) {
        //USART_print(" error4 ");
      }
    }

    /* Read data from RXDR */
    mpu6050_BufferRX[DataNum]= I2C_ReceiveData(I2C2);

    /* Update number of received data */
    DataNum++;
  }

  /* Wait until STOPF flag is set */
  mpu6050_Timeout = mpu6050_LONG_TIMEOUT;
  while(I2C_GetFlagStatus(I2C2, I2C_FLAG_STOPF) == RESET) {
    if((mpu6050_Timeout--) == 0) {
      //USART_print(" error5 ");
    }
  }

  /* Clear STOPF flag */
  I2C_ClearFlag(I2C2, I2C_ICR_STOPCF);


  // !< Store LM75_I2C received data
  temp = mpu6050_BufferRX[0];
  
  // return a Reg value
  return (uint8_t)temp;
}