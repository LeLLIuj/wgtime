#ifndef __PN532_H_
#define __PN532_H_

void initPN532(void);
uint8_t PN532_WRiteReg(uint8_t data,uint8_t RegAdd);
uint8_t PN532_ReadReg(uint8_t RegAdd);

#endif // __PN532_H_