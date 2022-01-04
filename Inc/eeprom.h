#ifndef __EEPROM_H
#define __EEPROM_H

#ifdef __cplusplus
 extern "C" {
#endif
	 
#include "stm32f1xx_hal.h"

extern unsigned char eeprom_wr_buf[16]; //EEPROM_WORD_SIZE
extern unsigned char eeprom_rd_buf[16];	//EEPROM_WORD_SIZE

int at24_WriteBytes(I2C_HandleTypeDef *hi2c,unsigned short int DevAddress,unsigned short int MemAddress, unsigned char *pData,unsigned short int TxBufferSize);
int at24_ReadBytes(I2C_HandleTypeDef *hi2c,unsigned short int DevAddress,unsigned short int MemAddress, unsigned char *pData,unsigned short int RxBufferSize);

#ifdef __cplusplus
}
#endif

/**
  * @}
*/ 

#endif /* __EEPROM_H */
