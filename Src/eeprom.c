#include "eeprom.h"
#include "const.h"

unsigned char eeprom_wr_buf[16]="AXONETHA"; //EEPROM_WORD_SIZE
unsigned char eeprom_rd_buf[16]= {0};	//EEPROM_WORD_SIZE


int at24_WriteBytes(I2C_HandleTypeDef *hi2c,uint16_t DevAddress,uint16_t MemAddress, uint8_t *pData,uint16_t TxBufferSize)
{
	/*
	 * program just get the DevAddress of the Slave (not master) and for the next step
	 * You know that the most of the EEprom address start with 0xA0
	 * give MemAddress for the location you want to write to
	 * give Data buffer so it can write Data on this location
	 */
	//Note that this function works properly to 31 bytes
	if(MemAddress+TxBufferSize > 16)
	{
		//Write to 16bytes
		while(HAL_I2C_Mem_Write(hi2c,(uint16_t)DevAddress,(uint16_t)MemAddress,I2C_MEMADD_SIZE_8BIT,pData,(uint16_t)16-MemAddress,1000)!= HAL_OK);
		//write remaining bytes
		*pData = *pData + (16-MemAddress);
		while(HAL_I2C_Mem_Write(hi2c,(uint16_t)DevAddress,(uint16_t)16,I2C_MEMADD_SIZE_8BIT,pData,(uint16_t)((MemAddress+TxBufferSize)-16),1000)!= HAL_OK);

	}
	else
	{
			while( (TxBufferSize-16)>0 )
			{
				//if your data is more than 16 bytes,you are here
				 while(HAL_I2C_Mem_Write(hi2c,(uint16_t)DevAddress,(uint16_t)MemAddress,I2C_MEMADD_SIZE_8BIT,pData,(uint16_t)16,1000)!= HAL_OK);
				 TxBufferSize-=16;
				 pData+=16;
				 MemAddress+=16;
			}
			//remaining data
			while(HAL_I2C_Mem_Write(hi2c,(uint16_t)DevAddress,(uint16_t)MemAddress,I2C_MEMADD_SIZE_8BIT,pData,(uint16_t)TxBufferSize,1000)!= HAL_OK);
	}
	return 1;
}
int at24_ReadBytes(I2C_HandleTypeDef *hi2c,uint16_t DevAddress,uint16_t MemAddress, uint8_t *pData,uint16_t RxBufferSize)
{
	int TimeOut;
	/*
	 * program just get the DevAddress of the Slave (not master) and for the next step
	 * You know that the most of the EEprom address start with 0xA0
	 * get the MemAddress for the location you want to write data on it
	 * get the Data buffer so it can write Data on this location
	 */
	//Note that this function works properly to 31bytes

			while( (RxBufferSize-16)>0 )
			{
				//if your data is more than 16 bytes,you are here
				TimeOut = 0;
				 while(HAL_I2C_Mem_Read(hi2c,(uint16_t)DevAddress,(uint16_t)MemAddress,I2C_MEMADD_SIZE_8BIT,pData,(uint16_t)16,1000)!= HAL_OK && TimeOut < 10)
				 {
						TimeOut++;
				 }

				 RxBufferSize-=16;
				 pData+=16;
				 MemAddress+=16;
			}
//			//remaining data
			TimeOut = 0;
			while(HAL_I2C_Mem_Read(hi2c,(uint16_t)DevAddress,(uint16_t)MemAddress,I2C_MEMADD_SIZE_8BIT,pData,(uint16_t)RxBufferSize,1000)!= HAL_OK && TimeOut < 10)
			{
				TimeOut++;
			}

	return 1;
}
