#include "flash.h"

uint8_t flash_wr_buf[F_PAGE_SIZE] = "STM32F4xx SPI Firmware: communication with an Winbond SPI FLASH";
uint8_t flash_rd_buf[F_PAGE_SIZE];
uint8_t flash_id;

//-------------------------------------------------------------------------------------------
uint32_t sFLASH_ReadID(void)
{
  uint32_t Temp = 0;
	uint8_t data[3];

	/*!< Select the FLASH: Chip Select low */
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET); //F_CS low

  /*!< Send "RDID " instruction */
	data[0] = sFLASH_CMD_RDID;
  HAL_SPI_Transmit(&hspi2,data,1,10); // to transmit data

  /*!< Read a bytes from the FLASH */
  HAL_SPI_Receive(&hspi2,data,3,10); // to receive data
 
  /*!< Deselect the FLASH: Chip Select high */
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET); //F_CS high
	
	Temp = data[0] << 16 | data[1] << 8 | data[2]; 
  return Temp;
}
//..............................................................
/**
  * @brief  Erases the specified FLASH sector.
  * @param  SectorAddr: address of the sector to erase.
  * @retval None
  */
void sFLASH_EraseSector(uint32_t SectorAddr)
{
	uint8_t data[4];
  /*!< Send write enable instruction */
  sFLASH_WriteEnable();

  /*!< Sector Erase */
  /*!< Select the FLASH: Chip Select low */
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET); //F_CS low
  /*!< Send Sector Erase instruction */
  
	data[0] = sFLASH_CMD_SE4KB;
	data[1] = (SectorAddr >> 16) & 0xFF;
	data[2] = (SectorAddr >> 8) & 0xFF;
	data[3] = SectorAddr & 0xFF;
	
  HAL_SPI_Transmit(&hspi2,data,4,10); // to transmit data
	/*!< Deselect the FLASH: Chip Select high */
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET); //F_CS high

  /*!< Wait the end of Flash writing */
  sFLASH_WaitForWriteEnd();
}
//.......................................................
/**
  * @brief  Enables the write access to the FLASH.
  * @param  None
  * @retval None
  */
void sFLASH_WriteEnable(void)
{
	uint8_t data[1];
  /*!< Select the FLASH: Chip Select low */
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET); //F_CS low

  /*!< Send "Write Enable" instruction */
  data[0] = sFLASH_CMD_WREN;
	HAL_SPI_Transmit(&hspi2,data,1,10); // to transmit data

  /*!< Deselect the FLASH: Chip Select high */
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET); //F_CS high
}
//....................................................
/**
  * @brief  Polls the status of the Write In Progress (WIP) flag in the FLASH's
  *         status register and loop until write opertaion has completed.
  * @param  None
  * @retval None
  */
void sFLASH_WaitForWriteEnd(void)
{
  uint8_t flashstatus = 0;
	uint8_t data[2];

  /*!< Select the FLASH: Chip Select low */
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET); //F_CS low
	
  /*!< Send "Read Status Register" instruction */
	data[0] = sFLASH_CMD_RDSR1;
  HAL_SPI_Transmit(&hspi2,data,1,10); // to transmit data

  /*!< Loop as long as the memory is busy with a write cycle */
  do
  {
    /*!< Send a dummy byte to generate the clock needed by the FLASH
    and put the value of the status register in FLASH_Status variable */
    data[0] = 0x00;
		HAL_SPI_Receive(&hspi2,data,1,10); // to receive data
		flashstatus = data[0];
  }
  while ((flashstatus & sFLASH_WIP_FLAG) == SET); /* Write in progress */

  /*!< Deselect the FLASH: Chip Select high */
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET); //F_CS high
}
//........................................................
/**
  * @brief  Writes block of data to the FLASH. In this function, the number of
  *         WRITE cycles are reduced, using Page WRITE sequence.
  * @param  pBuffer: pointer to the buffer  containing the data to be written
  *         to the FLASH.
  * @param  WriteAddr: FLASH's internal address to write to.
  * @param  NumByteToWrite: number of bytes to write to the FLASH.
  * @retval None
  */
void sFLASH_WriteBuffer(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite)
{
  uint8_t NumOfPage = 0, NumOfSingle = 0, Addr = 0, count = 0, temp = 0;

  Addr = WriteAddr % sFLASH_SPI_PAGESIZE;
  count = sFLASH_SPI_PAGESIZE - Addr;
  NumOfPage =  NumByteToWrite / sFLASH_SPI_PAGESIZE;
  NumOfSingle = NumByteToWrite % sFLASH_SPI_PAGESIZE;

  if (Addr == 0) /*!< WriteAddr is sFLASH_PAGESIZE aligned  */
  {
    if (NumOfPage == 0) /*!< NumByteToWrite < sFLASH_PAGESIZE */
    {
      sFLASH_WritePage(pBuffer, WriteAddr, NumByteToWrite);
    }
    else /*!< NumByteToWrite > sFLASH_PAGESIZE */
    {
      while (NumOfPage--)
      {
        sFLASH_WritePage(pBuffer, WriteAddr, sFLASH_SPI_PAGESIZE);
        WriteAddr +=  sFLASH_SPI_PAGESIZE;
        pBuffer += sFLASH_SPI_PAGESIZE;
      }

      sFLASH_WritePage(pBuffer, WriteAddr, NumOfSingle);
    }
  }
  else /*!< WriteAddr is not sFLASH_PAGESIZE aligned  */
  {
    if (NumOfPage == 0) /*!< NumByteToWrite < sFLASH_PAGESIZE */
    {
      if (NumOfSingle > count) /*!< (NumByteToWrite + WriteAddr) > sFLASH_PAGESIZE */
      {
        temp = NumOfSingle - count;

        sFLASH_WritePage(pBuffer, WriteAddr, count);
        WriteAddr +=  count;
        pBuffer += count;

        sFLASH_WritePage(pBuffer, WriteAddr, temp);
      }
      else
      {
        sFLASH_WritePage(pBuffer, WriteAddr, NumByteToWrite);
      }
    }
    else /*!< NumByteToWrite > sFLASH_PAGESIZE */
    {
      NumByteToWrite -= count;
      NumOfPage =  NumByteToWrite / sFLASH_SPI_PAGESIZE;
      NumOfSingle = NumByteToWrite % sFLASH_SPI_PAGESIZE;

      sFLASH_WritePage(pBuffer, WriteAddr, count);
      WriteAddr +=  count;
      pBuffer += count;

      while (NumOfPage--)
      {
        sFLASH_WritePage(pBuffer, WriteAddr, sFLASH_SPI_PAGESIZE);
        WriteAddr +=  sFLASH_SPI_PAGESIZE;
        pBuffer += sFLASH_SPI_PAGESIZE;
      }

      if (NumOfSingle != 0)
      {
        sFLASH_WritePage(pBuffer, WriteAddr, NumOfSingle);
      }
    }
  }
}
//.......................................................
/**
  * @brief  Writes more than one byte to the FLASH with a single WRITE cycle 
  *         (Page WRITE sequence).
  * @note   The number of byte can't exceed the FLASH page size.
  * @param  pBuffer: pointer to the buffer  containing the data to be written
  *         to the FLASH.
  * @param  WriteAddr: FLASH's internal address to write to.
  * @param  NumByteToWrite: number of bytes to write to the FLASH, must be equal
  *         or less than "sFLASH_PAGESIZE" value.
  * @retval None
  */
void sFLASH_WritePage(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite)
{
	uint8_t data[4];

  /*!< Enable the write access to the FLASH */
  sFLASH_WriteEnable();

  /*!< Select the FLASH: Chip Select low */
   HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET); //F_CS low
  /*!< Send "Write to Memory " instruction */
	
  data[0] = sFLASH_CMD_WRITE;
	data[1] = (WriteAddr >> 16) & 0xFF;
	data[2] = (WriteAddr >> 8) & 0xFF;
	data[3] = WriteAddr & 0xFF;
	
  HAL_SPI_Transmit(&hspi2,data,4,10); // to transmit data

  /*!< while there is data to be written on the FLASH */
  while (NumByteToWrite--)
  {
    /*!< Send the current byte */
    HAL_SPI_Transmit(&hspi2,pBuffer,1,10); // to transmit data
    /*!< Point on the next byte to be written */
    pBuffer++;
  }

  /*!< Deselect the FLASH: Chip Select high */
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET); //F_CS high

  /*!< Wait the end of Flash writing */
  sFLASH_WaitForWriteEnd();
}
//.....................................................
/**
  * @brief  Reads a block of data from the FLASH.
  * @param  pBuffer: pointer to the buffer that receives the data read from the FLASH.
  * @param  ReadAddr: FLASH's internal address to read from.
  * @param  NumByteToRead: number of bytes to read from the FLASH.
  * @retval None
  */
void sFLASH_ReadBuffer(uint8_t* pBuffer, uint32_t ReadAddr, uint16_t NumByteToRead)
{
	uint8_t data[4];
  /*!< Select the FLASH: Chip Select low */
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET); //F_CS low

  /*!< Send "Read from Memory " instruction */
 	data[0] = sFLASH_CMD_READ;
	data[1] = (ReadAddr >> 16) & 0xFF;
	data[2] = (ReadAddr >> 8) & 0xFF;
	data[3] = ReadAddr & 0xFF;
	
  HAL_SPI_Transmit(&hspi2,data,4,10); // to transmit data


  while (NumByteToRead--) /*!< while there is data to be read */
  {
    /*!< Read a byte from the FLASH */
    	HAL_SPI_Receive(&hspi2,pBuffer,1,10); // to receive data
    /*!< Point to the next location where the byte read will be saved */
    pBuffer++;
  }

  /*!< Deselect the FLASH: Chip Select high */
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET); //F_CS high
}
