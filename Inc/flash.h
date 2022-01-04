#ifndef __FLASH_H
#define __FLASH_H

#ifdef __cplusplus
 extern "C" {
#endif
	 
#include "stm32f1xx_hal.h"
#include "const.h"

/* Private macro -------------------------------------------------------------*/
#define sFLASH_CMD_WREN           0x06  /* Write enable instruction */
#define sFLASH_CMD_WRDIS          0x04  /* Write disable instruction */

#define sFLASH_CMD_WRITE          0x02  /* Write to Memory instruction (page program)*/
#define sFLASH_CMD_WRSR           0x01  /* Write Status Register instruction */

#define sFLASH_CMD_READ           0x03  /* Read from Memory instruction */
#define sFLASH_CMD_RDSR1          0x05  /* Read Status Register 1 instruction  */
#define sFLASH_CMD_RDSR2          0x35  /* Read Status Register 2 instruction  */

#define sFLASH_CMD_RDID           0x9F  /* Read identification */

#define sFLASH_CMD_SE4KB          0x20  /* Sector Erase instruction (4KB) */
#define sFLASH_CMD_BE32KB         0x52  /* Bulk Erase instruction (32KB)*/
#define sFLASH_CMD_BE64KB         0xD8  /* Bulk Erase instruction (64KB)*/
#define sFLASH_CMD_CE             0xC7  /* Chip Erase instruction */


#define sFLASH_WIP_FLAG           0x01  /* Write In Progress (WIP) flag */
//..........................................................................
#define  countof(a) (sizeof(a) / sizeof(*(a)))
#define  BufferSize (countof(flash_wr_buf)-1)

/* Private variables ---------------------------------------------------------*/
extern SPI_HandleTypeDef hspi2;
extern uint8_t flash_wr_buf[F_PAGE_SIZE];
extern uint8_t flash_rd_buf[F_PAGE_SIZE];
extern uint8_t flash_id;


//--------------------------------------------------------------------------------
uint32_t sFLASH_ReadID(void);
void sFLASH_EraseSector(uint32_t SectorAddr);
void sFLASH_WriteEnable(void);
void sFLASH_WaitForWriteEnd(void);
void sFLASH_WriteBuffer(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite);
void sFLASH_WritePage(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite);
void sFLASH_ReadBuffer(uint8_t* pBuffer, uint32_t ReadAddr, uint16_t NumByteToRead);

//--------------------------------------------------------------------------------

#ifdef __cplusplus
}
#endif

/**
  * @}
*/ 

#endif /* __FLASH_H */

