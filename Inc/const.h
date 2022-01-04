#ifndef __CONST_H
#define __CONST_H

#ifdef __cplusplus
 extern "C" {
#endif
	 
	 
#define SMS_DEBUG 1
//#define GPS_DEBUG 1
//#define GPRS_DEBUG 1
//#define METER_DEBUG 1

#define _500MS_EVENT_PERIOD		(unsigned int)500  		// event period for schedulers 
#define _5MS_EVENT_PERIOD			(unsigned int)5  		  // event period for schedulers 
#define _100MS_EVENT_PERIOD		(unsigned int)4			  // 100ms period  @ 25mS
#define _5S_EVENT_PERIOD			(unsigned int)5000 
#define ZERO                  (unsigned int)0

#define MOTOR_ON_OFF   1
#define MOTOR_FAULT    2
#define SYS_HEALTH     3
#define TIMER_MODE     4
#define SMS_GPRS       5
#define SIG_STR        6 

#define OFF   				0
#define ON    				1
#define SLOW_BLINK   	2
#define FAST_BLINK   	3

	
#define CLEAR  0
#define SET    1

#define GSM_RX_BUFF_SIZE     300   // sms receive buffer size //160
#define GSM_MSG_SIZE         60    // valid sms string size
#define GSM_SEND_MSG_SIZE    160   // valid sms string size

#define DB_TX_BUFF_SIZE     200   // transmit frame to display board - buffer size
#define DB_RX_BUFF_SIZE     100   // receive frame from display board - buffer size

#define METER_TOT_REG       18    //12    // total register to read from meter
#define METER_TOT_PARAMS    30    // total meter parametrs

#define F_PAGE_SIZE         256   // Flash page size for reading and writing  
#define EEPROM_WORD_SIZE    8     // EEPROM word size for reading and writing


#define EEPROM_AM_ADDR     				0     //  Auto/Manual status storage address
#define EEPROM_AM_SIZE     				1     //  Auto/Manual status number of bytes  
#define EEPROM_ON_OFF_ADDR     		1     //  Auto/Manual status storage address
#define EEPROM_ON_OFF_SIZE     		1     //  Auto/Manual status number of bytes 

#define EEPROM_CUM_KWH_ADDR       2
#define EEPROM_CUM_KWH_SIZE       6

#define EEPROM_SCH_TIME_SIZE      8
#define EEPROM_SCH1_ADDR  				10
#define EEPROM_SCH2_ADDR          18
#define EEPROM_SCH3_ADDR          26
#define EEPROM_SCH4_ADDR          34   // to 42

#define EEPROM_T_RUN_TIME_ADDR    42
#define EEPROM_T_RUN_TIME_SIZE    3
#define EEPROM_T_ONDELAY_ADDR     45
#define EEPROM_T_ONDELAY_SIZE     3
#define EEPROM_T_DRYRUN_ADDR    	48
#define EEPROM_T_DRYRUN_SIZE     	3

//.......................flash .........................
#define  FLASH_WRITE_ADDRESS      0x010000    // block1
#define  FLASH_READ_ADDRESS       FLASH_WRITE_ADDRESS
#define  FLASH_SECTOR_TO_ERASE    FLASH_WRITE_ADDRESS

#define  sFLASH_SPI_PAGESIZE      256 

#define UNDER_VOLTAGE    	330    // 220-280 VAC
#define OVER_VOLTAGE      477    // 450-500 VAC 
#define UNDER_LOAD     		2      //4    		// 1-18 A
																	/* Less than 50% of pump current
																	for 5HP motor current : 8 Amps
																	Less than 50% current: < 4 Amps */
																	
#define OVER_LOAD      		4       //11.2    // 1-18 A
																	/* More than 40% of pump current
																	for 5HP motor current : 8 Amps
																	More than 40% current: > 11.2 Amps */
																	
#define THERMAL_OVERLOAD   20
#define LOCKED_ROTOR       24     // 24 AMPS  

#define CURRENT_UNBALANCE    8    // persentage of current unbalance 2.5 to 25
#define VOLTAGE_UNBALANCE    8    // persentage of voltage unbalance

#define OVER_FREQ           52.5
#define UNDER_FREQ          47 
#define IFLC                7.6      
#define EARTH_FAULT         20

#define MAX_NO_ONS          30
#define MAX_NO_ONS_TIME     60000  // 60 SEC 
#define ON_REENABLE_TIME    120000 // 120 SEC


#ifdef __cplusplus
}
#endif

/**
  * @}
*/ 

#endif /* __CONST_H */

