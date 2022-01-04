#ifndef __GSM_H
#define __GSM_H

#ifdef __cplusplus
 extern "C" {
#endif
	 
#include "stm32f1xx_hal.h"
#include "const.h"

#define CTRL_Z       		"\x1A"
#define RETRY_CNT  			4
#define _1000MS_DELAY   (unsigned int)500   //1000

#define AT_QIMODE  	"AT+QIMODE=0\r\n" //gprs: select TCP/IP stack mode as non-transparent   
#define AT_QICSGP  	"AT+QICSGP=1,\"CMNET\"\r\n" //gprs: set APN as CMNET 
#define AT_QIREGAPP "AT+QIREGAPP\r\n"  //gprs: start TCP/IP task
#define AT_QIACT   	"AT+QIACT\r\n"  //gprs: active the gprs context
#define AT_QILOCIP 	"AT+QILOCIP\r\n" //gprs: get the local IP address
#define ATV1       	"ATV1\r\n" //gprs: set the response format
#define AT_QIHEAD  	"AT+QIHEAD=1\r\n" //gprs: add header information when receive data
#define AT_QIDNSIP 	"AT+QIDNSIP=0\r\n" //gprs: 0 to use the IP address to establish TCP/UDP session
                                      //       1 to use the domain name to establish TCP/UDP session 																
#define AT_QISEND  	"AT+QISEND\r\n"    //gprs: send data to server (max length: 1460 )
#define AT_QISACK  	"AT+QISACK\r\n"		//gprs: ack to sent data																
#define AT_QIDEACT 	"AT+QIDEACT\r\n"   //gprs: deactivate gprs context
#define AT_QLTS 		"AT+QLTS\r\n" // for RTC
#define AT_CNMI  		"AT+CNMI?\r\n" // for new msg indication
#define AT_CMGL  		"AT+CMGL=\"ALL\"\r\n"
#define AT_READ  		"AT+CMGR=1\r\n"
#define AT_CCLK  		"AT+CCLK?\r\n" // for date and time //+CCLK=?
#define AT_CSQ   		"AT+CSQ\r\n"  // for GSM signal strength

//-----------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------
extern int position;	/* save location of current message */
extern char buff[GSM_RX_BUFF_SIZE]; /* buffer to store responses and messages */
extern unsigned short  buffer_pointer;
extern char Mobile_no[14];		/* store mobile no. of received message */
extern char message_received[GSM_MSG_SIZE];		/* save received message */
extern char rx_byte_cnt;
extern short rx_index; 
extern unsigned char rx_byte; 
extern unsigned char gsm_ss;
extern short gps_rx_index;
extern char gps_buff[100];
extern unsigned char gps_rx_byte;

extern unsigned char SCH1_START_HH,SCH1_START_MM,SCH1_STOP_HH,SCH1_STOP_MM;
extern unsigned char SCH2_START_HH,SCH2_START_MM,SCH2_STOP_HH,SCH2_STOP_MM;
extern unsigned char SCH3_START_HH,SCH3_START_MM,SCH3_STOP_HH,SCH3_STOP_MM;
extern unsigned char SCH4_START_HH,SCH4_START_MM,SCH4_STOP_HH,SCH4_STOP_MM;

extern short T_RUN_TIME; // in minutes  5 - 480
extern short T_ONDELAY;  // 30 in Sec  30 - 300
extern short T_DRYRUN;   // in minutes  2 - 600

extern char latitude_buffer[16];
extern char longitude_buffer[16];
extern char user1[12]; 
extern char user2[12];

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;

extern struct STATUS_FLAGS Status_Flags,*Status_Flag;
extern struct rtc *pLocal_time;
extern unsigned short STAT_IND;
extern I2C_HandleTypeDef hi2c1;
extern unsigned char eeprom_wr_buf[16];//EEPROM_WORD_SIZE

//-------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------
extern void motor_on(void);
extern void motor_off(void);

void GSM_Engine_RESET(void);
void GSM_M60_Enable(void);
void GSM_Begin(void);
void GSM_Send_Msg(char *num,char *sms);
char GSM_Wait_for_Msg(void);
void GSM_Msg_Read(int);
void GSM_Msg_Decode(void);
void GSM_Msg_Delete(unsigned int);
void GSM_Delete_All_Msg(void);
void GNSS_Begin(void);
void GNSS_PowerON(void);
void GNSS_PowerOFF(void);
void GNSS_Read(void);
void GPS_Process_Data(void);
void gsm_sms_fsm(void);
void GPRS_Begin(void);
void GPRS_Connect(void);
void GPRS_Send_Data(char*);
void GPRS_Disconnect(void);
void get_sche_time(char *,unsigned char *,unsigned char *,unsigned char *,unsigned char *);
void get_timers(char *,short *);
void GSM_get_rtc(void);
void GSM_Signal_Strength(void);

#ifdef __cplusplus
}
#endif

/**
  * @}
*/ 

#endif /* __GSM_H */

