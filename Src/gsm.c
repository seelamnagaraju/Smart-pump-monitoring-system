#include "gsm.h"
#include <string.h>
#include <stdlib.h>
#include "struct_var.h"
#include "eeprom.h"

// commands for on/off/status
const char ON_PUMP[1] = "1";
const char ON_PUMP1[2] = "1*";
const char OFF_PUMP[1] = "2";
const char OFF_PUMP1[2] = "2*";
const char GETSTATUS[10] = "GETSTATUS*";
const char GETSETTINGS[12] = "GETSETTINGS*";
const char SMSOFF[7] = "SMSOFF*";
const char SMSON[6] = "SMSON*";
const char MISSEDCALL[11] = "MISSEDCALL*";
const char IVRS[5] = "IVRS*";
const char GETGPS[7] = "GETGPS*";

// commands for timers
const char SETSLOT1[8] = "SETSLOT1";
const char SETSLOT2[8] = "SETSLOT2";
const char SETSLOT3[8] = "SETSLOT3";
const char SETSLOT4[8] = "SETSLOT4";
const char DELSLOT1[10] = "DEL*SLOT1*";
const char DELSLOT2[10] = "DEL*SLOT2*";
const char DELSLOT3[10] = "DEL*SLOT3*";
const char DELSLOT4[10] = "DEL*SLOT4*";
const char SETRUNTIME[11] = "SETRUNTIME*";
const char ONDELAY[8] = "ONDELAY*";
const char DRYRUN[7] = "DRYRUN*";

//commands to add/delete other users
const char ADDUSER1[7] = "USER1*";
const char ADDUSER2[7] = "USER2*";
const char DELUSER1[9] = "DEL*USER1";
const char DELUSER2[9] = "DEL*USER2";
const char GETUSERS[9] = "GETUSERS*";

//commands for panel installation
const char SETSUPPORT[11] = "SETSUPPORT*";
const char SETMASTER[11] = "SETMASTER*";
const char INSTALL[8] = "INSTALL*"; // to set SPC id and date
const char SETFLOW[8] = "SETFLOW*"; // to set Motor Flow Rate
const char DISABLESER[18] = "DISABLE*SERVERCMD*";
const char DELALLSMS[10] = "DELALLSMS*";

const char AT[4] = "AT\r\n";
const char ATE0[6] = "ATE1\r\n"; 
const char AT_CMGF[11] = "AT+CMGF=1\r\n";// Set SMS message format as text mode.
const char AT_CMGD[13] = "AT+CMGD=1,4\r\n";

//char AT_CMGD[13] = "AT+CMGD=1,4\r\n"; // to delete all sms
//.........................................................
const char AT_CSCS[15] =  "AT+CSCS=\"GSM\"\r\n";// Set character set as GSM which is used by the TE.
const char AT_CNMI1[19] = "AT+CNMI=2,2,0,1,0\r\n";// SMS status report is supported under text mode if <fo> is set to 49.
    // code: +CDS.

// Enable GNSS.
const char AT_QGNSSC_OFF[13] = "AT+QGNSSC=0\r\n";
const char AT_QGNSSC_ON[13] = "AT+QGNSSC=1\r\n"; 

const char AT_QIFGCNT[14] = "AT+QIFGCNT=2\r\n";
const char AT_QICSGP1[21] = "AT+QICSGP=1,\"CMNET\"\r\n";  // set apn
const char AT_QGNSSTS[13] = "AT+QGNSSTS?\r\n";
const char AT_QGNSSEPO[15] = "AT+QGNSSEPO=1\r\n";
const char AT_QGEPOAID[13] = "AT+QGEPOAID\r\n";
const char AT_QGNSSCRD[13] = "AT+QGNSSRD?\r\n";

const char AT_QGNSSCRD_NAV[23] = "AT+QGNSSRD=\"NMEA/GLL\"\r\n";
//.........................................................
const char AT_QIOPEN[37] = "AT+QIOPEN=\"TCP\",\"49.207.4.146\",\"2401\""; //gprs : connect to TCP server 

//char AT_CMGF[11] = "AT+CNUM\r\n";
//.........................................................


int position = 0;	/* save location of current message */
char buff[GSM_RX_BUFF_SIZE]; /* buffer to store responses and messages */

unsigned short  buffer_pointer;  //volatile int buffer_pointer;
char Mobile_no[14];		/* store mobile no. of received message */
char message_received[GSM_MSG_SIZE];		/* save received message */
char rx_byte_cnt = 0;
short rx_index = 0; 
unsigned char rx_byte = 0; 
unsigned char gsm_ss;

short gps_rx_index;
char gps_buff[100];
unsigned char gps_rx_byte;

char latitude_buffer[16] = { 0 };
char longitude_buffer[16] = { 0 };

short T_RUN_TIME = 480; // in minutes  5 - 480
short T_ONDELAY  = 10;  // 30 in Sec  30 - 300
short T_DRYRUN   = 2;   // in minutes  2 - 600

unsigned char SCH1_START_HH,SCH1_START_MM,SCH1_STOP_HH,SCH1_STOP_MM;
unsigned char SCH2_START_HH,SCH2_START_MM,SCH2_STOP_HH,SCH2_STOP_MM;
unsigned char SCH3_START_HH,SCH3_START_MM,SCH3_STOP_HH,SCH3_STOP_MM;
unsigned char SCH4_START_HH,SCH4_START_MM,SCH4_STOP_HH,SCH4_STOP_MM;


extern char user1[12];
extern char user2[12];

//-------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------
void GSM_Engine_RESET(void)
{
		GSM_M60_Enable();
		GSM_Begin();	/* check GSM responses and initialize GSM */
		HAL_Delay(500);
		//GSM_Delete_All_Msg();
}


void GSM_M60_Enable(void)
{
		
	HAL_GPIO_WritePin(GPIOC, gsm_pwr_Pin, GPIO_PIN_SET);
	HAL_Delay(1); // 500
	HAL_GPIO_WritePin(GPIOC, gsm_pwr_Pin, GPIO_PIN_RESET);
	HAL_Delay(10); //20000
	HAL_GPIO_WritePin(GPIOC, gsm_pwr_Pin, GPIO_PIN_SET);
	HAL_Delay(1); //500
	
	HAL_Delay(10); // 10 sec delay for gsm engine ready
		
}

void GSM_Begin(void)
{
	char err_cnt = 0;
	
	do
	{
		HAL_UART_Transmit(&huart1,(uint8_t *)AT,sizeof(AT),1);/* "ATE0\r\n" send ATE0 to check module is ready or not */
		HAL_Delay(2000);
		HAL_Delay(_1000MS_DELAY); 
		#ifdef SMS_DEBUG
			HAL_UART_Transmit(&huart5,(uint8_t *)AT,sizeof(AT),50);
			#endif
		if(strstr(buff,"OK"))
		{
			#ifdef SMS_DEBUG
			HAL_UART_Transmit(&huart5,(uint8_t *)buff,strlen(buff),50);
			#endif
			memset(buff,0,sizeof(buff));
			rx_index = 0;
			break;
		}
		else
		{
			//"Error handling";
			#ifdef SMS_DEBUG
			HAL_UART_Transmit(&huart5,(uint8_t *)buff,strlen(buff),50);
			#endif
		}
	}while(err_cnt++ <= RETRY_CNT);
	
	err_cnt = 0;
	do
	{
		HAL_UART_Transmit(&huart1,(uint8_t *)ATE0,sizeof(ATE0),1);/* "ATE0\r\n" send ATE0 to check module is ready or not */
		HAL_Delay(2000);
		//HAL_Delay(_1000MS_DELAY);
		#ifdef SMS_DEBUG
			HAL_UART_Transmit(&huart5,(uint8_t *)ATE0,sizeof(ATE0),50);
			#endif	
		if(strstr(buff,"OK"))
		{
			#ifdef SMS_DEBUG
			HAL_UART_Transmit(&huart5,(uint8_t *)buff,strlen(buff),50);
			#endif
			memset(buff,0,sizeof(buff));
			rx_index = 0;
			break;
		}
		else
		{
			//"Error handling";
			#ifdef SMS_DEBUG
			HAL_UART_Transmit(&huart5,(uint8_t *)buff,strlen(buff),50);
			#endif
		}
	}while(err_cnt++ <= RETRY_CNT);
	HAL_Delay(1000);
	
err_cnt = 0;
	
do
{
	HAL_UART_Transmit(&huart1,(uint8_t *)AT_CMGF,sizeof(AT_CMGF),1);/* "AT+CMGF=1\r\n" select message format as text */
	HAL_Delay(2000);
	//HAL_Delay(_1000MS_DELAY); 
	#ifdef SMS_DEBUG
		HAL_UART_Transmit(&huart5,(uint8_t *)AT_CMGF,sizeof(AT_CMGF),50);
	#endif
	if(strstr(buff,"OK"))
		{
			#ifdef SMS_DEBUG
			HAL_UART_Transmit(&huart5,(uint8_t *)buff,strlen(buff),50);
			#endif
			memset(buff,0,sizeof(buff));
			rx_index = 0;
			break;
		}
		else
		{
			#ifdef SMS_DEBUG
			HAL_UART_Transmit(&huart5,(uint8_t *)buff,strlen(buff),50);
			#endif
			//"Error handling";
		}
	}while(err_cnt++ <= RETRY_CNT);
	HAL_Delay(1000);

}

void GSM_Send_Msg(char *num,char *sms)
{
	char err_cnt = 0;
	char sms_buffer[100];
	buffer_pointer=0;
	if(Status_Flag->gsm_sms_rdy && Status_Flag->gsm_sms_on)
	{
	sprintf(sms_buffer,"AT+CMGS=\"%s\"\r\n",num);
	#ifdef SMS_DEBUG
		HAL_UART_Transmit(&huart5,(uint8_t *)sms_buffer,strlen(sms_buffer),50);
	#endif
	HAL_UART_Transmit(&huart1,(uint8_t *)sms_buffer,strlen(sms_buffer),10);/*send command AT+CMGS="Mobile No."\r */
	HAL_Delay(_1000MS_DELAY);
	while(1)
	{
		if(buff[buffer_pointer]==0x3e)		/* wait for '>' character*/
		{
			buffer_pointer = 0;
			memset(buff,0,sizeof(buff));
			rx_index = 0;
					
			HAL_UART_Transmit(&huart1,(uint8_t *)sms,strlen(sms),10);/* send msg to given no. */
			HAL_Delay(_1000MS_DELAY);	
			HAL_UART_Transmit(&huart1,(uint8_t *)CTRL_Z,1,10); /* send Ctrl+Z then only message will transmit*/
			HAL_Delay(_1000MS_DELAY);
			
			err_cnt = 0;
			do
			{
				HAL_Delay(_1000MS_DELAY); 
				if(strstr(buff,"OK"))
					{
						#ifdef SMS_DEBUG
						HAL_UART_Transmit(&huart5,(uint8_t *)buff,strlen(buff),50);
						#endif
						memset(buff,0,sizeof(buff));
						rx_index = 0;
						break;
					}
					else
					{
						//"Error handling";
						//"CMS ERROR: 21" means that the mobile operator is actively rejecting/blocking this message. 
						//"CMS ERROR: 38" means that the "network is out of order"
						#ifdef SMS_DEBUG
						HAL_UART_Transmit(&huart5,(uint8_t *)buff,strlen(buff),50);
						#endif
						HAL_Delay(1000);
						if(strstr(buff,"+CMS ERROR"))
						{
							break;
						}
					}
			}while(err_cnt++ <= RETRY_CNT);
			//HAL_Delay(1000);
			break;
		}
		buffer_pointer++;
		if(buffer_pointer >= GSM_RX_BUFF_SIZE)
		{
				buffer_pointer = 0;
			break;
		}
		
	}
	//HAL_Delay(1000); //300
	buffer_pointer = 0;
	memset(buff,0,sizeof(buff));
	rx_index = 0;
	memset(sms_buffer,0,sizeof(sms_buffer));
	//HAL_Delay(1000); //500
}//Status_Flag->gsm_sms_rdy
	

}
char GSM_Wait_for_Msg(void)
{
	char msg_location[4];
	int i;
	HAL_Delay(_1000MS_DELAY);
	buffer_pointer = 0;

	memset(msg_location,0,sizeof(msg_location));
	
	for(i = 0; i < GSM_RX_BUFF_SIZE; i++)
	{
		if(buff[buffer_pointer]=='\r' || buff[buffer_pointer]== '\n')	/*eliminate "\r \n" which is start of string */
		{
			buffer_pointer++;
		}
		else
			break;
	}
	#ifdef SMS_DEBUG
	HAL_UART_Transmit(&huart5,(uint8_t *)buff,strlen(buff),50);	
	#endif
	if(strstr(buff,"CMTI:"))		/* "CMTI:" to check if any new message received */
	{
		while(buff[buffer_pointer++]!= ',')
		{
			if(buffer_pointer >= GSM_RX_BUFF_SIZE)
				break;
		}
				
		i=0;
		while((buff[buffer_pointer]!= '\r') && (buff[buffer_pointer]!= '\n'))
		{
			msg_location[i]=buff[buffer_pointer];		/* copy location of received message where it is stored */
			buffer_pointer++;
			i++;
		}
		msg_location[i] = '\0';
		#ifdef SMS_DEBUG
		HAL_UART_Transmit(&huart5,(uint8_t *)msg_location,strlen(msg_location),50);	
		HAL_UART_Transmit(&huart5,(uint8_t *)"\r\n",2,50);
		#endif
		/* convert string of position to integer value */
		position = atoi(msg_location);
		
		memset(buff,0,sizeof(buff));
		rx_index = 0;
		buffer_pointer=0;

		return 1; //true
	}
	else
	{
		return 0; //false
	}
}
void GSM_Msg_Read(int position)
{
	unsigned char err_cnt = 0;
	char read_cmd[30];
	sprintf(read_cmd,"AT+CMGR=%d\r\n",position);
	//sprintf(read_cmd,"AT+CMGL=\"REC UNREAD\"\r\n");
	#ifdef SMS_DEBUG
	HAL_UART_Transmit(&huart5,(uint8_t *)read_cmd,strlen(read_cmd),50);
	#endif
	HAL_Delay(10);
	//HAL_UART_Transmit(&huart1,(uint8_t *)read_cmd,strlen(read_cmd),1);/* read message at specified location/position */
	//HAL_Delay(100);
//	GSM_Msg_Decode();		/* decode message */
	do
	{
		HAL_Delay(10);//AT_CMGL
		HAL_UART_Transmit(&huart1,(uint8_t *)read_cmd,strlen(read_cmd),10);/* read message at specified location/position */
		HAL_Delay(600);
			
		if(strstr(buff,"OK"))
		{
			#ifdef SMS_DEBUG
			HAL_UART_Transmit(&huart5,(uint8_t *)buff,strlen(buff),50);
			#endif
			break;
		}
		else
		{
			//"Error handling";
			#ifdef SMS_DEBUG
			HAL_UART_Transmit(&huart5,(uint8_t *)buff,strlen(buff),50);
			#endif
		}
		memset(buff,0,sizeof(buff));
		rx_index = 0;
	}while(err_cnt++ <= 1);//RETRY_CNT
	
	GSM_Msg_Decode();		/* decode message */
	
}
void GSM_Msg_Delete(unsigned int position)
{
	unsigned char err_cnt;
	
	buffer_pointer=0;
	char delete_cmd[30];
	sprintf(delete_cmd,"AT+CMGD=%d\r\n",position);	/* delete message at specified position */
	HAL_UART_Transmit(&huart1,(uint8_t *)delete_cmd,strlen(delete_cmd),1);
	
	err_cnt = 0;
	do
	{
		HAL_Delay(_1000MS_DELAY); 
		if(strstr(buff,"OK"))
			{
				#ifdef SMS_DEBUG
				HAL_UART_Transmit(&huart5,(uint8_t *)buff,strlen(buff),50);
				#endif
				memset(buff,0,sizeof(buff));
				rx_index = 0;
				break;
			}
			else
			{
				#ifdef SMS_DEBUG
				HAL_UART_Transmit(&huart5,(uint8_t *)buff,strlen(buff),50);
				#endif
				//"Error handling";
			}
	}while(err_cnt++ <= RETRY_CNT);
}
void GSM_Delete_All_Msg(void)
{
	unsigned char err_cnt;
	
	#ifdef SMS_DEBUG
	HAL_UART_Transmit(&huart5,(uint8_t *)AT_CMGD,strlen(AT_CMGD),1);
	#endif
	HAL_UART_Transmit(&huart1,(uint8_t *)AT_CMGD,strlen(AT_CMGD),1); /*"AT+CMGDA=\"DEL ALL\"\r\n"   delete all messages of SIM */ 	

	err_cnt = 0;
	do
	{
		HAL_Delay(_1000MS_DELAY); 
		
		if(strstr(buff,"OK") || strstr(buff,"CMS"))
			{
				#ifdef SMS_DEBUG
				HAL_UART_Transmit(&huart5,(uint8_t *)buff,strlen(buff),50);
				#endif
				memset(buff,0,sizeof(buff));
				rx_index = 0;
				break;
			}
			else
			{
				#ifdef SMS_DEBUG
				HAL_UART_Transmit(&huart5,(uint8_t *)buff,strlen(buff),50);
				#endif
				//"Error handling";
			}
	}while(err_cnt++ <= RETRY_CNT);
				memset(buff,0,sizeof(buff));
				rx_index = 0;
}

void GSM_Msg_Decode(void)
{
	HAL_Delay(20); //500
	#ifdef SMS_DEBUG
	HAL_UART_Transmit(&huart5,(uint8_t *)buff,strlen(buff),50);
	#endif
	if(!(strstr(buff,"+CMGR")))		/*check for +CMGR response */
	{
			Status_Flag->gsm_engine_reset = SET; // To enable gsm engine

		//No message found;
//			HAL_Delay(500);
//			GSM_Send_Msg(user1,"not able to read msg");
//			HAL_Delay(500);
//			GSM_Send_Msg(user2,"not able to read msg");
			
		
	}
	else
	{
		buffer_pointer = 0;
		
		while(1)
		{
			if(buff[buffer_pointer]=='\r' || buff[buffer_pointer]== '\n')		/*wait till \r\n not over*/
			{
				buffer_pointer++;
			}
			else
			break;
		}
		
		/* search for 1st ',' to get mobile no.*/
		while(buff[buffer_pointer]!=',')
		{
			buffer_pointer++;
		}
		buffer_pointer = buffer_pointer+2;

		/* extract mobile no. of message sender */
		for(int i=0;i<=12;i++)
		{
			Mobile_no[i] = buff[buffer_pointer];
			buffer_pointer++;
		}
		
		do
		{
			buffer_pointer++;
		}while(buff[buffer_pointer-1]!= '\n');
		
		int i=0;

		/* display and save message */
		while(buff[buffer_pointer]!= '\r' && i<31)
		{
				message_received[i]=buff[buffer_pointer];
				buffer_pointer++;
				i++;
		}
		
		buffer_pointer = 0;
		memset(buff,0,sizeof(buff));
		rx_index = 0;
	}
	
}
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
void GPS_Process_Data(void)
{
		char deg[4] = { 0 };
    char min[3] = { 0 };
    char dec[5] = { 0 };

    float  latitude_f;
    float  longitude_f;

//    char latitude_buffer[16] = { 0 };
//    char longitude_buffer[16] = { 0 };
	
	
	//...............................
	char latitude[16];
	char longitude[16];
	char NS[2],EW[2];
	char valid[2];
	
	char *addr;
	char *p;
	int i = 0;
	addr = strstr(gps_buff,"$GNRMC");
	
	if(addr)
	{
			p = strtok (addr, ",");
   
    while (p != NULL)
    {
				if(i == 2)
				{
					strncpy(valid,p,1);
						
				}
				else if(i == 3)
					strcpy(latitude,p);
				else if(i == 4)
					strncpy(NS,p,1);
				
				else if(i == 5)
					strcpy(longitude,p);
				else if(i == 6)
					strncpy(EW,p,1);
				else if(i > 6)
						break;	
			
				i++;
        p = strtok (NULL, ",");
    }
	}
		
		#ifdef GPS_DEBUG
		HAL_UART_Transmit(&huart5,(uint8_t *)"\r\n",2,10);
		HAL_UART_Transmit(&huart5,(uint8_t *)valid,1,10);
		
		HAL_UART_Transmit(&huart5,(uint8_t *)"\r\n",2,10);
		HAL_UART_Transmit(&huart5,(uint8_t *)latitude,strlen(latitude),10);
		HAL_UART_Transmit(&huart5,(uint8_t *)"\r\n",2,10);
		
		HAL_UART_Transmit(&huart5,(uint8_t *)NS,1,10);
		HAL_UART_Transmit(&huart5,(uint8_t *)"\r\n",2,10);
		HAL_UART_Transmit(&huart5,(uint8_t *)longitude,strlen(longitude),10);	
		HAL_UART_Transmit(&huart5,(uint8_t *)"\r\n",2,10);
		HAL_UART_Transmit(&huart5,(uint8_t *)EW,1,10);
		HAL_UART_Transmit(&huart5,(uint8_t *)"\r\n",2,10);
		#endif
	
	if(valid[0] == 'A')
	{
		strncpy(deg, latitude, 2);
    strncpy(min, latitude + 2, 2);
    strncpy(dec, latitude + 5, 4);

    latitude_f = atoi(deg) + (atoi(min) + (atoi(dec) / 10000.0f)) / 60.0f;
		
	 if(NS[0] == 'S') latitude_f *= -1.0f;
	 
	 strncpy(deg, longitude, 3);
    strncpy(min, longitude + 3, 2);
    strncpy(dec, longitude + 6, 4);

    longitude_f = atoi(deg) + (atoi(min) + atoi(dec) / 10000.0f) / 60.0f;
	 
	  if(EW[0] == 'W') longitude_f *= -1.0f;
		
		
		sprintf(latitude_buffer,"%f",latitude_f);
		#ifdef GPS_DEBUG
		HAL_UART_Transmit(&huart5,(uint8_t *)"\r\n",2,10);
		HAL_UART_Transmit(&huart5,(uint8_t *)latitude_buffer,strlen(latitude_buffer),10);
		#endif
		sprintf(longitude_buffer,"%f",longitude_f);
		
		#ifdef GPS_DEBUG
		HAL_UART_Transmit(&huart5,(uint8_t *)"\r\n",2,10);
		HAL_UART_Transmit(&huart5,(uint8_t *)longitude_buffer,strlen(longitude_buffer),10);
		#endif
		
	}
	 
}

void GNSS_Begin(void) // gps initialization
{
	char err_cnt = 0;
	
//	do
//	{
//		HAL_UART_Transmit(&huart1,(uint8_t *)AT_CMGF,sizeof(AT_CMGF),1);
//		HAL_UART_Transmit(&huart4,(uint8_t *)AT_CMGF,sizeof(AT_CMGF),50);
//		HAL_Delay(_1000MS_DELAY); 
//		if(strstr(buff,"OK"))
//		{
//			HAL_UART_Transmit(&huart4,(uint8_t *)buff,strlen(buff),50);
//			memset(buff,0,sizeof(buff));
//			rx_index = 0;
//			break;
//		}
//		else
//		{
//			//"Error handling";
//		HAL_UART_Transmit(&huart4,(uint8_t *)buff,strlen(buff),50);
//		}
//		HAL_Delay(_1000MS_DELAY); 
//	}while(err_cnt++ <= RETRY_CNT);
//	//HAL_Delay(1000);
//	
//err_cnt = 0;
//	
//do
//{
//	HAL_UART_Transmit(&huart1,(uint8_t *)AT_CSCS,sizeof(AT_CSCS),1);
//	HAL_UART_Transmit(&huart4,(uint8_t *)AT_CSCS,sizeof(AT_CSCS),50);
//	HAL_Delay(_1000MS_DELAY); 
//	if(strstr(buff,"OK"))
//		{
//			HAL_UART_Transmit(&huart4,(uint8_t *)buff,strlen(buff),50);
//			memset(buff,0,sizeof(buff));
//			rx_index = 0;
//			break;
//		}
//		else
//		{
//			HAL_UART_Transmit(&huart4,(uint8_t *)buff,strlen(buff),50);
//			//"Error handling";
//		}
//		HAL_Delay(_1000MS_DELAY); 
//	}while(err_cnt++ <= RETRY_CNT);
//	HAL_Delay(1000);
//	
//	err_cnt = 0;
//	
//do
//{
//	HAL_UART_Transmit(&huart1,(uint8_t *)AT_CNMI1,sizeof(AT_CNMI1),1);
//	HAL_UART_Transmit(&huart4,(uint8_t *)AT_CNMI1,sizeof(AT_CNMI1),50);
//	
//	HAL_Delay(_1000MS_DELAY); 
//	if(strstr(buff,"OK"))
//		{
//			HAL_UART_Transmit(&huart4,(uint8_t *)buff,strlen(buff),50);
//			memset(buff,0,sizeof(buff));
//			rx_index = 0;
//			break;
//		}
//		else
//		{
//			HAL_UART_Transmit(&huart4,(uint8_t *)buff,strlen(buff),50);
//			//"Error handling";
//		}
//		HAL_Delay(_1000MS_DELAY); 
//	}while(err_cnt++ <= RETRY_CNT);
//	HAL_Delay(1000);
err_cnt = 0;
	
do
{
	HAL_UART_Transmit(&huart3,(uint8_t *)AT_QGNSSC_ON,sizeof(AT_QGNSSC_ON),1);
	HAL_UART_Transmit(&huart4,(uint8_t *)AT_QGNSSC_ON,sizeof(AT_QGNSSC_ON),50);
	HAL_Delay(_1000MS_DELAY); 
	if(strstr(buff,"OK"))
		{
			HAL_UART_Transmit(&huart4,(uint8_t *)buff,strlen(buff),50);
			memset(buff,0,sizeof(buff));
			rx_index = 0;
			break;
		}
		else
		{
			HAL_UART_Transmit(&huart4,(uint8_t *)buff,strlen(buff),50);
			//"Error handling";
		}
		HAL_Delay(_1000MS_DELAY); 
	}while(err_cnt++ <= RETRY_CNT);
	HAL_Delay(1000);
//err_cnt = 0;
//	
//do
//{
//	HAL_UART_Transmit(&huart1,(uint8_t *)AT_QIFGCNT,sizeof(AT_QIFGCNT),1);
//	HAL_UART_Transmit(&huart4,(uint8_t *)AT_QIFGCNT,sizeof(AT_QIFGCNT),50);
//	HAL_Delay(_1000MS_DELAY); 
//	if(strstr(buff,"OK"))
//		{
//			HAL_UART_Transmit(&huart4,(uint8_t *)buff,strlen(buff),50);
//			memset(buff,0,sizeof(buff));
//			rx_index = 0;
//			break;
//		}
//		else
//		{
//			HAL_UART_Transmit(&huart4,(uint8_t *)buff,strlen(buff),50);
//			//"Error handling";
//		}
//		HAL_Delay(_1000MS_DELAY); 
//	}while(err_cnt++ <= RETRY_CNT);
//	HAL_Delay(1000);
//err_cnt = 0;
//	
//do
//{
//	HAL_UART_Transmit(&huart1,(uint8_t *)AT_QICSGP1,sizeof(AT_QICSGP1),1);
//	HAL_UART_Transmit(&huart4,(uint8_t *)AT_QICSGP1,sizeof(AT_QICSGP1),50);
//	HAL_Delay(_1000MS_DELAY); 
//	if(strstr(buff,"OK"))
//		{
//			HAL_UART_Transmit(&huart4,(uint8_t *)buff,strlen(buff),50);
//			memset(buff,0,sizeof(buff));
//			rx_index = 0;
//			break;
//		}
//		else
//		{
//			HAL_UART_Transmit(&huart4,(uint8_t *)buff,strlen(buff),50);
//			//"Error handling";
//		}
//		HAL_Delay(_1000MS_DELAY); 
//	}while(err_cnt++ <= RETRY_CNT);
//	HAL_Delay(1000);
//err_cnt = 0;
//	
//do
//{
//	HAL_UART_Transmit(&huart1,(uint8_t *)AT_QGNSSTS,sizeof(AT_QGNSSTS),1);
//	HAL_UART_Transmit(&huart4,(uint8_t *)AT_QGNSSTS,sizeof(AT_QGNSSTS),50);
//	
//	HAL_Delay(_1000MS_DELAY); 
//	if(strstr(buff,"OK"))
//		{
//			HAL_UART_Transmit(&huart4,(uint8_t *)buff,strlen(buff),50);
//			memset(buff,0,sizeof(buff));
//			rx_index = 0;
//			break;
//		}
//		else
//		{
//			HAL_UART_Transmit(&huart4,(uint8_t *)buff,strlen(buff),50);
//			//"Error handling";
//		}
//		HAL_Delay(_1000MS_DELAY); 
//	}while(err_cnt++ <= RETRY_CNT);
//	HAL_Delay(1000);
//	
//err_cnt = 0;
//	
//do
//{
//	HAL_UART_Transmit(&huart1,(uint8_t *)AT_QGNSSEPO,sizeof(AT_QGNSSEPO),1);
//	HAL_UART_Transmit(&huart4,(uint8_t *)AT_QGNSSEPO,sizeof(AT_QGNSSEPO),50);
//	HAL_Delay(_1000MS_DELAY); 
//	if(strstr(buff,"OK"))
//		{
//			HAL_UART_Transmit(&huart4,(uint8_t *)buff,strlen(buff),50);
//			memset(buff,0,sizeof(buff));
//			rx_index = 0;
//			break;
//		}
//		else
//		{
//			HAL_UART_Transmit(&huart4,(uint8_t *)buff,strlen(buff),50);
//			//"Error handling";
//		}
//		HAL_Delay(_1000MS_DELAY); 
//	}while(err_cnt++ <= RETRY_CNT);
//	HAL_Delay(1000);

//	err_cnt = 0;
//	
//do
//{
//	HAL_UART_Transmit(&huart1,(uint8_t *)AT_QGEPOAID,sizeof(AT_QGEPOAID),1);
//	HAL_UART_Transmit(&huart4,(uint8_t *)AT_QGEPOAID,sizeof(AT_QGEPOAID),50);
//			
//	HAL_Delay(_1000MS_DELAY); 
//	if(strstr(buff,"OK"))
//		{
//			HAL_UART_Transmit(&huart4,(uint8_t *)buff,strlen(buff),50);
//			memset(buff,0,sizeof(buff));
//			rx_index = 0;
//			break;
//		}
//		else
//		{
//			HAL_UART_Transmit(&huart4,(uint8_t *)buff,strlen(buff),50);
//			//"Error handling";
//		}
//		HAL_Delay(_1000MS_DELAY); 
//	}while(err_cnt++ <= RETRY_CNT);
	HAL_Delay(1000);
	
}
void GNSS_PowerON(void)
{
	char err_cnt = 0;
	
	do
	{
		HAL_UART_Transmit(&huart3,(uint8_t *)AT_QGNSSC_ON,sizeof(AT_QGNSSC_ON),1);/* "ATE0\r\n" send ATE0 to check module is ready or not */
		HAL_UART_Transmit(&huart4,(uint8_t *)AT_QGNSSC_ON,sizeof(AT_QGNSSC_ON),50);
		HAL_Delay(_1000MS_DELAY); 
		if(strstr(buff,"OK"))
		{
			HAL_UART_Transmit(&huart4,(uint8_t *)buff,strlen(buff),50);
			memset(buff,0,sizeof(buff));
			rx_index = 0;
			break;
		}
		else
		{
			HAL_UART_Transmit(&huart4,(uint8_t *)buff,strlen(buff),50);
			//"Error handling";
			
		}
		HAL_Delay(1000);
	}while(err_cnt++ <= RETRY_CNT);
	
	//HAL_Delay(1000);
}
void GNSS_PowerOFF(void)
{
	char err_cnt = 0;
	
	do
	{
		HAL_UART_Transmit(&huart3,(uint8_t *)AT_QGNSSC_OFF,sizeof(AT_QGNSSC_OFF),1);/* "AT+CMGF=1\r\n" select message format as text */
		HAL_UART_Transmit(&huart4,(uint8_t *)AT_QGNSSC_OFF,sizeof(AT_QGNSSC_OFF),50);
		HAL_Delay(_1000MS_DELAY);
		if(strstr(buff,"OK"))
			{
				HAL_UART_Transmit(&huart4,(uint8_t *)buff,strlen(buff),50);
				memset(buff,0,sizeof(buff));
				rx_index = 0;
				break;
			}
			else
			{
				HAL_UART_Transmit(&huart4,(uint8_t *)buff,strlen(buff),50);
				//"Error handling";
			}
			HAL_Delay(1000);
	}while(err_cnt++ <= RETRY_CNT);
	//HAL_Delay(1000);
}
void GNSS_Read(void)
{
	char err_cnt = 0;
	
	do
	{
		HAL_UART_Transmit(&huart3,(uint8_t *)AT_QGNSSCRD,sizeof(AT_QGNSSCRD),10);/* "AT+CMGF=1\r\n" select message format as text */
		HAL_UART_Transmit(&huart4,(uint8_t *)AT_QGNSSCRD,sizeof(AT_QGNSSCRD),50);
		HAL_Delay(_1000MS_DELAY);
		if(strstr(buff,"OK"))
			{
				HAL_UART_Transmit(&huart4,(uint8_t *)buff,strlen(buff),50);
				memset(buff,0,sizeof(buff));
				rx_index = 0;
				break;
			}
			else
			{
				HAL_UART_Transmit(&huart4,(uint8_t *)buff,strlen(buff),50);
				//"Error handling";
			}
	}while(err_cnt++ <= RETRY_CNT);
	//HAL_Delay(1000);
	
}


void gsm_sms_fsm(void)
{
		char is_msg_arrived;
		
		is_msg_arrived = GSM_Wait_for_Msg();	/*check for message arrival*/
		if(is_msg_arrived == 1)
		{
				//UART0_SendString("New message");	/* new message arrived */
				//HAL_Delay(_1000MS_DELAY);
				GSM_Msg_Read(position);		/* read arrived message */		
				//HAL_Delay(3000);
					
					if(strstr(Mobile_no,user1) || strstr(Mobile_no,user2))
					{
						/*check conditions from  received message */
//							if(strstr( message_received,"MOTOR ON")){	
//								motor_on();												
//							}
//							else if(strstr( message_received,"MOTOR OFF")){	
//								motor_off();						
//							}
							
							
							if(!strncmp(message_received,ON_PUMP,sizeof(ON_PUMP))){
								//motor_on();
								Status_Flag->motor_on_flag = SET;
								Status_Flag->on_delay_start = SET;
							}
							else if(!strncmp(message_received,ON_PUMP1,sizeof(ON_PUMP1))){
								//motor_on();
								Status_Flag->motor_on_flag = SET;
								Status_Flag->on_delay_start = SET;
							}
							else if(!strncmp(message_received,OFF_PUMP,sizeof(OFF_PUMP))){
								//motor_off();
								Status_Flag->motor_off_flag = SET;
							}
							else if(!strncmp(message_received,OFF_PUMP1,sizeof(OFF_PUMP1))){
								//motor_off();
								Status_Flag->motor_off_flag = SET;
							}							
							else if(!strncmp(message_received,GETSTATUS,sizeof(GETSTATUS))){
								Status_Flag->get_status = SET;
							}
							else if(!strncmp(message_received,GETSETTINGS,sizeof(GETSETTINGS))){
								Status_Flag->get_settings = SET;
							}
							else if(!strncmp(message_received,SMSOFF,sizeof(SMSOFF))){
										Status_Flag->gsm_sms_on = CLEAR;
							}
							else if(!strncmp(message_received,SMSON,sizeof(SMSON))){
										Status_Flag->gsm_sms_on = SET;
							}
							else if(!strncmp(message_received,GETGPS,sizeof(GETGPS))){
										Status_Flag->get_gps = SET;
							}
							//....................commands for timers.............................
							else if(!strncmp(message_received,SETSLOT1,sizeof(SETSLOT1))){
									get_sche_time(message_received,&SCH1_START_HH,&SCH1_START_MM,&SCH1_STOP_HH,&SCH1_STOP_MM);
									Status_Flag->sch1_timer = SET;
									//store values in EEPROM
//										sprintf((char *)eeprom_wr_buf,"%2d%2d%2d%2d",SCH1_START_HH,SCH1_START_MM,SCH1_STOP_HH,SCH1_STOP_MM);
//										at24_WriteBytes(&hi2c1, 0xA0,EEPROM_SCH1_ADDR, eeprom_wr_buf,EEPROM_SCH_TIME_SIZE);	
										
										sprintf((char *)eeprom_wr_buf,"%02d",SCH1_START_HH);
										at24_WriteBytes(&hi2c1, 0xA0,EEPROM_SCH1_ADDR, eeprom_wr_buf,2);
																				
										sprintf((char *)eeprom_wr_buf,"%02d",SCH1_START_MM);
										at24_WriteBytes(&hi2c1, 0xA0,EEPROM_SCH1_ADDR + 2, eeprom_wr_buf,2);
										
										sprintf((char *)eeprom_wr_buf,"%02d",SCH1_STOP_HH);
										at24_WriteBytes(&hi2c1, 0xA0,EEPROM_SCH1_ADDR + 4, eeprom_wr_buf,2);
										
										sprintf((char *)eeprom_wr_buf,"%02d",SCH1_STOP_MM);
										at24_WriteBytes(&hi2c1, 0xA0,EEPROM_SCH1_ADDR + 6, eeprom_wr_buf,2);
								
							}
							else if(!strncmp(message_received,SETSLOT2,sizeof(SETSLOT2))){
									get_sche_time(message_received,&SCH2_START_HH,&SCH2_START_MM,&SCH2_STOP_HH,&SCH2_STOP_MM);
									Status_Flag->sch2_timer = SET;
									//store values in EEPROM
									//sprintf((char *)eeprom_wr_buf,"%2d%2d%2d%2d",SCH2_START_HH,SCH2_START_MM,SCH2_STOP_HH,SCH2_STOP_MM);
									//at24_WriteBytes(&hi2c1, 0xA0,EEPROM_SCH2_ADDR, eeprom_wr_buf,EEPROM_SCH_TIME_SIZE);
										sprintf((char *)eeprom_wr_buf,"%02d",SCH2_START_HH);
										at24_WriteBytes(&hi2c1, 0xA0,EEPROM_SCH2_ADDR, eeprom_wr_buf,2);
																				
										sprintf((char *)eeprom_wr_buf,"%02d",SCH2_START_MM);
										at24_WriteBytes(&hi2c1, 0xA0,EEPROM_SCH2_ADDR + 2, eeprom_wr_buf,2);
										
										sprintf((char *)eeprom_wr_buf,"%02d",SCH2_STOP_HH);
										at24_WriteBytes(&hi2c1, 0xA0,EEPROM_SCH2_ADDR + 4, eeprom_wr_buf,2);
										
										sprintf((char *)eeprom_wr_buf,"%02d",SCH2_STOP_MM);
										at24_WriteBytes(&hi2c1, 0xA0,EEPROM_SCH2_ADDR + 6, eeprom_wr_buf,2);
							}
							else if(!strncmp(message_received,SETSLOT3,sizeof(SETSLOT3))){
									get_sche_time(message_received,&SCH3_START_HH,&SCH3_START_MM,&SCH3_STOP_HH,&SCH3_STOP_MM);
									Status_Flag->sch3_timer = SET;
									//store values in EEPROM
									//sprintf((char *)eeprom_wr_buf,"%2d%2d%2d%2d",SCH3_START_HH,SCH3_START_MM,SCH3_STOP_HH,SCH3_STOP_MM);
									//at24_WriteBytes(&hi2c1, 0xA0,EEPROM_SCH3_ADDR, eeprom_wr_buf,EEPROM_SCH_TIME_SIZE);
										sprintf((char *)eeprom_wr_buf,"%02d",SCH3_START_HH);
										at24_WriteBytes(&hi2c1, 0xA0,EEPROM_SCH3_ADDR, eeprom_wr_buf,2);
																				
										sprintf((char *)eeprom_wr_buf,"%02d",SCH3_START_MM);
										at24_WriteBytes(&hi2c1, 0xA0,EEPROM_SCH3_ADDR + 2, eeprom_wr_buf,2);
										
										sprintf((char *)eeprom_wr_buf,"%02d",SCH3_STOP_HH);
										at24_WriteBytes(&hi2c1, 0xA0,EEPROM_SCH3_ADDR + 4, eeprom_wr_buf,2);
										
										sprintf((char *)eeprom_wr_buf,"%02d",SCH3_STOP_MM);
										at24_WriteBytes(&hi2c1, 0xA0,EEPROM_SCH3_ADDR + 6, eeprom_wr_buf,2);
							}
							else if(!strncmp(message_received,SETSLOT4,sizeof(SETSLOT4))){
									get_sche_time(message_received,&SCH4_START_HH,&SCH4_START_MM,&SCH4_STOP_HH,&SCH4_STOP_MM);
									Status_Flag->sch4_timer = SET;
									//store values in EEPROM
									//sprintf((char *)eeprom_wr_buf,"%2d%2d%2d%2d",SCH4_START_HH,SCH4_START_MM,SCH4_STOP_HH,SCH4_STOP_MM);
									//at24_WriteBytes(&hi2c1, 0xA0,EEPROM_SCH4_ADDR, eeprom_wr_buf,EEPROM_SCH_TIME_SIZE);
										sprintf((char *)eeprom_wr_buf,"%02d",SCH4_START_HH);
										at24_WriteBytes(&hi2c1, 0xA0,EEPROM_SCH4_ADDR, eeprom_wr_buf,2);
																				
										sprintf((char *)eeprom_wr_buf,"%02d",SCH4_START_MM);
										at24_WriteBytes(&hi2c1, 0xA0,EEPROM_SCH4_ADDR + 2, eeprom_wr_buf,2);
										
										sprintf((char *)eeprom_wr_buf,"%02d",SCH4_STOP_HH);
										at24_WriteBytes(&hi2c1, 0xA0,EEPROM_SCH4_ADDR + 4, eeprom_wr_buf,2);
										
										sprintf((char *)eeprom_wr_buf,"%02d",SCH4_STOP_MM);
										at24_WriteBytes(&hi2c1, 0xA0,EEPROM_SCH4_ADDR + 6, eeprom_wr_buf,2);
							}
							else if(!strncmp(message_received,DELSLOT1,sizeof(DELSLOT1))){
									Status_Flag->sch1_timer = CLEAR;
									SCH1_START_HH = SCH1_START_MM = SCH1_STOP_HH = SCH1_STOP_MM = 0x00;
								
									//write in eeprom
									//	sprintf((char *)eeprom_wr_buf,"%2d%2d%2d%2d",SCH1_START_HH,SCH1_START_MM,SCH1_STOP_HH,SCH1_STOP_MM);
									//	at24_WriteBytes(&hi2c1, 0xA0,EEPROM_SCH1_ADDR, eeprom_wr_buf,EEPROM_SCH_TIME_SIZE);	
										sprintf((char *)eeprom_wr_buf,"%02d",SCH1_START_HH);
										at24_WriteBytes(&hi2c1, 0xA0,EEPROM_SCH1_ADDR, eeprom_wr_buf,2);
																				
										sprintf((char *)eeprom_wr_buf,"%02d",SCH1_START_MM);
										at24_WriteBytes(&hi2c1, 0xA0,EEPROM_SCH1_ADDR + 2, eeprom_wr_buf,2);
										
										sprintf((char *)eeprom_wr_buf,"%02d",SCH1_STOP_HH);
										at24_WriteBytes(&hi2c1, 0xA0,EEPROM_SCH1_ADDR + 4, eeprom_wr_buf,2);
										
										sprintf((char *)eeprom_wr_buf,"%02d",SCH1_STOP_MM);
										at24_WriteBytes(&hi2c1, 0xA0,EEPROM_SCH1_ADDR + 6, eeprom_wr_buf,2);	
								
							}
							else if(!strncmp(message_received,DELSLOT2,sizeof(DELSLOT2))){
									Status_Flag->sch2_timer = CLEAR;
									SCH2_START_HH = SCH2_START_MM = SCH2_STOP_HH = SCH2_STOP_MM = 0x00;
									//write in eeprom
									//sprintf((char *)eeprom_wr_buf,"%2d%2d%2d%2d",SCH2_START_HH,SCH2_START_MM,SCH2_STOP_HH,SCH2_STOP_MM);
									//at24_WriteBytes(&hi2c1, 0xA0,EEPROM_SCH2_ADDR, eeprom_wr_buf,EEPROM_SCH_TIME_SIZE);
										sprintf((char *)eeprom_wr_buf,"%02d",SCH2_START_HH);
										at24_WriteBytes(&hi2c1, 0xA0,EEPROM_SCH2_ADDR, eeprom_wr_buf,2);
																				
										sprintf((char *)eeprom_wr_buf,"%02d",SCH2_START_MM);
										at24_WriteBytes(&hi2c1, 0xA0,EEPROM_SCH2_ADDR + 2, eeprom_wr_buf,2);
										
										sprintf((char *)eeprom_wr_buf,"%02d",SCH2_STOP_HH);
										at24_WriteBytes(&hi2c1, 0xA0,EEPROM_SCH2_ADDR + 4, eeprom_wr_buf,2);
										
										sprintf((char *)eeprom_wr_buf,"%02d",SCH2_STOP_MM);
										at24_WriteBytes(&hi2c1, 0xA0,EEPROM_SCH2_ADDR + 6, eeprom_wr_buf,2);
							}
							else if(!strncmp(message_received,DELSLOT3,sizeof(DELSLOT3))){
									Status_Flag->sch3_timer = CLEAR;
									SCH3_START_HH = SCH3_START_MM = SCH3_STOP_HH = SCH3_STOP_MM = 0x00;
									//write in eeprom
									//sprintf((char *)eeprom_wr_buf,"%2d%2d%2d%2d",SCH3_START_HH,SCH3_START_MM,SCH3_STOP_HH,SCH3_STOP_MM);
									//at24_WriteBytes(&hi2c1, 0xA0,EEPROM_SCH3_ADDR, eeprom_wr_buf,EEPROM_SCH_TIME_SIZE);
										sprintf((char *)eeprom_wr_buf,"%02d",SCH3_START_HH);
										at24_WriteBytes(&hi2c1, 0xA0,EEPROM_SCH3_ADDR, eeprom_wr_buf,2);
																				
										sprintf((char *)eeprom_wr_buf,"%02d",SCH3_START_MM);
										at24_WriteBytes(&hi2c1, 0xA0,EEPROM_SCH3_ADDR + 2, eeprom_wr_buf,2);
										
										sprintf((char *)eeprom_wr_buf,"%02d",SCH3_STOP_HH);
										at24_WriteBytes(&hi2c1, 0xA0,EEPROM_SCH3_ADDR + 4, eeprom_wr_buf,2);
										
										sprintf((char *)eeprom_wr_buf,"%02d",SCH3_STOP_MM);
										at24_WriteBytes(&hi2c1, 0xA0,EEPROM_SCH3_ADDR + 6, eeprom_wr_buf,2);
							}
							else if(!strncmp(message_received,DELSLOT4,sizeof(DELSLOT4))){
									Status_Flag->sch4_timer = CLEAR;
									SCH4_START_HH = SCH4_START_MM = SCH4_STOP_HH = SCH4_STOP_MM = 0x00;
									//write in eeprom
									//sprintf((char *)eeprom_wr_buf,"%2d%2d%2d%2d",SCH4_START_HH,SCH4_START_MM,SCH4_STOP_HH,SCH4_STOP_MM);
									//at24_WriteBytes(&hi2c1, 0xA0,EEPROM_SCH4_ADDR, eeprom_wr_buf,EEPROM_SCH_TIME_SIZE);
										sprintf((char *)eeprom_wr_buf,"%02d",SCH4_START_HH);
										at24_WriteBytes(&hi2c1, 0xA0,EEPROM_SCH4_ADDR, eeprom_wr_buf,2);
																				
										sprintf((char *)eeprom_wr_buf,"%02d",SCH4_START_MM);
										at24_WriteBytes(&hi2c1, 0xA0,EEPROM_SCH4_ADDR + 2, eeprom_wr_buf,2);
										
										sprintf((char *)eeprom_wr_buf,"%02d",SCH4_STOP_HH);
										at24_WriteBytes(&hi2c1, 0xA0,EEPROM_SCH4_ADDR + 4, eeprom_wr_buf,2);
										
										sprintf((char *)eeprom_wr_buf,"%02d",SCH4_STOP_MM);
										at24_WriteBytes(&hi2c1, 0xA0,EEPROM_SCH4_ADDR + 6, eeprom_wr_buf,2);
							}
							else if(!strncmp(message_received,SETRUNTIME,sizeof(SETRUNTIME))){
									get_timers(message_received,&T_RUN_TIME);
									//write in eeprom
									sprintf((char *)eeprom_wr_buf,"%d",T_RUN_TIME);
									at24_WriteBytes(&hi2c1, 0xA0,EEPROM_T_RUN_TIME_ADDR, eeprom_wr_buf,EEPROM_T_RUN_TIME_SIZE);
							}
							else if(!strncmp(message_received,ONDELAY,sizeof(ONDELAY))){
									get_timers(message_received,&T_ONDELAY);
									//write in eeprom
									sprintf((char *)eeprom_wr_buf,"%d",T_ONDELAY);
									at24_WriteBytes(&hi2c1, 0xA0,EEPROM_T_ONDELAY_ADDR, eeprom_wr_buf,EEPROM_T_ONDELAY_SIZE);
							}
							else if(!strncmp(message_received,DRYRUN,sizeof(DRYRUN))){
									get_timers(message_received,&T_DRYRUN);
									//write in eeprom
									sprintf((char *)eeprom_wr_buf,"%d",T_DRYRUN);
									at24_WriteBytes(&hi2c1, 0xA0,EEPROM_T_DRYRUN_ADDR, eeprom_wr_buf,EEPROM_T_DRYRUN_SIZE);
							}
							else if(!strncmp(message_received,ADDUSER1,sizeof(ADDUSER1))){
									strncpy(user1,message_received+6,10);
									//write in eeprom
							}
							else if(!strncmp(message_received,ADDUSER2,sizeof(ADDUSER2))){
									strncpy(user2,message_received+6,10);
									//write in eeprom
							}
							else if(!strncmp(message_received,DELUSER1,sizeof(DELUSER1))){
									memset(user1,0,sizeof(user1));
									//write in eeprom
							}
							else if(!strncmp(message_received,DELUSER2,sizeof(DELUSER2))){
									memset(user2,0,sizeof(user2));
									//write in eeprom
							}
							else if(!strncmp(message_received,GETUSERS,sizeof(GETUSERS))){
									Status_Flag->get_users = SET;
									//write in eeprom
							}
							else if(!strncmp(message_received,SETSUPPORT,sizeof(SETSUPPORT))){
									
									//write in eeprom
							}
							else if(!strncmp(message_received,SETMASTER,sizeof(SETMASTER))){
									
									//write in eeprom
							}
							else if(!strncmp(message_received,INSTALL,sizeof(INSTALL))){
									
									//write in eeprom
							}
							else if(!strncmp(message_received,SETFLOW,sizeof(SETFLOW))){
									
									//write in eeprom
							}
							else if(!strncmp(message_received,DISABLESER,sizeof(DISABLESER))){
									
									//write in eeprom
							}
							else if(!strncmp(message_received,DELALLSMS,sizeof(DELALLSMS))){
									
									Status_Flag->gsm_del_all_sms = SET;
							}	
							
					}
					GSM_Msg_Delete(position);		/* to save SIM memory delete current message */
//									
//				
//				//HAL_Delay(1000);					
		}											
		is_msg_arrived = 0;		
		//GSM_Delete_All_Msg();
		Status_Flag->gsm_cmd_seq = SET;
		memset(Mobile_no, 0, sizeof(Mobile_no));
		memset(message_received, 0, sizeof(message_received));	
		rx_byte_cnt = 0;
			
}
//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
void GPRS_Begin(void)
{
	char err_cnt = 0;
	
	do
	{
		HAL_UART_Transmit(&huart1,(uint8_t *)AT_QIMODE,strlen(AT_QIMODE),1);/* select TCP/IP stack mode as non-transparent */
		HAL_Delay(_1000MS_DELAY);
		if(strstr(buff,"OK"))
		{
			memset(buff,0,sizeof(buff));
			rx_index = 0;
			break;
		}
		else
		{
			//"Error handling";
		}
	}while(err_cnt++ <= RETRY_CNT);
	//HAL_Delay(1000);
	
err_cnt = 0;
do
{
	HAL_UART_Transmit(&huart1,(uint8_t *)AT_QICSGP,strlen(AT_QICSGP),1);/* set APN as CMNET */
	HAL_Delay(_1000MS_DELAY);
	if(strstr(buff,"OK"))
		{
			memset(buff,0,sizeof(buff));
			rx_index = 0;
			break;
		}
		else
		{
			//"Error handling";
		}
	}while(err_cnt++ <= RETRY_CNT);
	//HAL_Delay(1000);
	
err_cnt = 0;
do
{
	HAL_UART_Transmit(&huart1,(uint8_t *)AT_QIREGAPP,strlen(AT_QIREGAPP),1);/* start TCP/IP task */
	HAL_Delay(_1000MS_DELAY);
	if(strstr(buff,"OK"))
		{
			memset(buff,0,sizeof(buff));
			rx_index = 0;
			break;
		}
		else
		{
			//"Error handling";
		}
	}while(err_cnt++ <= RETRY_CNT);
	//HAL_Delay(1000);
	
err_cnt = 0;	
do
{
	HAL_UART_Transmit(&huart1,(uint8_t *)AT_QIACT,strlen(AT_QIACT),1);/* active the gprs context */
	HAL_Delay(_1000MS_DELAY); 
	if(strstr(buff,"OK"))
		{
			memset(buff,0,sizeof(buff));
			rx_index = 0;
			break;
		}
		else
		{
			//"Error handling";
		}
	}while(err_cnt++ <= RETRY_CNT);
	//HAL_Delay(1000);

err_cnt = 0;	
do
{
	HAL_UART_Transmit(&huart1,(uint8_t *)AT_QILOCIP,strlen(AT_QILOCIP),1);/* get the local IP address */
	HAL_Delay(_1000MS_DELAY); 
	if(strstr(buff,".")) // store the local IP address
		{
			memset(buff,0,sizeof(buff));
			rx_index = 0;
			break;
		}
		else
		{
			//"Error handling";
		}
	}while(err_cnt++ <= RETRY_CNT);
	//HAL_Delay(1000);
	
err_cnt = 0;	
do
{
	HAL_UART_Transmit(&huart1,(uint8_t *)ATV1,strlen(ATV1),1);/* set the response format */
	HAL_Delay(_1000MS_DELAY);
	if(strstr(buff,"OK")) 
		{
			memset(buff,0,sizeof(buff));
			rx_index = 0;
			break;
		}
		else
		{
			//"Error handling";
		}
	}while(err_cnt++ <= RETRY_CNT);
	//HAL_Delay(1000);
	
err_cnt = 0;	
do
{
	HAL_UART_Transmit(&huart1,(uint8_t *)AT_QIHEAD,strlen(AT_QIHEAD),1);/* add header information when receive data */
	HAL_Delay(_1000MS_DELAY); 
	if(strstr(buff,"OK")) 
		{
			memset(buff,0,sizeof(buff));
			rx_index = 0;
			break;
		}
		else
		{
			//"Error handling";
		}
	}while(err_cnt++ <= RETRY_CNT);
	//HAL_Delay(1000);
	
err_cnt = 0;	
do
{
	HAL_UART_Transmit(&huart1,(uint8_t *)AT_QIDNSIP,strlen(AT_QIDNSIP),1);/* use the IP address to establish TCP/UDP session */
	HAL_Delay(_1000MS_DELAY); 
	if(strstr(buff,"OK")) 
		{
			memset(buff,0,sizeof(buff));
			rx_index = 0;
			break;
		}
		else
		{
			//"Error handling";
		}
	}while(err_cnt++ <= RETRY_CNT);
	//HAL_Delay(1000);
	
	
}

void GPRS_Connect(void)
{
	unsigned char err_cnt = 0;	
do
{
	HAL_UART_Transmit(&huart1,(uint8_t *)AT_QIOPEN,strlen(AT_QIOPEN),10);/* connect to GPRS Server */
	HAL_Delay(_1000MS_DELAY); 
	if(strstr(buff,"CONNECT OK")) 
		{
			Status_Flag->gprs_connect_flag = SET;
			
			memset(buff,0,sizeof(buff));
			rx_index = 0;
			break;
		}
		else
		{
			//"Error handling";
		}
	}while(err_cnt++ <= RETRY_CNT);
	//HAL_Delay(1000);
	
}


void GPRS_Send_Data(char* data)
{
	char err_cnt = 0;
	buffer_pointer=0;
	HAL_UART_Transmit(&huart1,(uint8_t *)AT_QISEND,strlen(AT_QISEND),1);/*send command AT_QISEND \r */
	HAL_Delay(_1000MS_DELAY);
	while(1)
	{
		if(buff[buffer_pointer]==0x3e)		/* wait for '>' character*/
		{
			buffer_pointer = 0;
			memset(buff,0,sizeof(buff));
			rx_index = 0;
					
			HAL_UART_Transmit(&huart1,(uint8_t *)data,strlen(data),50);/* send data to server*/
			HAL_Delay(_1000MS_DELAY);	
			HAL_UART_Transmit(&huart1,(uint8_t *)CTRL_Z,1,1); /* send Ctrl+Z then only message will transmit*/
			HAL_Delay(_1000MS_DELAY);
			
			err_cnt = 0;
			do
			{
				HAL_Delay(_1000MS_DELAY); 
				if(strstr(buff,"OK"))
					{
						memset(buff,0,sizeof(buff));
						rx_index = 0;
						break;
					}
					else
					{
						//"Error handling";
					}
			}while(err_cnt++ <= RETRY_CNT);
			//HAL_Delay(1000);
			break;
		}
		buffer_pointer++;
	}
	//HAL_Delay(1000); 
	buffer_pointer = 0;
	memset(buff,0,sizeof(buff));
	rx_index = 0;
	
}
void GPRS_Disconnect(void)
{
	unsigned char err_cnt = 0;	
do
{
	HAL_UART_Transmit(&huart1,(uint8_t *)AT_QIDEACT,strlen(AT_QIDEACT),1);/* deactivate gprs context */
	HAL_Delay(_1000MS_DELAY); 
	if(strstr(buff,"DEACT OK")) 
		{
			Status_Flag->gprs_connect_flag = CLEAR;
			memset(buff,0,sizeof(buff));
			rx_index = 0;
			break;
		}
		else
		{
			//"Error handling";
		}
	}while(err_cnt++ <= RETRY_CNT);
	//HAL_Delay(1000);
}

void get_sche_time(char *addr,unsigned char *START_HH,unsigned char *START_MM,unsigned char *STOP_HH,unsigned char *STOP_MM)
{
	char *p;
	int i = 0;
	short START_TIME,STOP_TIME;
		
	p = strtok (addr, "*");
   
    while (p != NULL)
    {
				if(i == 1){
					
					START_TIME = atoi(p);			

					*START_MM = START_TIME % 100;
					*START_HH = START_TIME / 100;
					
				}
				else if(i == 2){
					STOP_TIME = atoi(p);
					
					*STOP_MM = STOP_TIME % 100;
					*STOP_HH = STOP_TIME /100;
				}
				else if(i > 2){
						break;	
				}
			
				i++;
        p = strtok (NULL, "*");
    }		
}

void get_timers(char *addr,short *TIMER)
{
	char *p;
	int i = 0;
			
	p = strtok (addr, "*");
   
    while (p != NULL)
    {
				if(i == 1){
					
					*TIMER = atoi(p);			
					
				}
				else if(i > 1){
						break;	
				}
			
				i++;
        p = strtok (NULL, "*");
    }
		
}
void GSM_get_rtc(void)
{
	char err_cnt = 0;
	char hh[2],mm[2],ss[2],dt[2],mon[2],year[2];
	char local_rtc_buff[50];
		
	do
	{
		HAL_Delay(1000);
		//HAL_Delay(_1000MS_DELAY); 
		//HAL_UART_Transmit(&huart1,(uint8_t *)"AT+CCLK?\r\n",10,3);/* AT+CCLK? to get rtc  */
		HAL_UART_Transmit(&huart1,(uint8_t *)AT_CCLK,10,3);/* AT+CCLK? to get rtc  */
		HAL_Delay(2000);
		//HAL_Delay(_1000MS_DELAY); 
		if(strstr(buff,"OK"))
		{			
			#ifdef SMS_DEBUG
			HAL_UART_Transmit(&huart5,(uint8_t *)buff,strlen(buff),50);
			#endif
			if(strstr(buff,"+CCLK"))		/* "CMTI:" to check if any new message received */
			{ 
				buffer_pointer = 0;
				while(buff[buffer_pointer++] != ':')
				{
					if(buffer_pointer >= GSM_RX_BUFF_SIZE)
						break;
				}
				buffer_pointer += 2;
				strncpy(year,buff+buffer_pointer,2);
				buffer_pointer += 3;
				strncpy(mon,buff+buffer_pointer,2);
				buffer_pointer += 3;
				strncpy(dt,buff+buffer_pointer,2);
				buffer_pointer += 3;
				
				strncpy(hh,buff+buffer_pointer,2);
				buffer_pointer += 3;
				
				strncpy(mm,buff+buffer_pointer,2);
				buffer_pointer += 3;
				
				strncpy(ss,buff+buffer_pointer,2);
				buffer_pointer += 3;
				
				pLocal_time->year = atoi(year)+2000;
				pLocal_time->month = atoi(mon);
				pLocal_time->date = atoi(dt);
				pLocal_time->hour = atoi(hh);
				pLocal_time->minute = atoi(mm);
				pLocal_time->second = atoi(ss);
				
				pLocal_time->hour -= 10;
				
				pLocal_time->minute += 30;
				if(pLocal_time->minute > 59){
					pLocal_time->minute -= 60;
					pLocal_time->hour += 1;
				}
				pLocal_time->hour += 5;
				if(pLocal_time->hour > 23){
					pLocal_time->hour -= 24;					
				}
				
				sprintf(local_rtc_buff,"\r\n %d-%d-%d,%d:%d:%d  \r\n",pLocal_time->date,pLocal_time->month,pLocal_time->year,pLocal_time->hour,pLocal_time->minute,pLocal_time->second);
					
				HAL_UART_Transmit(&huart5,(uint8_t *)local_rtc_buff,strlen(local_rtc_buff),50);
				
			}
			
			
			memset(buff,0,sizeof(buff));
			rx_index = 0;
			break;
		}
		else
		{
			//"Error handling";
			#ifdef SMS_DEBUG
			HAL_UART_Transmit(&huart5,(uint8_t *)buff,strlen(buff),50);
			#endif
			HAL_Delay(2000);
			//HAL_Delay(_1000MS_DELAY);
			//HAL_Delay(_1000MS_DELAY);
		}
	}while(err_cnt++ <= RETRY_CNT);
	
	HAL_Delay(_1000MS_DELAY);
		
}
void GSM_Signal_Strength(void)
{
	char err_cnt = 0;
	char ss[2];	
	do
	{
		HAL_Delay(1000);
		//HAL_Delay(_1000MS_DELAY); 
		HAL_UART_Transmit(&huart1,(uint8_t *)AT_CSQ,8,3);/* AT+CCLK? to get rtc  */
		HAL_Delay(1000);
		//HAL_Delay(_1000MS_DELAY); 
		if(strstr(buff,"OK"))
		{
			
			#ifdef SMS_DEBUG
			HAL_UART_Transmit(&huart5,(uint8_t *)buff,strlen(buff),50);
			#endif
			if(strstr(buff,"+CSQ"))		/* "CMTI:" to check if any new message received */
			{ 
				buffer_pointer = 0;
				while(buff[buffer_pointer++] != ':')
				{
					if(buffer_pointer >= GSM_RX_BUFF_SIZE)
						break;
				}
				buffer_pointer += 1;
							
				strncpy(ss,buff+buffer_pointer,2);
				gsm_ss = atoi(ss);
				HAL_UART_Transmit(&huart5,(uint8_t *)"\r\n",2,50);		
				HAL_UART_Transmit(&huart5,(uint8_t *)ss,strlen(ss),50);
				HAL_UART_Transmit(&huart5,(uint8_t *)"\r\n",2,50);
				
			}
					
			memset(buff,0,sizeof(buff));
			rx_index = 0;
			break;
		}
		else
		{
			//"Error handling";
			#ifdef SMS_DEBUG
			HAL_UART_Transmit(&huart5,(uint8_t *)buff,strlen(buff),50);
			#endif
			HAL_Delay(2000);
			//HAL_Delay(_1000MS_DELAY);
			//HAL_Delay(_1000MS_DELAY);
		}
	}while(err_cnt++ <= RETRY_CNT);
	HAL_Delay(1000);
	//HAL_Delay(_1000MS_DELAY);
		
}



