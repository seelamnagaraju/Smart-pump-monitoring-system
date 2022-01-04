/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  **   09262021::0948PM
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include <string.h>
#include <stdlib.h>
#include "struct_var.h"
#include "const.h"
#include "eeprom.h"
#include "flash.h"
#include "gsm.h"
#include "sams_sa9904b.h"

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;
RTC_TimeTypeDef time;
RTC_DateTypeDef date;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;

/* Private variables ---------------------------------------------------------*/
// 5mS Tick Time distribution FSM States
enum tag5mSTickStates e5mSTickState = ST_5mS_TICK1;
struct SCHED_FLAG Sched_Flags, *Sched_Flag;
struct STATUS_FLAGS Status_Flags,*Status_Flag; //flags for general conditions 
struct rtc local_time, *pLocal_time;

unsigned char rx_index_dis = 0; 
unsigned char rx_byte_dis = 0; 
char sw_stat_buff[6];
unsigned short SW_STAT;
unsigned short STAT_IND;
unsigned char first_on;
unsigned char on_counter;

char user1[12] = "9059911555";
char user2[12] = "8309348730";
char gprs_data[50] = "hi welcome to gprs";
char send_sms_buff[GSM_SEND_MSG_SIZE];
char disp_rx_buff[DB_RX_BUFF_SIZE];
char disp_tx_buff[DB_TX_BUFF_SIZE];// = "#230.00,229.50,220.00,50.00,49.80,50.00,34521,35000,34221,00021,00011,00031,89541,00212!";

unsigned int tim_5ms_cnt,tim_5ms_cnt1,gps_count,mtr_read_count;
unsigned int rtc_counter,rtc_read_counter;
unsigned int run_time_counter,on_delay_counter;
unsigned int phase_err_check_counter,no_of_on_counter,on_reenable_counter;
unsigned int sys_hlt_counter;

unsigned int sched1_100ms_counter;
unsigned int sched2_100ms_counter;
unsigned int sched3_100ms_counter;
unsigned int sched4_100ms_counter;
unsigned int sched5_100ms_counter;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI1_Init(void);
static void MX_UART4_Init(void);
static void MX_UART5_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);

//.................................................................
void motor_on(void);
void motor_off(void);
void motor_status_ind(int ,int );
void set_rtc(unsigned char,unsigned char,unsigned char,unsigned char,unsigned char,unsigned char);
void check_current_unbalance(void);
void check_voltage_unbalance(void);
void earth_fault(void);
void send_msg_to_users(char *);
void get_var_from_eeprom(void);
void get_sch_time_from_eeprom(void);

//.................................................................
/* USER CODE BEGIN 0 */
//.................................................................
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
		if(huart->Instance == USART1)
		{
				if(rx_index == 0) {
						memset(buff,0,sizeof(buff));
				}
				buff[rx_index++] = rx_byte;

				if(strstr(buff,"+CMTI:")) {
						Status_Flag->gsm_msg_rx_flag = SET;
				}
				if(Status_Flag->gprs_connect_flag) {
						if(strstr(buff,"CMD:")) {
								Status_Flag->gprs_command_flag = SET;
						}
						if(strstr(buff,"CONFIG:")) {
								Status_Flag->gprs_config_flag = SET;
						}
						if(strstr(buff,"DEACT:")) {
								Status_Flag->gprs_disconnect_flag = SET;	
						}							
				}			
				HAL_UART_Receive_IT(&huart1,&rx_byte,1);		
		}
		
		if(huart->Instance == USART3) {
				Status_Flag->gps_response_flag = SET;
		}
		
		if(huart->Instance == UART4)
		{		
				if(rx_byte_dis == '$') {
						Status_Flag->sw_status_flag = 1;
						rx_index_dis = 0;
						memset(sw_stat_buff,0,sizeof(sw_stat_buff));			
				}
				else if(Status_Flag->sw_status_flag) {				
						if(rx_byte_dis != '!') {
								sw_stat_buff[rx_index_dis++] = rx_byte_dis;//atoi(rx_byte);
						}
						else {
								SW_STAT = atoi(sw_stat_buff);							
								Status_Flag->sw_cmd_flag = SET;
								Status_Flag->sw_status_flag = CLEAR;
								rx_index_dis = 0;
								memset(sw_stat_buff,0,sizeof(sw_stat_buff));	
						}					
				}
				else if(rx_byte_dis == '#') {
							Status_Flag->update_flag = 1;
							rx_index_dis = 0;
							memset(disp_rx_buff,0,sizeof(disp_rx_buff));				
				}
				else if(Status_Flag->update_flag) {			
						if(rx_byte_dis != '!') {
								disp_rx_buff[rx_index_dis++] = rx_byte_dis;
						}
						else {
								if(strstr(disp_rx_buff,"UPDATE")) {
											Status_Flag->disp_msg_rx_flag = SET;	
								}
								Status_Flag->update_flag = 0;
								rx_index_dis = 0;
								memset(sw_stat_buff,0,sizeof(sw_stat_buff));								
						}			
			 }					
			 HAL_UART_Receive_IT(&huart4,&rx_byte_dis,1);
		}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
			if(htim->Instance == TIM2){ // 1ms interrupt
					if(tim_5ms_cnt++ >= _5MS_EVENT_PERIOD){ // 5ms 
							tim_5ms_cnt = CLEAR;
							Sched_Flag->fsm_5ms_tick = SET;
					}
					if(tim_5ms_cnt1++ >= _500MS_EVENT_PERIOD){ // 5ms 
							tim_5ms_cnt1 = CLEAR;
							Status_Flag->cpu_helth_led = SET;
							if(gps_count++ >= 120){
									gps_count = 0;
									Status_Flag->gps_read_flag = SET;
							}
							if(mtr_read_count++ >= 1){ // 100 ms
									mtr_read_count = 0;
									Status_Flag->read_meter = SET;
							}			
					}		
		  }
	
			if(htim->Instance == TIM3){ // 1ms interrupt
					if(Status_Flag->run_time_start){
							if(run_time_counter++ >= (T_RUN_TIME * 60000)){
									run_time_counter = 0;
									Status_Flag->run_time_start = CLEAR;
									Status_Flag->run_time_elapse = SET;
							}				
					}
					else{
							run_time_counter = 0;
					}		

					if(Status_Flag->on_delay_start){
							if(on_delay_counter++ >= (T_ONDELAY * 1000)){				
									on_delay_counter = 0;
									Status_Flag->on_delay_start = CLEAR;
									Status_Flag->on_delay_elapse = SET;
							}				
					}
					else{
							on_delay_counter = 0;
					}	
			
					if(rtc_read_counter++ >= 60000){
							rtc_read_counter = 0;
							Status_Flag->rtc_read = SET;
					}
					if(phase_err_check_counter++ >= 5000){
							phase_err_check_counter = 0;
							Status_Flag->phase_err_chk = SET;
					}
			
					if(Status_Flag->first_on){
							if(no_of_on_counter++ >= MAX_NO_ONS_TIME){
									no_of_on_counter = 0;	
									if(!Status_Flag->no_of_ons_exceded){
											on_counter  = 0;
											Status_Flag->first_on = CLEAR;
									}					
							}			
					}
					if(Status_Flag->no_of_ons_exceded){
							if(on_reenable_counter++ >= ON_REENABLE_TIME){
									on_reenable_counter = 0;	
									Status_Flag->no_of_ons_exceded = CLEAR;
									on_counter  = 0; 
									Status_Flag->first_on = CLEAR;
							}		
					}
			}
}

//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------
int main(void)
{
        buffer_pointer = 0;
        memset(message_received, 0, sizeof(message_received));
        Status_Flag = &Status_Flags;  
        pLocal_time = &local_time;
        Sched_Flag = &Sched_Flags; // Flags for schedular logics

        sched1_100ms_counter = _100MS_EVENT_PERIOD; // 100 ms counters initialization @25ms
        sched2_100ms_counter = _100MS_EVENT_PERIOD;
        sched3_100ms_counter = _100MS_EVENT_PERIOD;
        sched4_100ms_counter = _100MS_EVENT_PERIOD;
        sched5_100ms_counter = _100MS_EVENT_PERIOD;

		sams_meter_init();

		/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
		HAL_Init();

		/* Configure the system clock */
		SystemClock_Config();

		/* Initialize all configured peripherals */
		MX_GPIO_Init();
		MX_DMA_Init();
		MX_USART3_UART_Init();
		MX_USART1_UART_Init();
		MX_I2C1_Init();
		MX_SPI2_Init();
		MX_SPI1_Init();
		MX_UART4_Init();
		MX_UART5_Init();
		MX_RTC_Init();
		MX_TIM2_Init();
		MX_TIM3_Init();

		/* USER CODE BEGIN 2 */
		HAL_UART_Receive_IT(&huart1,&rx_byte,1);
		HAL_UART_Receive_IT(&huart4,&rx_byte_dis,1);
		HAL_TIM_Base_Start_IT(&htim2);
		HAL_TIM_Base_Start_IT(&htim3);
		HAL_UART_Receive_DMA(&huart3, (uint8_t *)gps_buff, sizeof(gps_buff));

		//SMS Ready
		//Call Ready
		//bn GPRS_Begin();
		//bn GPRS_Connect();


		//GSM_Send_Msg("9059911555","hitestsms");
		//HAL_Delay(500);

		//HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET);
		//HAL_Delay(10000);
		//HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET);
		//HAL_Delay(10000);

		//for (int loop = 0; loop <= 8; loop += 2)
		//{
		//	STAT_IND |= (0x01 << loop);  // to set bit position 1
		//						/* store the value in EEPROM*/
		//				
		//	//local_buff[1] = STAT_IND;
		//	sprintf(local_buff,"$%d!",STAT_IND);
		//	HAL_UART_Transmit(&huart4,(uint8_t *)local_buff,sizeof(local_buff),10);
		//	
		//	HAL_Delay(10000);
		//}

		//strcpy((char *)eeprom_wr_buf,"HI"); 
		//	//............................................................
		//	at24_WriteBytes(&hi2c1, 0xA0, 0, eeprom_wr_buf,2);//EEPROM_WORD_SIZE
		//  at24_ReadBytes(&hi2c1, 0xA0, 0, eeprom_rd_buf, 2); //EEPROM_WORD_SIZE
		//	
		//	strcpy((char *)eeprom_wr_buf,"HI1234"); 
		//	at24_WriteBytes(&hi2c1, 0xA0, 8, eeprom_wr_buf,EEPROM_WORD_SIZE);
		//  at24_ReadBytes(&hi2c1, 0xA0, 8, eeprom_rd_buf, EEPROM_WORD_SIZE);
		//	strcpy((char *)eeprom_wr_buf,"HI1234asd6"); 
		//	at24_WriteBytes(&hi2c1, 0xA0, 16, eeprom_wr_buf,10);
		//  at24_ReadBytes(&hi2c1, 0xA0, 16, eeprom_rd_buf, 10);


		//............................................................
		//	flash_id = sFLASH_ReadID();/* Get SPI Flash ID */
		//	
		//	/* Perform a write in the Flash followed by a read of the written data */
		//  /* Erase SPI FLASH Sector to write on */
		//	sFLASH_EraseSector(FLASH_SECTOR_TO_ERASE);
		//	
		//	 /* Write Tx_Buffer data to SPI FLASH memory */
		//   sFLASH_WriteBuffer(flash_wr_buf, FLASH_WRITE_ADDRESS, F_PAGE_SIZE);

		//    /* Read data from SPI FLASH memory */
		//   sFLASH_ReadBuffer(flash_rd_buf, FLASH_READ_ADDRESS, F_PAGE_SIZE);
		//............................................................
		get_var_from_eeprom();
		get_sch_time_from_eeprom();
		HAL_Delay(1000);
		//..............................................
		Status_Flag->gsm_engine_reset = SET; // To enable gsm engine
		GSM_Engine_RESET();
		//set_rtc(12,30,45,11,04,18); //set RTC time

		//	SCH1_START_HH = 2;
		//	sprintf((char *)eeprom_wr_buf,"%02d",SCH1_START_HH);
		//	at24_WriteBytes(&hi2c1, 0xA0,EEPROM_SCH1_ADDR, eeprom_wr_buf,2);
		//	
		//	SCH1_START_MM = 5;
		//	sprintf((char *)eeprom_wr_buf,"%02d",SCH1_START_MM);
		//	at24_WriteBytes(&hi2c1, 0xA0,EEPROM_SCH1_ADDR + 2, eeprom_wr_buf,2);
		//	SCH1_STOP_HH  = 8;
		//	sprintf((char *)eeprom_wr_buf,"%02d",SCH1_STOP_HH);
		//	at24_WriteBytes(&hi2c1, 0xA0,EEPROM_SCH1_ADDR + 4, eeprom_wr_buf,2);
		//	SCH1_STOP_MM  = 3;
		//	sprintf((char *)eeprom_wr_buf,"%02d",SCH1_STOP_MM);
		//	at24_WriteBytes(&hi2c1, 0xA0,EEPROM_SCH1_ADDR + 6, eeprom_wr_buf,2);



		//sprintf((char *)eeprom_wr_buf,"%2d%2d%2d%2d",SCH1_START_HH,SCH1_START_MM,SCH1_STOP_HH,SCH1_STOP_MM);
		//at24_WriteBytes(&hi2c1, 0xA0,EEPROM_SCH1_ADDR, eeprom_wr_buf,EEPROM_SCH_TIME_SIZE);



		/* Infinite loop */
		while (1)
		{	               
            //-----------------------------------------------------------------
            //		Handle 5mS Tick events
            //-----------------------------------------------------------------
            if(Sched_Flag->fsm_5ms_tick) {
                Sched_Flag->fsm_5ms_tick = CLEAR;

                switch (e5mSTickState)
                {
                    //-------------------------------
                    //			5mS Tick 1
                    //-------------------------------
                    case ST_5mS_TICK1:					
                            Sched_Flag->sched1_25ms_tick = SET;
                            e5mSTickState = ST_5mS_TICK2;			// bump state for next tick
                            break;

                    //-------------------------------
                    //			5mS Tick 2
                    //-------------------------------
                    case ST_5mS_TICK2:
                        Sched_Flag->sched2_25ms_tick = SET;
                        e5mSTickState = ST_5mS_TICK3;			// bump state for next tick
                        break;

                    //-------------------------------
                    //			5mS Tick 3
                    //-------------------------------
                    case ST_5mS_TICK3:                      // check for 25mS event overrun
                        Sched_Flag->sched3_25ms_tick = SET;
                        e5mSTickState = ST_5mS_TICK4;			// bump state for next tick
                        break;

                    //-------------------------------
                    //			5mS Tick 4
                    //-------------------------------
                    case ST_5mS_TICK4:                     // check for 25mS event overrun
                        Sched_Flag->sched4_25ms_tick = SET;
                        e5mSTickState = ST_5mS_TICK5;			// bump state for next tick
                        break;

                    //-------------------------------
                    //			5mS Tick 5
                    //-------------------------------
                    case ST_5mS_TICK5:                     // check for 25mS event overrun
                        Sched_Flag->sched5_25ms_tick = SET;
                        e5mSTickState = ST_5mS_TICK1;			// bump state for next tick
                        break;
				} //switch (e5mSTickState)			 
            }//if(Sched_Flag->fsm_5ms_tick)
            
            //-----------------------------------------------------------------
            //		Handle 25mS Tick events
            //-----------------------------------------------------------------
            // these events are tied to 25mS ticks, and the use of multiple 25mS tick flags forces them to be distributed in time

            if(Sched_Flag->sched1_25ms_tick) {
                Sched_Flag->sched1_25ms_tick = CLEAR;
                sched1_100ms_counter--;
                if((sched1_100ms_counter == ZERO)  || (sched1_100ms_counter > _100MS_EVENT_PERIOD)) {
                    Sched_Flag->sched1_100ms_tick = SET;
                    sched1_100ms_counter = _100MS_EVENT_PERIOD;
                }	
                //....................................................
                HAL_GPIO_TogglePin(CPU_LED_GPIO_Port, CPU_LED_Pin); // cpu led toggle for every 25ms 
                //....................................................
                continue;
            }
            if(Sched_Flag->sched2_25ms_tick) {
                Sched_Flag->sched2_25ms_tick = CLEAR;	 
                sched2_100ms_counter--;
                if((sched2_100ms_counter == ZERO)  || (sched2_100ms_counter > _100MS_EVENT_PERIOD)) {
                    Sched_Flag->sched2_100ms_tick = SET;
                    sched2_100ms_counter = _100MS_EVENT_PERIOD;
                }
                if(Status_Flag->gsm_engine_reset) {
                    Status_Flag->gsm_engine_reset = CLEAR;
                    //GSM_Engine_RESET();
                }

                if(strstr(buff,"SMS Ready")) {
                    #ifdef SMS_DEBUG
                        HAL_UART_Transmit(&huart5,(uint8_t *)buff,strlen(buff),50);
                    #endif
                    Status_Flag->gsm_sms_rdy = SET;
                    memset(buff,0,sizeof(buff));
                    rx_index = 0;
                }
                if(strstr(buff,"Call Ready")) {
                    #ifdef SMS_DEBUG
                        HAL_UART_Transmit(&huart5,(uint8_t *)buff,strlen(buff),50);
                    #endif
                    memset(buff,0,sizeof(buff));
                    rx_index = 0;
                }             
                continue;
            }        

           //...................................................................
            if(Sched_Flag->sched3_25ms_tick)
            {
                Sched_Flag->sched3_25ms_tick = CLEAR;
                if(Status_Flag->disp_msg_rx_flag) // response to display board request  
                {
                    Status_Flag->disp_msg_rx_flag = CLEAR;
                    rx_index_dis = 0;
                    memset(disp_rx_buff,0,sizeof(disp_rx_buff));					

                    /*sprintf(disp_tx_buff,"#%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%1.3f,%1.3f,%d!",
                    cal_val_buff[0],cal_val_buff[1],cal_val_buff[2],cal_val_buff[3],cal_val_buff[4],cal_val_buff[5],cal_val_buff[6],cal_val_buff[7],
                    cal_val_buff[8],cal_val_buff[9],cal_val_buff[10],cal_val_buff[11],cal_val_buff[12],cal_val_buff[13],cal_val_buff[14],cal_val_buff[15],
                    cal_val_buff[16],cal_val_buff[17],cal_val_buff[18],STAT_IND);*/

                    sprintf(disp_tx_buff,"#%1.1fU,%1.1fU,%1.1fU,%1.1fU,%1.2fA,%1.2fA,%1.2fA,%1.2fA,%1.2f,%1.2f,%1.2f,%1.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.1fH2,%1.3f,%1.3f,%d!",
                    cal_val_buff[0],cal_val_buff[1],cal_val_buff[2],cal_val_buff[3],cal_val_buff[4],cal_val_buff[5],cal_val_buff[6],cal_val_buff[7],
                    cal_val_buff[8],cal_val_buff[9],cal_val_buff[10],cal_val_buff[11],cal_val_buff[12],cal_val_buff[13],cal_val_buff[14],cal_val_buff[15],
                    cal_val_buff[16],cal_val_buff[17],cal_val_buff[18],STAT_IND);

                    HAL_UART_Transmit(&huart4,(uint8_t *)disp_tx_buff,strlen(disp_tx_buff),50);			
                }

                //store cumm.kwh values in to eeprom
                sprintf((char *)eeprom_wr_buf,"%1.3f",cal_val_buff[17]);
                at24_WriteBytes(&hi2c1, 0xA0,EEPROM_CUM_KWH_ADDR, eeprom_wr_buf,EEPROM_CUM_KWH_SIZE);
                sched3_100ms_counter--;
                if((sched3_100ms_counter == ZERO)  || (sched3_100ms_counter > _100MS_EVENT_PERIOD)) {
                    Sched_Flag->sched3_100ms_tick = SET;
                    sched3_100ms_counter = _100MS_EVENT_PERIOD;
                }
                continue;
            }
            //..................................................
            if(Sched_Flag->sched4_25ms_tick)
            {
                Sched_Flag->sched4_25ms_tick = CLEAR;
                if(Status_Flag->sw_cmd_flag) {
                    Status_Flag->sw_cmd_flag = CLEAR;
                    if(SW_STAT & 0x01){
                        //motor on
                        Status_Flag->motor_on_flag = SET;
                        Status_Flag->on_delay_start = SET;
                    }
                    else{ 
                        // motor off
                        Status_Flag->motor_off_flag = SET;
                    }
                    if((SW_STAT >> 1) & 0x01){
                        // auto mode
                        strcpy((char *)eeprom_wr_buf,"1"); 
                        at24_WriteBytes(&hi2c1, 0xA0,EEPROM_AM_ADDR, eeprom_wr_buf,EEPROM_AM_SIZE);
                    }
                    else{
                        //manual mode
                        strcpy((char *)eeprom_wr_buf,"0"); 
                        at24_WriteBytes(&hi2c1, 0xA0,EEPROM_AM_ADDR, eeprom_wr_buf,EEPROM_AM_SIZE);
                    }
                }	
             
                sched4_100ms_counter--;
                if((sched4_100ms_counter == ZERO)  || (sched4_100ms_counter > _100MS_EVENT_PERIOD)) {
                    Sched_Flag->sched4_100ms_tick = SET;
                    sched4_100ms_counter = _100MS_EVENT_PERIOD;
                }
            continue;
        }
            
        //..................................................			 
        if(Sched_Flag->sched5_25ms_tick) {
            Sched_Flag->sched5_25ms_tick = CLEAR;
            if(Status_Flag->run_time_elapse || Status_Flag->motor_off_flag || Status_Flag->over_voltage_off || Status_Flag->under_voltage_off) {
                // motor off after full run time
                Status_Flag->run_time_elapse = CLEAR;
                Status_Flag->motor_off_flag = CLEAR;
                Status_Flag->sch1_timer_on = CLEAR;
                Status_Flag->sch2_timer_on = CLEAR;
                Status_Flag->sch3_timer_on = CLEAR;
                Status_Flag->sch4_timer_on = CLEAR;
                Status_Flag->under_voltage_off = CLEAR;	
                Status_Flag->over_voltage_off = CLEAR;		
                motor_off();	
                motor_status_ind(TIMER_MODE,OFF);
            }          
            if(Status_Flag->on_delay_elapse && Status_Flag->motor_on_flag && (!Status_Flag->over_voltage) && (!Status_Flag->under_voltage)){
                Status_Flag->on_delay_elapse = CLEAR;
                Status_Flag->motor_on_flag = CLEAR;
                motor_on();
            }
                //:::::::::::::::::::::::::::::::::::::::::::::::::::::::::		 
             sched5_100ms_counter--;
             if((sched5_100ms_counter == ZERO)  || (sched5_100ms_counter > _100MS_EVENT_PERIOD)) {
                Sched_Flag->sched5_100ms_tick = SET;
                sched5_100ms_counter = _100MS_EVENT_PERIOD;
             }
             //GSM_Signal_Strength();
             //meter_read_fsm(); // meter ic read            
             continue;
        }
        
		//-----------------------------------------------------------------
		//		Handle 100mS Tick events
		//-----------------------------------------------------------------
		// these events are tied to 100mS ticks, and the use of multiple 100mS tick flags forces them to be distributed in time		 
		if(Sched_Flag->sched1_100ms_tick) {  // For Meter Module
			 Sched_Flag->sched1_100ms_tick = CLEAR;		 						 
			 meter_read_fsm(); // meter ic read
			 
			// sprintf(disp_tx_buff,"#%1.2f,%1.2f,%1.2f,%1.2f,%1.2f,%1.2f,%1.2f,%1.2f,%1.2f,%1.2f,%1.2f,%1.2f,%1.3f,%1.3f!\n",parameter_value[0],parameter_value[1],parameter_value[2],parameter_value[3],parameter_value[4],parameter_value[5],parameter_value[6],parameter_value[7],parameter_value[8],parameter_value[9],parameter_value[10],parameter_value[11],parameter_value[12],parameter_value[13]);
            //	 HAL_UART_Transmit(&huart5,(uint8_t *)disp_tx_buff,strlen(disp_tx_buff),20);
			//................sending sms upon phase errors................................
			if(Status_Flag->no_phase_err != Status_Flag->pre_no_phase_err) {  // No phase error			 
                if(Status_Flag->no_phase_err) {
                    motor_status_ind(MOTOR_FAULT,OFF);  //send SMS               
                    #ifdef METER_DEBUG
                        sprintf(val_buff,"no phase error\n ");
                        HAL_UART_Transmit(&huart5,(uint8_t *)val_buff,strlen(val_buff),50);
                    #endif
                    send_msg_to_users("no phase error");
                    HAL_Delay(1000);
                }
                Status_Flag->pre_no_phase_err = Status_Flag->no_phase_err;		
            }
			if(Status_Flag->phase_seq_err != Status_Flag->pre_phase_seq_err)  { //phase seq error		
                if(Status_Flag->phase_seq_err) {
                    motor_status_ind(MOTOR_FAULT,ON);
                    //send SMS
                    #ifdef METER_DEBUG
                        sprintf(val_buff,"phase seq error\n ");
                        HAL_UART_Transmit(&huart5,(uint8_t *)val_buff,strlen(val_buff),50);
                    #endif
                    send_msg_to_users("phase seq error");
                    HAL_Delay(1000);
                }
                Status_Flag->pre_phase_seq_err = Status_Flag->phase_seq_err;
            }
			if(Status_Flag->missing_phase_err != Status_Flag->pre_missing_phase_err)  {//phase missing error		
				 if(Status_Flag->missing_phase_err)
				 {
					 motor_status_ind(MOTOR_FAULT,ON);		
					 #ifdef METER_DEBUG
						sprintf(val_buff,"phase missing error\n ");
						HAL_UART_Transmit(&huart5,(uint8_t *)val_buff,strlen(val_buff),50);
					 #endif
					 send_msg_to_users("phase missing error");  //send SMS
					 HAL_Delay(1000);					 
				 }
				 Status_Flag->pre_missing_phase_err = Status_Flag->missing_phase_err;						 
			}
		
            if(Status_Flag->phase_missing_state != Status_Flag->pre_phase_missing_state) {				 
				switch(Status_Flag->phase_missing_state)
				{
					case 0:						 
                        break;
					case 1:
                        #ifdef METER_DEBUG
                            sprintf(val_buff,"phase 1 missing\n ");
                            HAL_UART_Transmit(&huart5,(uint8_t *)val_buff,strlen(val_buff),50);
                        #endif
                        send_msg_to_users("phase 1 missing");	
                        break;
                    case 2:
                        #ifdef METER_DEBUG
                            sprintf(val_buff,"phase 2 missing\n ");
                            HAL_UART_Transmit(&huart5,(uint8_t *)val_buff,strlen(val_buff),50);
                        #endif
                        send_msg_to_users("phase 2 missing");
                        break;
                    case 3:
                        #ifdef METER_DEBUG
                            sprintf(val_buff,"phase 1,2 missing\n ");
                            HAL_UART_Transmit(&huart5,(uint8_t *)val_buff,strlen(val_buff),50);
                        #endif
                        send_msg_to_users("phase 1,2 missing");
                        break;
                   case 4:
                        #ifdef METER_DEBUG
                            sprintf(val_buff,"phase 3 missing\n ");
                            HAL_UART_Transmit(&huart5,(uint8_t *)val_buff,strlen(val_buff),50);
                        #endif
                        send_msg_to_users("phase 3 missing");
                        break;
                   case 5:
                        #ifdef METER_DEBUG
                            sprintf(val_buff,"phase 1,3 missing\n ");
                            HAL_UART_Transmit(&huart5,(uint8_t *)val_buff,strlen(val_buff),50);
                        #endif
                        send_msg_to_users("phase 1,3 missing");
                        break;
                   case 6:
                        #ifdef METER_DEBUG
                            sprintf(val_buff,"phase 2,3 missing\n ");
                            HAL_UART_Transmit(&huart5,(uint8_t *)val_buff,strlen(val_buff),50);
                        #endif
                        send_msg_to_users("phase 2,3 missing");
                        break;
                   case 7:
                        #ifdef METER_DEBUG
                            sprintf(val_buff,"phase 1,2,3 missing\n ");
                            HAL_UART_Transmit(&huart5,(uint8_t *)val_buff,strlen(val_buff),50);
                        #endif
                        send_msg_to_users("phase 1,2,3 missing");
                        break;	
                }
                HAL_Delay(500);	
                Status_Flag->pre_phase_missing_state = Status_Flag->phase_missing_state;
            }		 		 	 
			 continue;
	    }
        //..................................................................		
		if(Sched_Flag->sched2_100ms_tick)  { // For GSM Module
			 Sched_Flag->sched2_100ms_tick = CLEAR;
			 if(Status_Flag->gsm_msg_rx_flag){ /*check if any new message received */
				 #ifdef SMS_DEBUG
					HAL_UART_Transmit(&huart5,(uint8_t *)buff,strlen(buff),50);
				 #endif
				 gsm_sms_fsm();// sms handling 
				 Status_Flag->gsm_msg_rx_flag = CLEAR;
			}
            if(Status_Flag->gprs_connect_flag) { // gprs data handling  
                // we can send data to the server 
                if(Status_Flag->gprs_command_flag) {// process command from server and send response to server
                    Status_Flag->gprs_command_flag = CLEAR;
                    memset(buff,0,sizeof(buff));
                    rx_index = 0;
                    Status_Flag->gprs_update_flag = SET; //for testing
                }
                if(Status_Flag->gprs_config_flag) { // process and update configuration data from the server  
                    Status_Flag->gprs_config_flag = CLEAR;
                    memset(buff,0,sizeof(buff));
                    rx_index = 0;
                    Status_Flag->gprs_update_flag = SET; //for testing
                }
                if(Status_Flag->gprs_update_flag)  { // update data to the server         
                    Status_Flag->gprs_update_flag = CLEAR;
                    GPRS_Send_Data(gprs_data);
                }
                if(Status_Flag->gprs_disconnect_flag) {
                    Status_Flag->gprs_disconnect_flag = CLEAR;
                    Status_Flag->gprs_connect_flag = CLEAR;
                    memset(buff,0,sizeof(buff));
                    rx_index = 0;
                    GPRS_Disconnect();
                }
            }
            continue;
        }
        //..................................................
		if(Sched_Flag->sched3_100ms_tick) { // For Motor Control operations
            Sched_Flag->sched3_100ms_tick = CLEAR;
            #ifdef METER_DEBUG
                sprintf(disp_tx_buff,"#%1.2f,%1.2f,%1.2f,%1.2f,%1.2f,%1.2f,%1.2f,%1.2f,%1.2f,%1.2f,%1.2f,%1.2f,%1.3f,%1.3f!\n",parameter_value[0],parameter_value[1],parameter_value[2],parameter_value[3],parameter_value[4],parameter_value[5],parameter_value[6],parameter_value[7],parameter_value[8],parameter_value[9],parameter_value[10],parameter_value[11],parameter_value[12],parameter_value[13]);		 
                HAL_UART_Transmit(&huart5,(uint8_t *)disp_tx_buff,strlen(disp_tx_buff),20);
            #endif
			//........................OVER and UNDER VOLTAGE conditions........................
/*			//if(!Status_Flag->over_voltage_alert && ((cal_val_buff[0] >= (OVER_VOLTAGE - 2)) || (cal_val_buff[1] >= (OVER_VOLTAGE - 2)) || (cal_val_buff[2] >= (OVER_VOLTAGE - 2)))){
			if(!Status_Flag->over_voltage_alert && (cal_val_buff[3] >= (OVER_VOLTAGE - 17))){	// 477 - 17 = 460 for alert 
                    Status_Flag->over_voltage_alert = SET;	
                    sprintf(send_sms_buff,"over voltage alert");					
                    send_msg_to_users(send_sms_buff);
			}
	
	
			//if(!Status_Flag->over_voltage && ((cal_val_buff[0] >= OVER_VOLTAGE) || (cal_val_buff[1] >= OVER_VOLTAGE) || (cal_val_buff[2] >= OVER_VOLTAGE))){
			if(!Status_Flag->over_voltage && (cal_val_buff[3] >= OVER_VOLTAGE)){
                    Status_Flag->over_voltage = SET;
                    Status_Flag->over_voltage_off = SET;
                    sprintf(send_sms_buff,"over voltage cutoff");					
                    send_msg_to_users(send_sms_buff);						
			}
            else if(Status_Flag->over_voltage && (cal_val_buff[3] >= OVER_VOLTAGE - 17)){
				// nothing to do, same state for this condition  			
			}
			else if(Status_Flag->over_voltage && (cal_val_buff[3] <= OVER_VOLTAGE - 17)){
					Status_Flag->over_voltage = CLEAR;
					Status_Flag->over_voltage_alert = CLEAR;
			}
				
			//if(!Status_Flag->under_voltage_alert && ((cal_val_buff[0] <= (UNDER_VOLTAGE + 2)) || (cal_val_buff[1] <= (UNDER_VOLTAGE + 2)) || (cal_val_buff[2] <= (UNDER_VOLTAGE + 2)))){
			if(!Status_Flag->under_voltage_alert && (cal_val_buff[3] <= (UNDER_VOLTAGE + 17))){
					Status_Flag->under_voltage_alert = SET;
                    sprintf(send_sms_buff,"under voltage alert");					
                    send_msg_to_users(send_sms_buff);
			}
			
			//if(!Status_Flag->under_voltage && ((cal_val_buff[0] <= UNDER_VOLTAGE) || (cal_val_buff[1] <= UNDER_VOLTAGE) || (cal_val_buff[2] <= UNDER_VOLTAGE))){
			if(!Status_Flag->under_voltage && (cal_val_buff[3] <= UNDER_VOLTAGE)){
					Status_Flag->under_voltage = SET;
					Status_Flag->under_voltage_off = SET;	
					sprintf(send_sms_buff,"under voltage cutoff");					
					send_msg_to_users(send_sms_buff);	
			}
			else if(Status_Flag->under_voltage && (cal_val_buff[3] <= UNDER_VOLTAGE + 17)){
                    // nothing to do, same state for this condition 
			}
			else if(Status_Flag->under_voltage && (cal_val_buff[3] >= UNDER_VOLTAGE + 17)){
					Status_Flag->under_voltage = CLEAR;
					Status_Flag->under_voltage_alert = CLEAR;
			}
			//........................OVER and UNDER LOAD conditions........................
			//if(!Status_Flag->over_load_alert && ((cal_val_buff[4] >= OVER_LOAD) || (cal_val_buff[5] >= OVER_LOAD) || (cal_val_buff[6] >= OVER_LOAD))){
			if(!Status_Flag->over_load_alert && (cal_val_buff[7] >= OVER_LOAD)){
                    Status_Flag->over_load_alert = SET;	
                    sprintf(send_sms_buff,"over load alert");					
                    send_msg_to_users(send_sms_buff);
			}
			else{
					Status_Flag->over_load_alert = CLEAR;
			}		
			//if(!Status_Flag->over_load && ((cal_val_buff[4] >= THERMAL_OVERLOAD) || (cal_val_buff[5] >= THERMAL_OVERLOAD) || (cal_val_buff[6] >= THERMAL_OVERLOAD))){
			if(!Status_Flag->over_load && (cal_val_buff[7] >= THERMAL_OVERLOAD)){
                    Status_Flag->over_load = SET;
                    Status_Flag->over_load_off = SET;
			}
			else if(Status_Flag->over_load && (cal_val_buff[7] >= OVER_LOAD)){
                    // nothing to do, same state for this condition 
			}
			else if(Status_Flag->over_load && (cal_val_buff[7] <= OVER_LOAD)){				
                    Status_Flag->over_load = CLEAR;						
			}		
			//if(!Status_Flag->under_load_alert && ((cal_val_buff[4] <= UNDER_LOAD) || (cal_val_buff[5] <= UNDER_LOAD) || (cal_val_buff[6] <= UNDER_LOAD))){
			if(!Status_Flag->under_load_alert && (cal_val_buff[7] <= UNDER_LOAD + 5)){		
                    Status_Flag->under_load_alert = SET;				
                    sprintf(send_sms_buff,"under load alert");					
                    send_msg_to_users(send_sms_buff);
			}		
			if(!Status_Flag->under_load && (cal_val_buff[7] <= UNDER_LOAD)){
                    Status_Flag->under_load = SET;
                    Status_Flag->under_load_off = SET;
			}
			else if(Status_Flag->under_load && (cal_val_buff[7] <= UNDER_LOAD + 5)){
				// nothing to do, same state for this condition
			}
			else if(Status_Flag->under_load && (cal_val_buff[7] >= UNDER_LOAD + 5)){
                    Status_Flag->under_load = CLEAR;
                    Status_Flag->under_load_alert = CLEAR;
			}
			if((cal_val_buff[7] >= LOCKED_ROTOR)){
				// send locked motor fault indicaion to display board
			    // fail indication
			}		
			check_current_unbalance();
			check_voltage_unbalance();
			
			if(cal_val_buff[16] >= OVER_FREQ){
				// send indication to display board
			}
			else if(cal_val_buff[16] <= UNDER_FREQ){
				// send indication to display board	
			}		
			if(!(STAT_IND & 0x01)) { // if motor is off 
					earth_fault();
            }
			if(!Status_Flag->no_of_ons_exceded && (on_counter >= MAX_NO_ONS)){
				Status_Flag->no_of_ons_exceded = SET;
				//Status_Flag->on_reenable = SET;		
			}
*/			
            //.............................................................................. 			 
		    if(Status_Flag->get_status){
                char mode_type[6];				 
                Status_Flag->get_status = CLEAR;
                if(SW_STAT & 0x20){ // 2nd bit position
                    strcpy(mode_type,"AUTO");
                }
                else{
                    strcpy(mode_type,"MANUAL");
                }				

                //freq,avg.voltage,total kW,cumm.kwh,avg pf
                sprintf(send_sms_buff,"%1.2f,%1.2f,%1.2f,%1.2f,%1.2f,%s",cal_val_buff[16],cal_val_buff[3],cal_val_buff[22],cal_val_buff[17],cal_val_buff[15],mode_type);
                send_msg_to_users(send_sms_buff);			
            }
            if(Status_Flag->get_settings){
                char mode_type[6];					 
                Status_Flag->get_settings = CLEAR;
                if(Status_Flag->gsm_sms_on){ 
                    strcpy(mode_type,"SMSON");
                }
                else{
                    strcpy(mode_type,"SMSOFF");
                }	
                sprintf(send_sms_buff,"S1*%2d%2d*%2d%2d*,S2*%2d%2d*%2d%2d*,S3*%2d%2d*%2d%2d*,S4*%2d%2d*%2d%2d*,%d,%d,%d,%s",SCH1_START_HH,SCH1_START_MM,SCH1_STOP_HH,SCH1_STOP_MM,SCH2_START_HH,SCH2_START_MM,SCH2_STOP_HH,SCH2_STOP_MM,SCH3_START_HH,SCH3_START_MM,SCH3_STOP_HH,SCH3_STOP_MM,SCH4_START_HH,SCH4_START_MM,SCH4_STOP_HH,SCH4_STOP_MM,T_RUN_TIME,T_ONDELAY,T_DRYRUN,mode_type);
                send_msg_to_users(send_sms_buff);
            }
            if(Status_Flag->get_gps){
                Status_Flag->get_gps = CLEAR;				 
                sprintf(send_sms_buff,"https://www.google.com/maps/place/%s,%s",latitude_buffer,longitude_buffer);
                send_msg_to_users(send_sms_buff);
            }
            if(Status_Flag->get_users){
                Status_Flag->get_users = CLEAR;
                sprintf(send_sms_buff,"user1:%s,user2:%s",user1,user2);					
                send_msg_to_users(send_sms_buff);
            }
            continue;
		}
        //......................................................
        if(Sched_Flag->sched4_100ms_tick) {
            Sched_Flag->sched4_100ms_tick = CLEAR;			
            if(Status_Flag->gps_response_flag) {
                Status_Flag->gps_response_flag = CLEAR;
                HAL_Delay(1000);
                #ifdef GPS_DEBUG
                HAL_UART_Transmit(&huart5,(uint8_t *)gps_buff,sizeof(gps_buff),3000);	
                #endif
                // process data
                GPS_Process_Data();
                memset(gps_buff,0,sizeof(gps_buff));
            }
            if(Status_Flag->gps_read_flag) {
                Status_Flag->gps_read_flag = CLEAR;
                HAL_UART_Receive_DMA(&huart3, (uint8_t *)gps_buff, sizeof(gps_buff));
            }
            continue;
	    }
        //......................................................		
        if(Sched_Flag->sched5_100ms_tick)  { //For RTC and Schedulers
       
            Sched_Flag->sched5_100ms_tick = CLEAR;
            //  HAL_GPIO_TogglePin(CPU_LED_GPIO_Port, CPU_LED_Pin); // cpu led toggle for every 100ms 
            //.............................................
            //			 if(rtc_counter++ >= 10) // for every 1 sec, system rtc will read and compare with schedular logics 
            //			 {	
            //					rtc_counter = 0;
            //					//get RTC time
            //					HAL_RTC_GetTime(&hrtc, &time, RTC_FORMAT_BIN);
            //					HAL_RTC_GetDate(&hrtc, &date, RTC_FORMAT_BIN);
            //				 
            //			 }
            //..............................................
            if(sys_hlt_counter++ >= 50) { // for every 5 sec
                sys_hlt_counter = 0;
                // add all system helth conditions in this if()
                if(Status_Flag->no_phase_err)
                {
                    motor_status_ind(SYS_HEALTH,SLOW_BLINK);
                }
                else{
                    motor_status_ind(SYS_HEALTH,OFF);
                }
			}		
			if(Status_Flag->gsm_sms_rdy && Status_Flag->rtc_read){
                Status_Flag->rtc_read = CLEAR;
                GSM_get_rtc();		
                HAL_GPIO_TogglePin(GPIOC,gsm_ss_Pin);

                if(pLocal_time->minute == 0  || pLocal_time->minute == 30)
                        Status_Flag->gsm_ss_read = SET;
			}
			if(Status_Flag->gsm_ss_read) {
				 Status_Flag->gsm_ss_read = CLEAR;
				 GSM_Signal_Strength();
				 if(gsm_ss < 12){
					 Status_Flag->gsm_ss_low = SET;
				 }
				 else{
						 Status_Flag->gsm_ss_low = CLEAR;
				 }					 			 
			}			 
			if(Status_Flag->gsm_del_all_sms) {
				 Status_Flag->gsm_del_all_sms = CLEAR;
				 GSM_Delete_All_Msg();				 
			}
			 
			if(Status_Flag->motor_auto_mode_on_flag) {
                Status_Flag->motor_auto_mode_on_flag = CLEAR;
                Status_Flag->motor_on_flag = SET;
                Status_Flag->on_delay_start = SET;	
			}
		
			if(Status_Flag->sch1_timer){		 
				 if((pLocal_time->minute == SCH1_START_MM) &&	(pLocal_time->hour == SCH1_START_HH) && (!Status_Flag->sch1_timer_on)){
					 // motor on
					 Status_Flag->sch1_timer_on = SET;
					 Status_Flag->motor_on_flag = SET;
					 Status_Flag->on_delay_start = SET;
					 motor_status_ind(TIMER_MODE,ON);					 
				 }			 
				 if((pLocal_time->minute == SCH1_STOP_MM) &&	(pLocal_time->hour == SCH1_STOP_HH) && Status_Flag->sch1_timer_on){
					 // motor off
                    Status_Flag->sch1_timer_on = CLEAR;
                    Status_Flag->motor_off_flag = SET;
                    motor_status_ind(TIMER_MODE,OFF);
				 }				 
			}
            if(Status_Flag->sch2_timer){
                if((pLocal_time->minute == SCH2_START_MM) &&	(pLocal_time->hour == SCH2_START_HH) && (!Status_Flag->sch2_timer_on)){
                    // motor on
                    Status_Flag->sch2_timer_on = SET;
                    Status_Flag->motor_on_flag = SET;
                    Status_Flag->on_delay_start = SET;
                    motor_status_ind(TIMER_MODE,ON);
                }

                if((pLocal_time->minute == SCH2_STOP_MM) &&	(pLocal_time->hour == SCH2_STOP_HH) && Status_Flag->sch2_timer_on){
                    // motor off
                    Status_Flag->sch2_timer_on = CLEAR;
                    Status_Flag->motor_off_flag = SET;
                    motor_status_ind(TIMER_MODE,OFF);
                }
            }
            if(Status_Flag->sch3_timer){
                if((pLocal_time->minute == SCH3_START_MM) &&	(pLocal_time->hour == SCH3_START_HH) && (!Status_Flag->sch3_timer_on)){
                    // motor on
                    Status_Flag->sch3_timer_on = SET;
                    Status_Flag->motor_on_flag = SET;
                    Status_Flag->on_delay_start = SET;
                    motor_status_ind(TIMER_MODE,ON);
                }

                if((pLocal_time->minute == SCH3_STOP_MM) &&	(pLocal_time->hour == SCH3_STOP_HH) && Status_Flag->sch3_timer_on){
                    // motor off
                    Status_Flag->sch3_timer_on = CLEAR;
                    Status_Flag->motor_off_flag = SET;
                    motor_status_ind(TIMER_MODE,OFF);
                }
            }
            if(Status_Flag->sch4_timer){
                if((pLocal_time->minute == SCH4_START_MM) &&	(pLocal_time->hour == SCH4_START_HH) && (!Status_Flag->sch4_timer_on)){
                    // motor on
                    Status_Flag->sch4_timer_on = SET;
                    Status_Flag->motor_on_flag = SET;
                    Status_Flag->on_delay_start = SET;
                    motor_status_ind(TIMER_MODE,ON);
                }
                if((pLocal_time->minute == SCH4_STOP_MM) &&	(pLocal_time->hour == SCH4_STOP_HH) && Status_Flag->sch4_timer_on){
                    // motor off
                    Status_Flag->sch4_timer_on = CLEAR;
                    Status_Flag->motor_off_flag = SET;
                    motor_status_ind(TIMER_MODE,OFF);
                }
            }
            continue;
        }
        
    } //while(1)
}

//--------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------
/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV5;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.Prediv1Source = RCC_PREDIV1_SOURCE_PLL2;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL2.PLL2State = RCC_PLL2_ON;
  RCC_OscInitStruct.PLL2.PLL2MUL = RCC_PLL2_MUL10;
  RCC_OscInitStruct.PLL2.HSEPrediv2Value = RCC_HSE_PREDIV2_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

    /**Configure the Systick interrupt time 
    */
  __HAL_RCC_PLLI2S_ENABLE();

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}


//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------
/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
}


//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------
/* RTC init function */
static void MX_RTC_Init(void)
{

    /**Initialize RTC Only 
    */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
}


//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------
/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
}

//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------
/* SPI2 init function */
static void MX_SPI2_Init(void)
{

  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
}

//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------
/* TIM2 init function */
static void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 35999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
}

//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------
/* TIM3 init function */
static void MX_TIM3_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 35999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
}

//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------
/* UART4 init function */
static void MX_UART4_Init(void)
{
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
}


//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------
/* UART5 init function */
static void MX_UART5_Init(void)
{

  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
}

//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------
/* USART1 init function */
static void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART3 init function */
static void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
}

//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------
/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

}


//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------
/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, relay_ctrl_Pin|gsm_ss_Pin|gsm_pwr_en_Pin|gsm_pwr_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CPU_LED_Pin|gps_pwr_en_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : relay_ctrl_Pin gsm_ss_Pin gsm_pwr_en_Pin gsm_pwr_Pin */
  GPIO_InitStruct.Pin = relay_ctrl_Pin|gsm_ss_Pin|gsm_pwr_en_Pin|gsm_pwr_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : CPU_LED_Pin gps_pwr_en_Pin SPI2_CS_Pin */
  GPIO_InitStruct.Pin = CPU_LED_Pin|gps_pwr_en_Pin|SPI2_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI1_CS_Pin */
  GPIO_InitStruct.Pin = SPI1_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI1_CS_GPIO_Port, &GPIO_InitStruct);

}


//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------
void set_rtc(unsigned char hh,unsigned char mm,unsigned char ss,unsigned char date,unsigned char month,unsigned char year)
{//set RTC time
		RTC_TimeTypeDef time1;
		RTC_DateTypeDef date1;
		
	time1.Hours = hh;
  time1.Minutes = mm;
  time1.Seconds = ss;
  HAL_RTC_SetTime(&hrtc, &time1, RTC_FORMAT_BIN);
 
  //date.WeekDay = RTC_WEEKDAY_THURSDAY;
		
	switch(month)	
	{
		case 1:
			date1.Month = RTC_MONTH_JANUARY;
			break;
		case 2:
			date1.Month = RTC_MONTH_FEBRUARY;
			break;
		case 3:
			date1.Month = RTC_MONTH_MARCH;
			break;
		case 4:
			date1.Month = RTC_MONTH_APRIL;
			break;
		case 5:
			date1.Month = RTC_MONTH_MAY;
			break;
		case 6:
			date1.Month = RTC_MONTH_JUNE;
			break;
		case 7:
			date1.Month = RTC_MONTH_JULY;
			break;
		case 8:
			date1.Month = RTC_MONTH_AUGUST;
			break;
		case 9:
			date1.Month = RTC_MONTH_SEPTEMBER;
			break;
		case 10:
			date1.Month = RTC_MONTH_OCTOBER;
			break;
		case 11:
			date1.Month = RTC_MONTH_NOVEMBER;
			break;
		case 12:
			date1.Month = RTC_MONTH_DECEMBER;
			break;		
	}
	//date.Month = RTC_MONTH_FEBRUARY;
  date1.Date = date;
  date1.Year = year;
  HAL_RTC_SetDate(&hrtc, &date1, RTC_FORMAT_BIN);
}


//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------
void motor_on(void)
{
		char local_buff[6];
	
		if(Status_Flag->no_phase_err)
		{	
			if(!Status_Flag->first_on){
					Status_Flag->first_on = SET;
					no_of_on_counter = 0;
			}
			on_counter += 1;
			on_reenable_counter = 0;
			
			if(!Status_Flag->no_of_ons_exceded){
			Status_Flag->run_time_start = SET;
			
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET);
			HAL_Delay(100);
			
			STAT_IND |= (0x01 << 0);  // to set bit position 1
				
			/* store the value in EEPROM*/
			sprintf(local_buff,"$%d!",STAT_IND);
			HAL_UART_Transmit(&huart4,(uint8_t *)local_buff,strlen(local_buff),10);
			
			send_msg_to_users("motor on");
			
			strcpy((char *)eeprom_wr_buf,"1"); 
			at24_WriteBytes(&hi2c1, 0xA0,EEPROM_ON_OFF_ADDR, eeprom_wr_buf,EEPROM_ON_OFF_SIZE);
			}
		}
}


//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------
void motor_off(void)
{
			char local_buff[6];
	
			if(Status_Flag->no_phase_err)
			{
				
				Status_Flag->run_time_start = CLEAR;
				
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET);
				
				STAT_IND &= ~(0x01 << 0); // to clear bit position 1
			
				/* store the value in EEPROM*/
		
				sprintf(local_buff,"$%d!",STAT_IND);
				HAL_UART_Transmit(&huart4,(uint8_t *)local_buff,strlen(local_buff),10);
				
				send_msg_to_users("motor off");
				
				strcpy((char *)eeprom_wr_buf,"1"); 
				at24_WriteBytes(&hi2c1, 0xA0,EEPROM_ON_OFF_ADDR, eeprom_wr_buf,EEPROM_ON_OFF_SIZE);
				
			}	
}


//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------
void motor_status_ind(int position,int status){
	char local_buff[6];	
	
	if(position == MOTOR_ON_OFF){
					if(status == OFF){
								STAT_IND &= ~(0x01 << 1); // to clear bit position 1
								STAT_IND &= ~(0x01 << 0); // to clear bit position 0
					}
					else if(status == ON){
								STAT_IND &= ~(0x01 << 1); // to clear bit position 1
								STAT_IND |= (0x01 << 0);  // to set bit position 0
					}
					else if(status == SLOW_BLINK){
								STAT_IND |= (0x01 << 1);  // to set bit position 1
								STAT_IND &= ~(0x01 << 0); // to clear bit position 0
					}
					else if(status == FAST_BLINK){
								STAT_IND |= (0x01 << 1);  // to set bit position 1
								STAT_IND |= (0x01 << 0);  // to set bit position 0				
					}
	}
	else if(position == MOTOR_FAULT){
					if(status == OFF){
								STAT_IND &= ~(0x01 << 3); // to clear bit position 3
								STAT_IND &= ~(0x01 << 2); // to clear bit position 2
					}
					else if(status == ON){
								STAT_IND &= ~(0x01 << 3); // to clear bit position 3
								STAT_IND |= (0x01 << 2);  // to set bit position 2
					}
					else if(status == SLOW_BLINK){
								STAT_IND |= (0x01 << 3);  // to set bit position 3
								STAT_IND &= ~(0x01 << 2); // to clear bit position 2
					}
					else if(status == FAST_BLINK){
								STAT_IND |= (0x01 << 3);  // to set bit position 3
								STAT_IND |= (0x01 << 2);  // to set bit position 2				
					}
	}
	else if(position == SYS_HEALTH){
					if(status == OFF){
								STAT_IND &= ~(0x01 << 5); // to clear bit position 5
								STAT_IND &= ~(0x01 << 4); // to clear bit position 4
					}
					else if(status == ON){
								STAT_IND &= ~(0x01 << 5); // to clear bit position 5
								STAT_IND |= (0x01 << 4);  // to set bit position 4
					}
					else if(status == SLOW_BLINK){
								STAT_IND |= (0x01 << 5);  // to set bit position 5
								STAT_IND &= ~(0x01 << 4); // to clear bit position 4
					}
					else if(status == FAST_BLINK){
								STAT_IND |= (0x01 << 5);  // to set bit position 5
								STAT_IND |= (0x01 << 4);  // to set bit position 4				
					}
	}
	else if(position == TIMER_MODE){
					if(status == OFF){
								STAT_IND &= ~(0x01 << 7); // to clear bit position 7
								STAT_IND &= ~(0x01 << 6); // to clear bit position 6
					}
					else if(status == ON){
								STAT_IND &= ~(0x01 << 7); // to clear bit position 7
								STAT_IND |= (0x01 << 6);  // to set bit position 6
					}
					else if(status == SLOW_BLINK){
								STAT_IND |= (0x01 << 7);  // to set bit position 7
								STAT_IND &= ~(0x01 << 6); // to clear bit position 6
					}
					else if(status == FAST_BLINK){
								STAT_IND |= (0x01 << 7);  // to set bit position 7
								STAT_IND |= (0x01 << 6);  // to set bit position 6			
					}
	}
	else if(position == SMS_GPRS){
					if(status == OFF){
								STAT_IND &= ~(0x01 << 9); // to clear bit position 9
								STAT_IND &= ~(0x01 << 8); // to clear bit position 8
					}
					else if(status == ON){
								STAT_IND &= ~(0x01 << 9); // to clear bit position 9
								STAT_IND |= (0x01 << 8);  // to set bit position 8
					}
					else if(status == SLOW_BLINK){
								STAT_IND |= (0x01 << 9);  // to set bit position 9
								STAT_IND &= ~(0x01 << 8); // to clear bit position 8
					}
					else if(status == FAST_BLINK){
								STAT_IND |= (0x01 << 9);  // to set bit position 9
								STAT_IND |= (0x01 << 8);  // to set bit position 8				
					}
	}
	else if(position == SIG_STR){
					if(status == OFF){
								STAT_IND &= ~(0x01 << 11); // to clear bit position 11
								STAT_IND &= ~(0x01 << 10); // to clear bit position 10
					}
					else if(status == ON){
								STAT_IND &= ~(0x01 << 11); // to clear bit position 11
								STAT_IND |= (0x01 << 10);  // to set bit position 10
					}
					else if(status == SLOW_BLINK){
								STAT_IND |= (0x01 << 11);  // to set bit position 11
								STAT_IND &= ~(0x01 << 10); // to clear bit position 10
					}
					else if(status == FAST_BLINK){
								STAT_IND |= (0x01 << 11);  // to set bit position 11
								STAT_IND |= (0x01 << 10);  // to set bit position 10				
					}
	}	
		
	sprintf(local_buff,"$%d!",STAT_IND);
	HAL_UART_Transmit(&huart4,(uint8_t *)local_buff,strlen(local_buff),10);
	
}

void check_current_unbalance(void)
{
		float ry_current_diff,yb_current_diff,rb_current_diff;
		float large_diff;	
	
		ry_current_diff = cal_val_buff[7] - cal_val_buff[4];
		yb_current_diff = cal_val_buff[7] - cal_val_buff[5];
		rb_current_diff = cal_val_buff[7] - cal_val_buff[6];
	
		if(ry_current_diff < (float)0.0){
				ry_current_diff *= -1;
		}
		if(yb_current_diff < (float)0.0){
				yb_current_diff *= -1;
		}
		if(rb_current_diff < (float)0.0){
				rb_current_diff *= -1;
		}
		
		ry_current_diff = (ry_current_diff / cal_val_buff[7]) * 100; 
		yb_current_diff = (yb_current_diff / cal_val_buff[7]) * 100; 
		rb_current_diff = (rb_current_diff / cal_val_buff[7]) * 100; 
	
//		if(ry_current_diff >= CURRENT_UNBALANCE  || yb_current_diff >= CURRENT_UNBALANCE || rb_current_diff >= CURRENT_UNBALANCE){
//					// send indication to the display board
//					// fault cutoff the motor
//		}
//		if(ry_current_diff >= CURRENT_UNBALANCE){
//				// send indication to the display board
//				// fault cutoff the motor
//		}
//		if(yb_current_diff >= CURRENT_UNBALANCE){
//				// send indication to the display board
//				// fault cutoff the motor
//		}
//		if(rb_current_diff >= CURRENT_UNBALANCE){
//				// send indication to the display board
//				// fault cutoff the motor
//		}

(ry_current_diff >= yb_current_diff) ? (large_diff = ry_current_diff) : (large_diff = yb_current_diff) ;
	(large_diff >= rb_current_diff) ? (large_diff = large_diff) : (large_diff = rb_current_diff) ;
	
	if(large_diff >= CURRENT_UNBALANCE){
			// send indication to the display board (CUB)
			// fault cutoff the motor
		}		
}


//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------
void check_voltage_unbalance(void)
{
		float ry_voltage_diff,yb_voltage_diff,rb_voltage_diff;
		float large_diff;
		
		ry_voltage_diff = cal_val_buff[3] - cal_val_buff[0];
		yb_voltage_diff = cal_val_buff[3] - cal_val_buff[1];
		rb_voltage_diff = cal_val_buff[3] - cal_val_buff[2];
	
		if(ry_voltage_diff < (float)0.0){
				ry_voltage_diff *= -1;
		}
		if(yb_voltage_diff < (float)0.0){
				yb_voltage_diff *= -1;
		}
		if(rb_voltage_diff < (float)0.0){
				rb_voltage_diff *= -1;
		}
		
		ry_voltage_diff = (ry_voltage_diff / cal_val_buff[3]) * 100; 
		yb_voltage_diff = (yb_voltage_diff / cal_val_buff[3]) * 100; 
		rb_voltage_diff = (rb_voltage_diff / cal_val_buff[3]) * 100; 
		
		
		
//		if(ry_voltage_diff >= VOLTAGE_UNBALANCE  || yb_voltage_diff >= VOLTAGE_UNBALANCE || rb_voltage_diff >= VOLTAGE_UNBALANCE){
//					// send indication to the display board
//					// fault cutoff the motor
//		}
//		if(ry_voltage_diff >= VOLTAGE_UNBALANCE){
//			// send indication to the display board
//			// fault cutoff the motor
//		}
//		if(yb_voltage_diff >= VOLTAGE_UNBALANCE){
//			// send indication to the display board
//			// fault cutoff the motor
//		}
//		if(rb_voltage_diff >= VOLTAGE_UNBALANCE){
//			// send indication to the display board
//			// fault cutoff the motor
//		}

	(ry_voltage_diff >= yb_voltage_diff) ? (large_diff = ry_voltage_diff) : (large_diff = yb_voltage_diff) ;
	(large_diff >= rb_voltage_diff) ? (large_diff = large_diff) : (large_diff = rb_voltage_diff) ;
	
	
	
	if(large_diff >= VOLTAGE_UNBALANCE){
			// send indication to the display board (UUB)
			// fault cutoff the motor
		}		
}


//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------
void earth_fault(void){
	float tot_currents;
	tot_currents = cal_val_buff[4] + cal_val_buff[5] + cal_val_buff[6];
	
	
		
	if(tot_currents != (float)0.0){
		// send indication to display board
	}
		
}
	

//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------
void send_msg_to_users(char *msg){
	
		GSM_Send_Msg(user1,msg);
		GSM_Send_Msg(user2,msg);
}

//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------
void get_var_from_eeprom(void){	
	
	at24_ReadBytes(&hi2c1, 0xA0,EEPROM_AM_ADDR, eeprom_rd_buf, EEPROM_AM_SIZE);
	if(atoi((char *)eeprom_rd_buf)){
		at24_ReadBytes(&hi2c1, 0xA0,EEPROM_ON_OFF_ADDR, eeprom_rd_buf, EEPROM_ON_OFF_SIZE);
		if(atoi((char *)eeprom_rd_buf)){
			
					Status_Flag->motor_auto_mode_on_flag = SET;
						
		}		
	}
	
	memset(eeprom_wr_buf,0,sizeof(eeprom_wr_buf));
	at24_WriteBytes(&hi2c1, 0xA0,EEPROM_CUM_KWH_ADDR, eeprom_wr_buf,EEPROM_CUM_KWH_SIZE);
	
	//for cumm.kwh
	at24_ReadBytes(&hi2c1, 0xA0,EEPROM_CUM_KWH_ADDR, eeprom_rd_buf, EEPROM_CUM_KWH_SIZE);
	energy_A += atof((char *)eeprom_rd_buf);
	
	at24_ReadBytes(&hi2c1, 0xA0,EEPROM_T_RUN_TIME_ADDR, eeprom_rd_buf, EEPROM_T_RUN_TIME_SIZE);
	T_RUN_TIME = atoi((char *)eeprom_rd_buf);
	
	at24_ReadBytes(&hi2c1, 0xA0,EEPROM_T_ONDELAY_ADDR, eeprom_rd_buf, EEPROM_T_ONDELAY_SIZE);
	T_ONDELAY = atoi((char *)eeprom_rd_buf);
	
	at24_ReadBytes(&hi2c1, 0xA0,EEPROM_T_DRYRUN_ADDR, eeprom_rd_buf, EEPROM_T_DRYRUN_SIZE);
	T_DRYRUN = atoi((char *)eeprom_rd_buf);
}


//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------
void get_sch_time_from_eeprom(void){
	char split_buff[16];
	at24_ReadBytes(&hi2c1, 0xA0,EEPROM_SCH1_ADDR, eeprom_rd_buf, EEPROM_SCH_TIME_SIZE);
	
	strncpy(split_buff,(char *)eeprom_rd_buf+0,2);
	SCH1_START_HH = atoi(split_buff);
	strncpy(split_buff,(char *)eeprom_rd_buf+2,2);
	SCH1_START_MM = atoi(split_buff);
	strncpy(split_buff,(char *)eeprom_rd_buf+4,2);
	SCH1_STOP_HH = atoi(split_buff);
	strncpy(split_buff,(char *)eeprom_rd_buf+6,2);
	SCH1_STOP_MM = atoi(split_buff);
	
	at24_ReadBytes(&hi2c1, 0xA0,EEPROM_SCH2_ADDR, eeprom_rd_buf, EEPROM_SCH_TIME_SIZE);
	
	strncpy(split_buff,(char *)eeprom_rd_buf+0,2);
	SCH2_START_HH = atoi(split_buff);
	strncpy(split_buff,(char *)eeprom_rd_buf+2,2);
	SCH2_START_MM = atoi(split_buff);
	strncpy(split_buff,(char *)eeprom_rd_buf+4,2);
	SCH2_STOP_HH = atoi(split_buff);
	strncpy(split_buff,(char *)eeprom_rd_buf+6,2);
	SCH2_STOP_MM = atoi(split_buff);
	
	at24_ReadBytes(&hi2c1, 0xA0,EEPROM_SCH3_ADDR, eeprom_rd_buf, EEPROM_SCH_TIME_SIZE);
	strncpy(split_buff,(char *)eeprom_rd_buf+0,2);
	SCH3_START_HH = atoi(split_buff);
	strncpy(split_buff,(char *)eeprom_rd_buf+2,2);
	SCH3_START_MM = atoi(split_buff);
	strncpy(split_buff,(char *)eeprom_rd_buf+4,2);
	SCH3_STOP_HH = atoi(split_buff);
	strncpy(split_buff,(char *)eeprom_rd_buf+6,2);
	SCH3_STOP_MM = atoi(split_buff);
	
	at24_ReadBytes(&hi2c1, 0xA0,EEPROM_SCH4_ADDR, eeprom_rd_buf, EEPROM_SCH_TIME_SIZE);
	strncpy(split_buff,(char *)eeprom_rd_buf+0,2);
	SCH4_START_HH = atoi(split_buff);
	strncpy(split_buff,(char *)eeprom_rd_buf+2,2);
	SCH4_START_MM = atoi(split_buff);
	strncpy(split_buff,(char *)eeprom_rd_buf+4,2);
	SCH4_STOP_HH = atoi(split_buff);
	strncpy(split_buff,(char *)eeprom_rd_buf+6,2);
	SCH4_STOP_MM = atoi(split_buff);
	
	if((SCH1_START_HH != 0x00) && (SCH1_START_MM != 0x00) && (SCH1_STOP_HH != 0x00) && (SCH1_STOP_MM != 0x00)){
		Status_Flag->sch1_timer = SET;
	}
	if((SCH2_START_HH != 0x00) && (SCH2_START_MM != 0x00) && (SCH2_STOP_HH != 0x00) && (SCH2_STOP_MM != 0x00)){
		Status_Flag->sch2_timer = SET;
	}
	if((SCH3_START_HH != 0x00) && (SCH3_START_MM != 0x00) && (SCH3_STOP_HH != 0x00) && (SCH3_STOP_MM != 0x00)){
		Status_Flag->sch3_timer = SET;
	}
	if((SCH4_START_HH != 0x00) && (SCH4_START_MM != 0x00) && (SCH4_STOP_HH != 0x00) && (SCH4_STOP_MM != 0x00)){
		Status_Flag->sch4_timer = SET;
	}
	
}


//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------
/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}
#endif


//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------
/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
