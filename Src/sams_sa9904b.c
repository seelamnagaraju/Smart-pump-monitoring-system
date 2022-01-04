#include "sams_sa9904b.h"
#include <string.h>
#include <stdlib.h>
#include <math.h> 
#include "struct_var.h"

extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;

extern char disp_tx_buff[DB_TX_BUFF_SIZE];
extern unsigned char STAT_IND;
extern struct STATUS_FLAGS Status_Flags,*Status_Flag;

char val_buff[300];
unsigned char phase_error_status;
struct phase_data phase[3]; //0,1,2
volatile long	new_val;  	//present register value 		
volatile long	delta_val;	//difference between consecutive readings
volatile signed long	active_accumulator;	//accumulates active energy
volatile signed long	reactive_accumulator;	//  "      re-active energy
volatile long test;			//temp storage for labs() use
long  threshold_A;		//active    energy threshold 
long	threshold_R;		//re-active energy threshold
float energy_A, energy_R;		//active/re-active energy
float parameter_value[METER_TOT_REG];
float cal_val_buff[METER_TOT_PARAMS]; // 20
float act_buf[3];
float react_buf[3];

//-----------------------------------------------------------------
void sams_meter_init(void)
{
	int loop;
	active_accumulator   = 0;
	reactive_accumulator = 0;

	for(loop = 0;loop < 3;loop++)	//initialise data variables
	{
		phase[loop].voltage  = 0;
		phase[loop].frequency= 0;
		phase[loop].first_A= OFF;
		phase[loop].first_R= OFF;
	}

	energy_A=0.0; //4 energy display
	energy_R=0.0;

	threshold_A= THRES;		
	threshold_R= THRES;
}
//tx 9 bit address 1bit/(rising edge of spi clock), MSb 1st.
//must rx on falling edge of clock...
//rx data 1bit/clock, MSb 1st.
/*
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_RESET);  //SPI3_CS Low
		HAL_SPI_Transmit(&hspi3,spi3_tx_buff,sizeof(spi3_tx_buff),50);
		HAL_SPI_Receive(&hspi3,spi3_rx_buff,sizeof(spi3_rx_buff),50);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_SET);  //SPI3_CS High
*/

long acquire(unsigned int addr)
{
 long	input;	//the spi return variable
 unsigned char	in[3],	//buffer for received bytes
	              out[2];
// int sloop;
	
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_SET);  //SPI1_CS High
	//for (sloop = 0; sloop >= 10;sloop++);
// --- tx address ---
//The format is : 0000 0001 10A5A4 A3A2A1A0
    addr |= 0x180;			       	//or in 'read' command
		out[1] = addr |= 0x80;
		out[0] = 0x01;
 
		
	HAL_SPI_Transmit(&hspi1,out,2,60);
// --- rx data ---
//The format is : D23 ... D0
    input = 0;			 	       	//init var
	
	
	HAL_SPI_Receive(&hspi1,in,3,60);

//reconstruct variable
	input = (in[0] << 16) | (in[1] << 8) | in[2];
				
  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_RESET);  //SPI1_CS Low	

//	for (sloop = 0; sloop >= 10;sloop++);
 return(input);
}
//next fcn works checks register wrapping between (consecutive) readings
//feed fcn with (new_val-old_val) from get_data(),
//then just 'save' result in accumulator...
long consumption(long value)
{
 signed char sign;
  
  test=labs(value);
  if( test > 0x7fffff)    	 	 //val>8388607?
   {
     if(value>0) sign=-1;
            else sign= 1;
     value=(0x1000000+(sign*value))*sign; //(16777216(+or-)value)*(+or-)
   }
  return (value);
}
//..........................................................
void process_a_data(void)
{

  test=labs(active_accumulator);
  if(test > threshold_A)
  {
  //bn   if((!aLED)&&(oA_LED))	aLED=ON;//if led not on signal that it must be 
    if(active_accumulator > 0)  active_accumulator-= threshold_A;
     else  			active_accumulator+= threshold_A;

   //energy_A+= (float)0.001;
		//energy_A+= (float)0.020408;
		//energy_A+= (float)0.0128;
		//energy_A+= (float)0.00232;	
		//	energy_A+= (float)0.0023188;	
		energy_A+= (float)0.00262723792;
  }
}

void process_r_data(void)
{
  test=labs(reactive_accumulator);
  if( test > threshold_R)
  {
   //bn if((!rLED)&&(oR_LED))	rLED=ON; //if led not on signal that it must be 
    if(reactive_accumulator > 0) reactive_accumulator-= threshold_R;
       else                      reactive_accumulator+= threshold_R;

    energy_R+=(float)0.001;
  }
}
long labs(long val) 
{
  if(val<0) 
            return(-val);
  return(val);
}

void data_assign(void)
{	
	int cnt = 0,loop = 0;
	float temp = 0;
	
	/*
	for (loop = 0;loop <= 2; loop++)
	{
		temp  =(float)(phase[loop].voltage);
		temp /=(float)(730);
		temp *=(float)(230);
		parameter_value[cnt++] = temp;
	}	
	for (loop = 0;loop <= 2; loop++)
	{
		temp  =(float)(phase[loop].frequency);
		temp *=(float)(50);
		temp /=(float)(280);
		parameter_value[cnt++] = temp;
	}
	for (loop = 0;loop <= 2; loop++)
	{
		parameter_value[cnt++] = phase[loop].active;
	}
	for (loop = 0;loop <= 2; loop++)
	{
		parameter_value[cnt++] = phase[loop].reactive;
	}
	parameter_value[cnt++] = active_accumulator;
	parameter_value[cnt++] = reactive_accumulator;
	
	*/
	
//	cal_val_buff[0] = ((parameter_value[0] + parameter_value[1])/ (float)2) * 1.732;
//	cal_val_buff[1] = ((parameter_value[1] + parameter_value[2])/ (float)2) * 1.732;
//	cal_val_buff[2] = ((parameter_value[0] + parameter_value[2])/ (float)2) * 1.732;
//	cal_val_buff[3] =  (cal_val_buff[0] + cal_val_buff[1] + cal_val_buff[2]) / (float)3;
//	cal_val_buff[4] = parameter_value[3];
//	cal_val_buff[5] = parameter_value[4];
//	cal_val_buff[6] = parameter_value[5];
//	cal_val_buff[7] =  (cal_val_buff[3] + cal_val_buff[4] + cal_val_buff[5]) / (float)3;
	
	for (loop = 0;loop <= 2; loop++)
	{
		parameter_value[cnt++] = phase[loop].active;
		parameter_value[cnt++] = phase[loop].reactive;
		
		temp  =(float)(phase[loop].voltage);
		temp /=(float)(764);//730 760
		temp *=(float)(230);
		parameter_value[cnt++] = temp;
		
		temp  =(float)(phase[loop].frequency);
		temp *=(float)(50);
		temp /=(float)(280);
		parameter_value[cnt++] = temp;
		
	}
	parameter_value[cnt++] = energy_A;
	parameter_value[cnt++] = energy_R;
		
}
void calc_data_values(void)
{
	long double active,reactive,app;
	float value;
	
	
	
	if((react_buf[0] > (float)17321.02) || (react_buf[1] > (float)17321.02) || (react_buf[2] > (float)17321.02)){
		return;
	}
	if((act_buf[0] > (float)17321.02) || (act_buf[1] > (float)17321.02) || (act_buf[2] > (float)17321.02)){
		return;//30A
	}
	
	// voltages
	value = ((parameter_value[2] + parameter_value[6])/ (float)2) * 1.732;  // ry
	value += ERROR_RY_V;
	if(value < 1000.0)
		cal_val_buff[0] = value;
	else 
		return;
	
	value = ((parameter_value[6] + parameter_value[10])/ (float)2) * 1.732; //yb
	value += ERROR_YB_V;
	if(value < 1000.0)
		cal_val_buff[1] = value;
	else 
		return;
	
	value = ((parameter_value[2] + parameter_value[10])/ (float)2) * 1.732;  //br
	value += ERROR_BR_V;
	if(value < 1000.0)
		cal_val_buff[2] = value;
	else 
		return;
	
	cal_val_buff[3] =  (cal_val_buff[0] + cal_val_buff[1] + cal_val_buff[2]) / (float)3;
	
	// currents
	cal_val_buff[4] = 0;
	cal_val_buff[5] = 0;
	cal_val_buff[6] = 0;
	//cal_val_buff[7] =  (cal_val_buff[4] + cal_val_buff[5] + cal_val_buff[6]) / (float)3;
	
	// powers kw
	value = (act_buf[0]/1000)  * 1.732;
	
	if(value < 100.0)
		cal_val_buff[8] = value;
	else 
		return;
	
	if(cal_val_buff[8] < (float)0.0){
		cal_val_buff[8] *= -1;
	}
	if(cal_val_buff[8] < 0.3){
		cal_val_buff[8] = 0;
	}
	value = ((act_buf[1]/1000)  * 1.732);
	
	if(value < 100.0)
		cal_val_buff[9] = value;
	else 
		return;
	
	if(cal_val_buff[9] < (float)0.0){
		cal_val_buff[9] *= -1;
	}
	if(cal_val_buff[9] < 0.3){
		cal_val_buff[9] = 0;
	}
	value = ((act_buf[2]/1000) * 1.732);
	
	if(value < 100.0)
		cal_val_buff[10] = value;
	else 
		return;
	
	if(cal_val_buff[10] < (float)0.0){
		cal_val_buff[10] *= -1;
	}
	if(cal_val_buff[10] < 0.3){
		cal_val_buff[10] = 0;
	}
	cal_val_buff[11] =  (cal_val_buff[8] + cal_val_buff[9] + cal_val_buff[10]);// total kw / (float)3;
	
	
	
	
	
	// powers kw
	cal_val_buff[19] = (react_buf[0]/1000)* 1.732;
	cal_val_buff[20] = (react_buf[1]/1000)* 1.732;
	cal_val_buff[21] = (react_buf[2]/1000)* 1.732;
	cal_val_buff[22] =  (cal_val_buff[19] + cal_val_buff[20] + cal_val_buff[21]) / (float)3;
	
	// power factor
	
	active = act_buf[0];
	active *= active;
	
	reactive = react_buf[0];
	reactive *= reactive;
	
	app = active + reactive;
	app = sqrt(app);
	
	cal_val_buff[23] = app/1000;  //for app power
	cal_val_buff[12] = act_buf[0]/app;
	
	active = act_buf[1];
	active *= active;
	
	reactive = react_buf[1];
	reactive *= reactive;
	
	app = active + reactive;
	app = sqrt(app);
	
	cal_val_buff[24] = app/1000;  //for app power
	cal_val_buff[13] = act_buf[1]/app;
	
	active = act_buf[2];
	active *= active;
	
	reactive = react_buf[2];
	reactive *= reactive;
	
	app = active + reactive;
	app = sqrt(app);
	cal_val_buff[25] = app/1000;  //for app power
	cal_val_buff[14] = act_buf[2]/app;

//	for(i = 0; i >=2 ; i++)
//	{
//		active = act_buf[i];
//		active *= active;
//		
//		reactive = react_buf[i];
//		reactive *= reactive;
//		
//		app = active + reactive;
//		app = sqrt(app);
//		cal_val_buff[23+i] = app/1000;  //for app power
//		
//		cal_val_buff[12+i] = act_buf[i]/app;
//	}
//	
	cal_val_buff[26] = (cal_val_buff[23] + cal_val_buff[24] + cal_val_buff[25]) / 3; // for app power
	
	cal_val_buff[15] = 0;
	if(cal_val_buff[12] < (float)0.0){
		cal_val_buff[15] += cal_val_buff[12] * -1;
	}
	else{
		cal_val_buff[15] += cal_val_buff[12];
	}
	if(cal_val_buff[13] < (float)0.0){
		cal_val_buff[15] += cal_val_buff[13] * -1;
	}
	else{
		cal_val_buff[15] += cal_val_buff[13];
	}
	if(cal_val_buff[14] < (float)0.0){
		cal_val_buff[15] += cal_val_buff[14] * -1;
	}
	else{
		cal_val_buff[15] += cal_val_buff[14];
	}
	
	cal_val_buff[15] = cal_val_buff[15] /3;
	
	//cal_val_buff[15] =  (cal_val_buff[12] + cal_val_buff[13] + cal_val_buff[14]) / 3; //power factor
	
	// frequency
	cal_val_buff[16] = (parameter_value[3] + parameter_value[7] + parameter_value[11]) / 3;
	
	// energy
	cal_val_buff[17] = energy_A;
	cal_val_buff[18] = energy_R;	
	//......................................................
	// currents
	
	if((parameter_value[2] >= 500.0)  || (parameter_value[6] >= 500.0) || (parameter_value[10] >= 500.0))
	{ // for skipping junk values
		return;
	}
	
	
	cal_val_buff[4] = act_buf[0] / (parameter_value[2] * cal_val_buff[12]);
	
//	if(cal_val_buff[4] < 0.9)
//		cal_val_buff[4] = 0;
	cal_val_buff[5] = act_buf[1] / (parameter_value[6] * cal_val_buff[13]);
	
//	if(cal_val_buff[5] < 0.9)
//		cal_val_buff[5] = 0;
	cal_val_buff[6] = act_buf[2] / (parameter_value[10] * cal_val_buff[14]);
	
//	if(cal_val_buff[6] < 0.9)
//		cal_val_buff[6] = 0;
	//cal_val_buff[7] =  (cal_val_buff[4] + cal_val_buff[5] + cal_val_buff[6]) / (float)3;
	
	
	if(cal_val_buff[4] == 0  && cal_val_buff[5] != 0 && cal_val_buff[6] != 0){
			cal_val_buff[7] =  (cal_val_buff[4] + cal_val_buff[5] + cal_val_buff[6]);// / (float)2;
	}
	else if(cal_val_buff[4] != 0  && cal_val_buff[5] == 0 && cal_val_buff[6] != 0){
			cal_val_buff[7] =  (cal_val_buff[4] + cal_val_buff[5] + cal_val_buff[6]);// / (float)2;
	}
	else if(cal_val_buff[4] != 0  && cal_val_buff[5] != 0 && cal_val_buff[6] == 0){
			cal_val_buff[7] =  (cal_val_buff[4] + cal_val_buff[5] + cal_val_buff[6]);// / (float)2;
	}
	else if(cal_val_buff[4] != 0  && cal_val_buff[5] != 0 && cal_val_buff[6] != 0){
			cal_val_buff[7] =  (cal_val_buff[4] + cal_val_buff[5] + cal_val_buff[6]);// / (float)3;
	}
	else if(cal_val_buff[4] == 0  && cal_val_buff[5] == 0 && cal_val_buff[6] == 0){
			cal_val_buff[7] =  0;
	}
	
	
}
void meter_read_fsm(void)
{
	unsigned char i,address,start,end;
			//--------------------------active-----------------------------------
    for(i=0;i<3;i++)
    {
			HAL_Delay(10);
			address = i*4;					//address for 'active' register
			new_val= acquire(address);			//read the register

			if(phase[i].first_A)				//if this is not the 1st sample, add to accumulator.
			{				
	        delta_val= new_val - phase[i].active;		//delta_val=new_val-previous_val
					act_buf[i] = (ENERGY_CONSTANT * delta_val); 
	        active_accumulator+= consumption(delta_val);	
			}
			else phase[i].first_A = 1;			//we've completed 1st sample
				phase[i].active = new_val;			//store new_val
			
    }
		if((cal_val_buff[4] != 0  && cal_val_buff[5] != 0) || (cal_val_buff[5] != 0  && cal_val_buff[6] != 0) || (cal_val_buff[4] != 0  && cal_val_buff[6] != 0)){
			// if load available update energy
			process_a_data();
		}
		//process_a_data();					//threshold & LEDs
		//--------------------------reactive---------------------------------
		for(i=0;i<3;i++)
		{
				HAL_Delay(10);
				address = (i*4)+1;				//address for re-active register
	      new_val= acquire(address);			//read the register

	      if(phase[i].first_R)				//if this is not the 1st sample, add to accumulator.
				{
	        delta_val= new_val - phase[i].reactive;
					react_buf[i] = (ENERGY_CONSTANT * delta_val); 
	        reactive_accumulator+= consumption(delta_val);
				}
	      else phase[i].first_R = 1;			//we've completed 1st sample
					phase[i].reactive = new_val;
				
		 }
			process_r_data();					//threshold & LEDs
	 
		 //---------------------- voltage and frequency ------------------------
		 for(i=0;i<3;i++)
	   {			
	    start = i*4+2;						//address for voltage register
	    end   = start+1;						//address for frequency register
	    for(address=start;address<=end;address++)
	     {
				 HAL_Delay(10);
	      new_val= acquire(address);				//read the register(s)

	      if(address == start) phase[i].voltage = (signed int)new_val;
	       else if(address == end)
	       {
					 
					 if(Status_Flag->phase_err_chk){
						 Status_Flag->phase_err_chk = CLEAR;
					 //.................phase errors.......................
					 //phase error status bits (D21,D22) in frequency registor Ref: SA9904B datasheet page:12
					 phase_error_status = (new_val >> 21) & 0x03;
						 
					//send phanse errors to display board	 
					sprintf(val_buff,"^%d!",phase_error_status);
					HAL_UART_Transmit(&huart4,(uint8_t *)val_buff,strlen(val_buff),50);

					HAL_UART_Transmit(&huart5,(uint8_t *)val_buff,strlen(val_buff),50);			
					
//							sprintf(val_buff,"reg value- %d  -  %lx \n ",phase_error_status,new_val);
//								HAL_UART_Transmit(&huart4,(uint8_t *)val_buff,strlen(val_buff),50);
					 
					 if(phase_error_status == 0)
					 {//No phase error
						 
						 Status_Flag->no_phase_err = SET;
						 Status_Flag->phase_seq_err = CLEAR;
						 Status_Flag->missing_phase_err = CLEAR;
						 	#ifdef METER_DEBUG					 
								sprintf(val_buff,"No phase error- %lx \n ",new_val);
								HAL_UART_Transmit(&huart5,(uint8_t *)val_buff,strlen(val_buff),50);
						 #endif
					 }
					 if(phase_error_status & 0x02)
					 {	//phase sequence error
							Status_Flag->no_phase_err = CLEAR;
							Status_Flag->phase_seq_err = SET;
						 #ifdef METER_DEBUG
								sprintf(val_buff,"phase seq err- %lx \n ",new_val);
								HAL_UART_Transmit(&huart5,(uint8_t *)val_buff,strlen(val_buff),50);
						 #endif
					 }
					 if(phase_error_status & 0x01)
					 {//Missing phase					 
							 Status_Flag->no_phase_err = CLEAR;
							 Status_Flag->missing_phase_err = SET;
								#ifdef METER_DEBUG
									sprintf(val_buff,"missing phase err- %lx \n ",new_val);
									HAL_UART_Transmit(&huart5,(uint8_t *)val_buff,strlen(val_buff),50);
								#endif
								Status_Flag->phase_missing_state = ((new_val >> 18) & 0x07);// D20,D19,D18 bits for phase
								
								// send phase missing status to display board
								sprintf(val_buff,"@%d!",Status_Flag->phase_missing_state);
								HAL_UART_Transmit(&huart4,(uint8_t *)val_buff,strlen(val_buff),50);	
									HAL_UART_Transmit(&huart5,(uint8_t *)val_buff,strlen(val_buff),50);
										
									
					 }//if(phase_error_status & 0x01)
						#ifdef METER_DEBUG	
							sprintf(val_buff,"phase status - %lx \n ",new_val);
							HAL_UART_Transmit(&huart5,(uint8_t *)val_buff,strlen(val_buff),50);
					 #endif
					  //....................................................
				 }
	        new_val &= 0x0003ff; 					//only interested in last 10 bits for frequency
	        phase[i].frequency = (unsigned int)new_val;
	       }
	     }
	   }
	 
		data_assign();
		 
		calc_data_values();	
		 
//		if(cal_val_buff[4] != 0  && cal_val_buff[5] != 0 && cal_val_buff[6] != 0){
//			// if load available update energy
//			process_a_data();
//		}
}

