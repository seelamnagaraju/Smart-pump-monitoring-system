
#include "struct_var.h"
#include "stm32f1xx_hal.h"
#include "const.h"

#define LOW   0
#define HIGH  1
#define ON 1
#define OFF 0
#define THRES	50000 	//MTB=1.068e3   pulses/kwh, 3phase 'sum' mode
#define ENERGY_CONSTANT  (float)0.575   // (230*80*delta_val)/320000 * 100ms
	                                      //calculated for every 100ms time interval 


unsigned char phase_error_status;

char val_buff[300];

volatile long	new_val,	//present register value 		
      	   		delta_val;	//difference between consecutive readings
	


struct phase_data phase[3]; //0,1,2

volatile signed long	active_accumulator,	//accumulates active energy
										reactive_accumulator;	//  "      re-active energy

long  	    threshold_A,		//active    energy threshold 
						threshold_R;		//re-active energy threshold
float	    energy_A, energy_R;		//active/re-active energy

volatile long test;			//temp storage for labs() use

float parameter_value[METER_TOT_REG];
float cal_val_buff[METER_TOT_PARAMS]; // 20
float act_buf[3];
float react_buf[3];
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern char disp_tx_buff[DB_TX_BUFF_SIZE];



void sams_meter_init(void);
long acquire(unsigned int);
long consumption(long);
void process_a_data(void);
void process_r_data(void);
long labs(long );
void data_assign(void);
void calc_data_values(void);
void meter_read_fsm(void);
