#ifndef __SAMS_SA9904B_H
#define __SAMS_SA9904B_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include "const.h"

#define LOW   0
#define HIGH  1
#define ON 1
#define OFF 0
#define THRES	50000 	//MTB=1.068e3   pulses/kwh, 3phase 'sum' mode

#define ERROR_RY_V  6
#define ERROR_YB_V  7
#define ERROR_BR_V   2
//#define ENERGY_CONSTANT  (float)0.6325   // (230*95*delta_val)/320000 * 100ms
	                                      //calculated for every 100ms time interval 

//#define ENERGY_CONSTANT  (float)0.6325   // (230*88*delta_val)/320000 * 100ms
	                                      //calculated for every 100ms time interval 

//#define ENERGY_CONSTANT  (float)0.575   // (230*80*delta_val)/320000 * 100ms
	                                      //calculated for every 100ms time interval 


//#define ENERGY_CONSTANT  (float)0.43125   // (230*60*delta_val)/320000 * 100ms
	                                      //calculated for every 100ms time interval 

//#define ENERGY_CONSTANT  (float)0.2875   // (230*40*delta_val)/320000 * 100ms
	                                      //calculated for every 100ms time interval 

//#define ENERGY_CONSTANT  (float)0.474375   // (230*66*delta_val)/320000 * 100ms
	                                      //calculated for every 100ms time interval 

//#define ENERGY_CONSTANT  (float)0.5390625   // (230*75*delta_val)/320000 * 100ms
	                                      //calculated for every 100ms time interval 

#define ENERGY_CONSTANT  (float)0.512566   // (230*71.3135*delta_val)/320000 * 100ms
	                                      //calculated for every 100ms time interval

extern char   val_buff[300];
extern float  parameter_value[METER_TOT_REG];
extern float  cal_val_buff[METER_TOT_PARAMS];
extern float	energy_A;

void sams_meter_init(void);
long acquire(unsigned int);
long consumption(long);
void process_a_data(void);
void process_r_data(void);
long labs(long );
void data_assign(void);
void calc_data_values(void);
void meter_read_fsm(void);

//--------------------------------------------------------------------------------

#ifdef __cplusplus
}
#endif

/**
  * @}
*/ 

#endif /* __SAMS_SA9904B_H */
