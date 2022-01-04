#ifndef __STRUCT_VAR_H
#define __STRUCT_VAR_H

#ifdef __cplusplus
 extern "C" {
#endif
	 

//......................................................
struct STATUS_FLAGS
{
	unsigned int cpu_helth_led : 1;
	unsigned int gsm_sms_on : 1;
	unsigned int gsm_sms_rdy : 1;
	unsigned int gsm_cmd_seq : 1;
	unsigned int gsm_engine_reset : 1;
	unsigned int gsm_msg_rx_flag : 1;
	unsigned int disp_msg_rx_flag : 1;
	unsigned int gprs_connect_flag : 1;
	unsigned int gprs_disconnect_flag : 1;
	unsigned int gprs_command_flag : 1;
	unsigned int gprs_config_flag : 1;
	unsigned int gprs_update_flag : 1;
	unsigned int gps_response_flag : 1;
	unsigned int gps_read_flag : 1;
	unsigned int gsm_ss_read : 1;
	unsigned int gsm_ss_low : 1;
	unsigned int gsm_del_all_sms : 1;
	
	unsigned int run_time_start : 1;
	unsigned int run_time_elapse : 1;
	
	unsigned int on_delay_start : 1;
	unsigned int on_delay_elapse : 1;
	
	unsigned int motor_auto_mode_on_flag : 1;
	
	unsigned int dry_run_start : 1;
	
	unsigned int motor_on_flag : 1;
	unsigned int motor_off_flag : 1;
	
	unsigned int sw_status_flag : 1;
	unsigned int sw_cmd_flag : 1;
	unsigned int update_flag : 1;
	unsigned int rtc_read : 1;
	unsigned int sch1_timer : 1;
	unsigned int sch2_timer : 1;
	unsigned int sch3_timer : 1;
	unsigned int sch4_timer : 1;
	unsigned int sch1_timer_on : 1;
	unsigned int sch2_timer_on : 1;
	unsigned int sch3_timer_on : 1;
	unsigned int sch4_timer_on : 1;
	unsigned int get_status : 1;
	unsigned int get_settings : 1;
	unsigned int get_gps : 1;
	unsigned int get_users : 1;
	unsigned int under_voltage : 1;
	unsigned int under_voltage_off : 1; 
	unsigned int over_voltage : 1;
	unsigned int over_voltage_off : 1;
	unsigned int under_load : 1;
	unsigned int under_load_off : 1;
	unsigned int under_load_alert : 1;
	unsigned int over_load : 1;
	unsigned int over_load_off : 1;
	unsigned int over_voltage_alert : 1;
	unsigned int under_voltage_alert : 1;
	unsigned int over_load_alert : 1;
	
	unsigned int read_meter : 1;
	//phase errors from meter ic
	unsigned int phase_err_chk : 1;
	unsigned int no_phase_err : 1;
	unsigned int phase_seq_err : 1;
	unsigned int missing_phase_err : 1;
	
	unsigned int pre_no_phase_err : 1;
	unsigned int pre_phase_seq_err : 1;
	unsigned int pre_missing_phase_err : 1;
	
//	unsigned int phase_1_missing: 1;
//	unsigned int phase_2_missing: 1;
//	unsigned int phase_3_missing: 1;
//	
//	unsigned int pre_phase_1_missing: 1;
//	unsigned int pre_phase_2_missing: 1;
//	unsigned int pre_phase_3_missing: 1;	
	
	unsigned int phase_missing_state : 3;
	unsigned int pre_phase_missing_state : 3;
	
	unsigned int first_on : 1;
	unsigned int no_of_ons_exceded : 1;
	unsigned int on_reenable : 1;
};


//..............for sams_sa9904b data...................
struct phase_data
{
	signed long	active;				//active energy in 'counts'
	signed long	reactive;			//re-active energy
	unsigned char first_A;			//1st active    sample flag
	unsigned char first_R;			//1st re-active sample flag
	signed int voltage;			    //supply voltage
	unsigned int frequency;			//supply frequency
}; 
//.......................................................
struct rtc{
	unsigned short year;
	unsigned char month;
	unsigned char date;
	unsigned char hour;
	unsigned char minute;
	unsigned char second;
	
};

#ifdef __cplusplus
}
#endif
/**
  * @}
*/ 

#endif /* __STRUCT_VAR_H */


