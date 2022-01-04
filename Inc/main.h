/**
  ******************************************************************************
  * File Name          : main.hpp
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  **
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
 extern "C" {
#endif
	  
/* Includes ------------------------------------------------------------------*/


/* Private define ------------------------------------------------------------*/
#define relay_ctrl_Pin GPIO_PIN_0
#define relay_ctrl_GPIO_Port GPIOC
#define gsm_ss_Pin GPIO_PIN_1
#define gsm_ss_GPIO_Port GPIOC
#define CPU_LED_Pin GPIO_PIN_0
#define CPU_LED_GPIO_Port GPIOB
#define gps_pwr_en_Pin GPIO_PIN_1
#define gps_pwr_en_GPIO_Port GPIOB
#define gps_tx_Pin GPIO_PIN_10
#define gps_tx_GPIO_Port GPIOB
#define gps_rx_Pin GPIO_PIN_11
#define gps_rx_GPIO_Port GPIOB
#define SPI2_CS_Pin GPIO_PIN_12
#define SPI2_CS_GPIO_Port GPIOB
#define gsm_pwr_en_Pin GPIO_PIN_6
#define gsm_pwr_en_GPIO_Port GPIOC
#define gsm_pwr_Pin GPIO_PIN_7
#define gsm_pwr_GPIO_Port GPIOC
#define gsm_tx_Pin GPIO_PIN_9
#define gsm_tx_GPIO_Port GPIOA
#define gsm_rx_Pin GPIO_PIN_10
#define gsm_rx_GPIO_Port GPIOA
#define SPI1_CS_Pin GPIO_PIN_15
#define SPI1_CS_GPIO_Port GPIOA
#define display_tx_Pin GPIO_PIN_10
#define display_tx_GPIO_Port GPIOC
#define display_rx_Pin GPIO_PIN_11
#define display_rx_GPIO_Port GPIOC
#define config_tx_Pin GPIO_PIN_12
#define config_tx_GPIO_Port GPIOC
#define config_rx_Pin GPIO_PIN_2
#define config_rx_GPIO_Port GPIOD

//------------------------------------------------------------------------
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
	 

// 5mS Tick Time distribution FSM States
enum tag5mSTickStates
{
    ST_5mS_TICK1,
    ST_5mS_TICK2,
    ST_5mS_TICK3,
    ST_5mS_TICK4,
    ST_5mS_TICK5
};
extern enum tag5mSTickStates e5mSTickState;

struct SCHED_FLAG
{
	unsigned int fsm_5ms_tick : 1; 
	unsigned int sched1_25ms_tick : 1;
	unsigned int sched2_25ms_tick : 1;
	unsigned int sched3_25ms_tick : 1;
	unsigned int sched4_25ms_tick : 1;
	unsigned int sched5_25ms_tick : 1;
	unsigned int sched1_100ms_tick : 1;
	unsigned int sched2_100ms_tick : 1;
	unsigned int sched3_100ms_tick : 1;
	unsigned int sched4_100ms_tick : 1;
	unsigned int sched5_100ms_tick : 1;
};
extern struct SCHED_FLAG Sched_Flags, *Sched_Flag;
	 
#ifdef __cplusplus
}
#endif
/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
