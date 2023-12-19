#ifndef _Timer_H
#define _Timer_H

#ifndef   _TIM_FILE
#define   GLOBAL_TIM_   extern
#else
#define   GLOBAL_TIM_  
#endif
#include "SysConfig.h"

//GLOBAL_TIM_  int16_t StateContr.Duty;



#define   U_Mos_H_Pin       GPIO_Pin_8
#define   V_Mos_H_Pin       GPIO_Pin_9
#define   W_Mos_H_Pin       GPIO_Pin_10

#define   U_Mos_L_Pin       GPIO_Pin_13
#define   V_Mos_L_Pin       GPIO_Pin_14
#define   W_Mos_L_Pin       GPIO_Pin_15


#define BLDC_TIMx                       TIM1
#define BLDC_TIM_APBxClock_FUN          RCC_APB2PeriphClockCmd
#define BLDC_TIM_CLK                    RCC_APB2Periph_TIM1

#define BLDC_TIM_GPIO_CLK               (RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB)

#define BLDC_TIM_CH1_PIN                GPIO_Pin_8
#define BLDC_TIM_CH2_PIN                GPIO_Pin_9
#define BLDC_TIM_CH3_PIN                GPIO_Pin_10

#define BLDC_TIM_CH1N_PIN               GPIO_Pin_13
#define BLDC_TIM_CH2N_PIN               GPIO_Pin_14
#define BLDC_TIM_CH3N_PIN               GPIO_Pin_15

#define BLDC_TIM_BKIN_PORT              GPIOB
#define BLDC_TIM_BKIN_PIN               GPIO_Pin_12  ///这个脚好象没有用


void TIM1_Config(void);
void TIM2_Config(void);
void TIM4_Config(void);  
void TIM1_BRK_UP_TRG_COM_IRQHandler(void);
GLOBAL_TIM_ void Stop_Motor(void);
GLOBAL_TIM_ void Start_Motor(void);
GLOBAL_TIM_ void MOS_Q15PWM(void); 
GLOBAL_TIM_ void MOS_Q16PWM(void);
GLOBAL_TIM_ void MOS_Q26PWM(void);
GLOBAL_TIM_ void MOS_Q24PWM(void); 
GLOBAL_TIM_ void MOS_Q34PWM(void);
GLOBAL_TIM_ void MOS_Q35PWM(void);
#endif
