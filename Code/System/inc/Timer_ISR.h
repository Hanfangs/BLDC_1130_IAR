#ifndef _Timer_ISR_H
#define _Timer_ISR_H

#ifndef   _Timer_ISR_FILE
#define   GLOBAL_Timer_ISR_   extern
#else
#define   GLOBAL_Timer_ISR_  
#endif
#include "SysConfig.h"
GLOBAL_Timer_ISR_  uint16_t ADCIntProtectCnt;//adc�жϼ�ʱ����
GLOBAL_Timer_ISR_  uint16_t TuneDutyRatioCnt ;// ����ת��ռ�ձȵ����ڼ���
GLOBAL_Timer_ISR_ void TIM1_BRK_UP_TRG_COM_IRQHandler(void);
GLOBAL_Timer_ISR_ void  TIM3_IRQHandler(void);
#endif


