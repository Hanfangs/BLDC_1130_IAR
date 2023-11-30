
#ifndef  _SysConfig_H
#define  _SysConfig_H

#ifndef  _SYSTEM_FILE
#define  GLOBAL_SYS_  extern
#else
#define  GLOBAL_SYS_ 
#endif

#include "stm32f10x.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#define  State_DEFAULTS  {0,0,0,0,0,0,0,1,0,0,0,0,0,0,0} // ��ʼ������
#define  Debug  1
enum{
	  Normal=0,
	  InNormal
};

typedef struct {
	    uint8_t    Control_Mode;   // ����ģʽ  
	    uint8_t    TripFlagDMC;    // ���� ������־
	    uint8_t    drive_car;      // ��ʼ�������״̬
	    uint8_t    olddrive_car;   // ��ʷ��ʼ�������״̬
	    uint8_t    clear_PWMtripz; // ���������־
	    uint8_t    Run_mode;       // ����ת����״̬
	    uint16_t   Qiehuan_count;  // �л�״̬�ļ���ֵ
	    uint8_t    Start_order;    // ����PWM���������
	    uint16_t   Duty;           // �л�״̬�ļ���ֵ
	    uint16_t   Speed_Count;    // �ٶȻ���ʱ
	    uint16_t   Current_Count;  // ��������ʱ	
			uint16_t   Aim_Speed;
			uint32_t   Aim_Duty;
      uint32_t	 INVERSION;
	    uint32_t	 Temp;
 }State; 

GLOBAL_SYS_    uint16_t nTime ;
extern  uint8_t  Sys_Status;

//���Ա���
extern  uint32_t  TTEMP1;
void SYSTICK_Init(void);
GLOBAL_SYS_  void Delay_ms( uint16_t time_ms );
GLOBAL_SYS_  void Delay(__IO uint32_t nTime);
GLOBAL_SYS_  void Delay_us( uint16_t time_us );  
void Led_RunTask_Op(u8 Led_Status);

#define LED_LIGHTUP     0  //����
#define LED_FLASH       1  //����
#define LED_LIGHTDOWM   2  //���
 
#endif
 
