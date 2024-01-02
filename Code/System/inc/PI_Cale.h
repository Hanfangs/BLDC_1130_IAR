//############################################################
// Created on: 2019��5��18��
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//��Ȩ���У�����ؾ�
//STM32������ƿ�����
//���ĿƼ�
//��ַ: https://shop298362997.taobao.com/
//������ƴ������䣺JXKJ_2007@163.com
//############################################################
#ifndef  PI_Cale_H
#define  PI_Cale_H

#include "SysConfig.h"

//�ٶ�PID
#define  INIT_PURPOSE       0
#define  RUN_PURPOSE        1
#define  SPEEDLOOPCNT       3   //�ٶ�ռ�ձȵ�������

#define    _IQmpy(A,B)         ((A) * (B))
#define    FirstOrder_LPF_Cacl(Xn, Yn_1, a)	\
																	Yn_1 = (1-a)*Yn_1 + a*Xn; //Xn:in;Yn:out;a:ϵ�� //����˲�, a=t/RC
#define    UP16LIMIT(var,max,min) {(var) = (var)>(max)?(max):(var) ;\
																					(var) = (var)<(min)?(min):(var) ;\
																					}
#define PID_CALC(v)	\
											v.Err = v.Ref - v.Fdb; \
											v.Up= _IQmpy(v.Kp,v.Err);\
											v.Ui= v.Ui + _IQmpy(v.Ki,v.Up);\
											UP16LIMIT(v.Ui,v.OutMax,v.OutMin);\
											v.Ud = v.Kd * (v.Up - v.Up1);\
											v.Out = v.Up + v.Ui + v.Ud;\
											UP16LIMIT(v.Out,v.OutMax,v.OutMin);\
											v.Up1 = v.Up;	

/*********************************************************************************************************
  ���PID���ƽṹ��
*********************************************************************************************************/
typedef struct  {
	float			Kp;
	float			Ki;
	float			Kd;
	uint16_t		MaxValue;		//���
	uint16_t		MinValue;
	float			fpAllErr;		
	float			Error;			//���
	float			Out;			//PID���
	uint8_t			Purpose;
	float			Ref_Value;		//0
	float			Ui;				//����ֵ
	uint16_t		Max;
	uint16_t		Min;
}PIDCONTROL;

extern   PIDCONTROL     PID_Speed ;
extern   float  Speed;
extern  uint8_t  Enable_Times;
void     PID_init(void);
extern   void CalcSpeedTime(void);
extern  void bldcCalcSpeed(void);
extern  void SpeedController(void);
extern  void bldcSpeedControlTime(int32_t Idle_SpeedValue,int32_t Actual_SpeedValue);
#endif
