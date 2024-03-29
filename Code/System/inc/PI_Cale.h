//############################################################
// Created on: 2019年5月18日
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//版权所有，盗版必究
//STM32电机控制开发板
//匠心科技
//网址: https://shop298362997.taobao.com/
//电机控制答疑邮箱：JXKJ_2007@163.com
//############################################################
#ifndef  PI_Cale_H
#define  PI_Cale_H

#include "SysConfig.h"

//速度PID
#define  INIT_PURPOSE       0
#define  RUN_PURPOSE        1
#define  SPEEDLOOPCNT       50   //速度占空比调整周期  3->50

#define    _IQmpy(A,B)         ((A) * (B))
#define    FirstOrder_LPF_Cacl(Xn, Yn_1, a)	\
																	Yn_1 = (1-a)*Yn_1 + a*Xn; //Xn:in;Yn:out;a:系数 //软件滤波, a=t/RC
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

//定义PID相关宏
// 这三个参数设定对电机运行影响非常大
/*************************************/
#define  P_DATA_ACC                   0.4 //0.7              //加速P参数  0.35
#define  I_DATA_ACC                   0.05//0.04        //I参数 0.03
#define  D_DATA_ACC                   0                 //D参数、

#define  P_DATA_DEC                   0.8               //减速P参数  0.35
#define  I_DATA_DEC                   0.01//0.06        //I参数 0.03
#define  D_DATA_DEC                   0                 //D参数

/*********************************************************************************************************
  电机PID控制结构体
*********************************************************************************************************/
typedef struct  {
	float			Kp;
	float			Ki;
	float			Kd;
	uint16_t		MaxValue;		//最大
	uint16_t		MinValue;
	float			fpAllErr;		
	float			Error;			//误差
	float			Out;			//PID结果
	uint8_t			Purpose;
	float			Ref_Value;		//0
	float			Ui;				//积分值
	uint16_t		Max;
	uint16_t		Min;
}PIDCONTROL;

//定义PID结构体
typedef struct 
{
   __IO int      SetPoint;      //设定目标 Desired Value
   __IO double   Proportion;    //比例常数 Proportional Const
   __IO double   Integral;      //积分常数 Integral Const
   __IO double   Derivative;    //微分常数 Derivative Const
   __IO int      LastError;     //Error[-1]
   __IO int      PrevError;     //Error[-2]
}PID;

extern   PIDCONTROL     PID_Speed ;
extern   float  Speed;
extern  uint8_t  Enable_Times;
void     PID_init(void);
extern  PID bldc_pid;
extern   void ErrorCorrection(void);
extern   void CalcSpeedTime(void);
extern  void bldcCalcSpeed(void);
extern  void SpeedController(void);
extern  void bldcSpeedControlTime(int32_t Idle_SpeedValue,int32_t Actual_SpeedValue);
#endif
