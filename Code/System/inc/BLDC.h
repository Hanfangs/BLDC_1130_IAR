#ifndef  _BLDC_H
#define  _BLDC_H

#ifndef  _BLDC_FILE
#define  GLOBAL_BLDC_  extern
#else
#define  GLOBAL_BLDC_ 
#endif

#include "SysConfig.h"


#define NUM_AVR              3
#define BLDCSTOP             1                   /* 电机停止     */
#define BLDCSTART            2                   /* 电机启动     */
#define BLDCRUN              3                   /* 电机运行     */
#define BLDCSTARTERR         4                   /*电机启动错误  */
#define BLDCTIMEERR          5
#define BLDCRUNTIMEERR       6                   /*电机运行错误  */

#define  Flag_DEFAULTS    {0,0,0}

// #define  PWM_DUTYCYCLE_05    75
// #define  PWM_DUTYCYCLE_10    150
// #define  PWM_DUTYCYCLE_15    225
// #define  PWM_DUTYCYCLE_20    300
// #define  PWM_DUTYCYCLE_25    375

// #define  PWM_DUTYCYCLE_30    450
// #define  PWM_DUTYCYCLE_50    750
// #define  PWM_DUTYCYCLE_60    900
// #define  PWM_DUTYCYCLE_80    1200
// #define  PWM_DUTYCYCLE_90    1350
// #define  PWM_DUTYCYCLE_100   1500


// #define  PWM_DUTYCYCLE_20    600
// #define  PWM_DUTYCYCLE_100   3000

#define  PWM_DUTYCYCLE_05    150
#define  PWM_DUTYCYCLE_10    300
#define  PWM_DUTYCYCLE_15    450
#define  PWM_DUTYCYCLE_20    600
#define  PWM_DUTYCYCLE_25    750

#define  PWM_DUTYCYCLE_30    900
#define  PWM_DUTYCYCLE_50    1500
#define  PWM_DUTYCYCLE_60    1800
#define  PWM_DUTYCYCLE_80    2400
#define  PWM_DUTYCYCLE_90    2700
#define  PWM_DUTYCYCLE_100   3000

#define  PWM_MIN_DUTY			PWM_DUTYCYCLE_20	// 最小占空比
#define  PWM_MAX_DUTY			PWM_DUTYCYCLE_50	// 最大占空比

// #define    MINIMUMSPEED        5000    //最小转速
// #define    MAXIMUMSPEED				22000	 // 电机最大转   DELTADUTYCYCLE		  30


//电位器和给定速度的系数  
#define   Coff_1        3.6
#define   TEST_MANUELL  0
#define   RHEOSTATMIN   200 
#define   RHEOSTATMAX   4000
#define  OFF_RHEOSTAT   200
#define  ON_RHEOSTAT    400


#define  Motor_DelayTime   6000


#define	EMPTY_LOAD 0
#if (EMPTY_LOAD == 1)
//空载
#define	P1	0.0005349
#define	P2	-0.1232
#define	P3	13.48
#define	P4	7.269
//带载
#else
	#define	P1	0.001656
	#define	P2	-0.2403
	#define	P3	16.29 
	#define	P4	24.74
#endif
#define    ACC_FUN(X, P1, P2, P3, P4)	((P1)*(X)*(X)*(X) + (P2)*(X)*(X) + (P3)*(X) + (P4))

typedef enum
{	
	Startup = 0,
	Operation = 1,
	MotorOFF = 2,
}MasterState_T;

GLOBAL_BLDC_ MasterState_T MasterState;	



typedef enum
{
	mcInit = 0,
	mcAlignment = 1,
	mcDrag = 2, 
	mcRun = 3,
	mcDec = 4,
	mcStop = 5,	// 电机停止工作
	mcStopNext = 6,
	mcStopNext2 = 7,
}MotorState_T;	//电机状态标志
GLOBAL_BLDC_  MotorState_T mcState;

typedef enum
{
	RunNormal=0,
	OverUnderVoltage= 1,
	OverCurrent = 2,
	OverTemperature = 3, 
	Motor_Stall_1 = 4,
	HardWare_OverCurrent=5,
	Overtime_PhaseChange=6,
}Fault;

GLOBAL_BLDC_ Fault   mcFault;
typedef struct {

	uint8_t    ControlMode;                            /* 电机控制方式    */
	uint8_t    State;                                 /* 电机当前状态   */
	uint16_t   TheorySpeed;                           /* 电机理论速度   */
	uint16_t   ActualSpeed;                           /* 电机实际速度   */
	uint16_t   ActAvgSpeed;                           /* 电机平均速度   */
	uint16_t   Last_Speed;
	uint16_t   OpenSpeed;
	uint16_t   Duty;                                  /* 电机PWM占空比  */
	uint16_t   Order;
	float      Current;
	uint16_t   StepNum;
	uint8_t    PhaseCnt;															/* 当前相位  */
	uint8_t    LastPhase;
	uint8_t    NextPhase;
	uint16_t   Bemf;
	uint16_t   UVoltage;
	uint16_t   VVoltage;
	uint16_t   WVoltage;
	uint16_t   CheckCurrent;
	int16_t	   Uab;
	int16_t	   Ubc;
	int16_t	   Uca;
	int16_t    Ea;
	int16_t    Eb;
	int16_t    Ec;
	uint16_t   step_counter;
	uint16_t   step_counter_prev;
	uint16_t   motor_speed;                           /* 电机设置速度   */
	uint32_t   motorRunTime;	
}BLDC;
GLOBAL_BLDC_  BLDC  Motor;

typedef struct 
{
    uint16_t  DragTime; 
    uint16_t  SamePhaseCnt;  //换向错误计数
    uint16_t  ADCTimeCnt;
    uint16_t  Timer3OutCnt;
    uint16_t  BlankingCnt;
    uint16_t  Stop_Time;
	  uint16_t  SpeedTimeCnt ;	
    uint16_t  SpeedTime ;
    uint16_t  SpeedTimeTemp ;
    uint32_t  SpeedTimeSum;
    uint16_t  DelayTime30;
	uint16_t  CorrectionTime;
	uint32_t  ChangeCount;
	uint32_t  ChangeCount_Dec;
	uint32_t  ChangeCount_Stop;
	uint16_t  ChangeTime_Count;
	uint16_t  LastDragTime;
	uint16_t  StopDelayTime;
}variable;
GLOBAL_BLDC_  variable  Sysvariable;
GLOBAL_BLDC_  uint8_t    pos_check_stage;
GLOBAL_BLDC_  uint8_t    pos_idx;
GLOBAL_BLDC_  uint8_t    charger_idx;
GLOBAL_BLDC_  uint16_t  ADC_check_buf[6];
GLOBAL_BLDC_  uint16_t  ADC_Sum_buf[6];
GLOBAL_BLDC_  uint8_t  Flag_adc;
GLOBAL_BLDC_  uint8_t  Charger_Time;
GLOBAL_BLDC_  uint8_t  Flag_Charger;
GLOBAL_BLDC_  uint8_t  Flag_OverCurr;
GLOBAL_BLDC_  uint16_t    usTmsAvrgDly;
GLOBAL_BLDC_  uint16_t   UserSpeedSample;
GLOBAL_BLDC_  uint16_t  test_idx;
GLOBAL_BLDC_  uint8_t  Flag_Alignment;
GLOBAL_BLDC_ void  MotorInit(void);
GLOBAL_BLDC_ void  Startup_Turn(void);
GLOBAL_BLDC_ void  Motor_PWM_IDLE(void);
GLOBAL_BLDC_ void  MotorStop(void);
GLOBAL_BLDC_ void  MotorStopNext(void);
GLOBAL_BLDC_ void  MotorStopNext2(void);
GLOBAL_BLDC_ void  Set_Motor_Stop_Delay(uint32_t delay_time);
GLOBAL_BLDC_ void  Motor_Stop_Delay(void);
GLOBAL_BLDC_ void  MotorAlignment(void);
GLOBAL_BLDC_ void  EnterDragInit(void);
GLOBAL_BLDC_ void  EnterRunInit(void);
GLOBAL_BLDC_ void  StartupDrag(void);
GLOBAL_BLDC_ void  MotorControl(void);
GLOBAL_BLDC_ void  UserSpeedControlInit(void);
GLOBAL_BLDC_ void  AccSpeedControl(void);
GLOBAL_BLDC_ void  DecSpeedControl(void);
GLOBAL_BLDC_ void  Sys_Variable_Init(void);
GLOBAL_BLDC_ void  Align_pos_check_proc(void);
GLOBAL_BLDC_ void All_Discharg(void);
GLOBAL_BLDC_ void Change_Voltage(void);

#endif




