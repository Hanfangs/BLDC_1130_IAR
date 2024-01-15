
#ifndef   _Analog_H
#define   _Analog_H

#ifndef   _ADC_FILE
#define   GLOBAL_ADC_   extern
#else
#define   GLOBAL_ADC_  
#endif
#include "SysConfig.h"
#define  ADCSamp_DEFAULTS  {0,0,0,0,0,0,0,0,1000,24,0}   // 初始化参数
#define  mcCurOffset_DEFAULTS  {0,0,0,0,0,0,0,0,0}

#define  U_Collect_Pin        GPIO_Pin_0       //U相电压采集引脚
#define  V_Collect_Pin        GPIO_Pin_1       //V相电压采集引脚
#define  Bat_Collect_Pin        GPIO_Pin_4 
#define  W_Collect_Pin        GPIO_Pin_5       //W相电压采集引脚
//#define  Inst_Curr_Pin          GPIO_Pin_2    
//#define  PCB_Temp_Pin           GPIO_Pin_3    

#define  Current_Collect_Pin    GPIO_Pin_1  



//#define  Inst_Curr_Channel   ADC_Channel_2


#define  U_Vol_Channel     ADC_Channel_0
#define  V_Vol_Channel     ADC_Channel_1
#define  BatVol_Channel    ADC_Channel_4
#define  W_Vol_Channel     ADC_Channel_5
#define  Current_Channel   ADC_Channel_9
#define  PCB_Temp_Channel  ADC_Channel_16

#define ADC_CHSELR_CONFIG_U  0x0000004F   			
#define ADC_CHSELR_CONFIG_V  0x0000002F			
#define ADC_CHSELR_CONFIG_W  0x0000001F	


GLOBAL_ADC_   uint8_t    Filter_Count;
GLOBAL_ADC_   float      Aver_Current;
GLOBAL_ADC_   float      DC_TotalCurrent;
GLOBAL_ADC_   uint16_t  EndTime;



#define BUFFER_SIZE 16

typedef struct
{
    uint16_t array[BUFFER_SIZE];                    /*!< Buffer array to storage noisy signal to be filtered */
    uint16_t bufferIndex;                           /*!< Buffer array storage index */
} BUFFER_TYPE;

typedef struct
{
  uint16_t   IuOffset;       
  uint32_t   IuOffsetSum;   
  uint16_t   IvOffset;       
  uint32_t   IvOffsetSum;    
  uint16_t   Iw_busOffset;   
  uint32_t   Iw_busOffsetSum;
	uint16_t   I_Off_Average;
  uint16_t   OffsetCount;    
  uint8_t    OffsetFlag;      
}CurrentOffset;   //电流偏移量
GLOBAL_ADC_ CurrentOffset  mcCurOffset;

typedef struct
{
    unsigned System_Start :1;
    unsigned Motor_Stop :1;
	  unsigned Angle_Mask :1;
	  unsigned ChangePhase :1;
	  unsigned flag_SpeedTime:1;
	  unsigned SwapFlag :1;
}sysflag;

typedef struct {
	     float     PCB_Temp;              //PCB温度
			 float     BUS_Curr ;             //母线电流
	     float     PhaseW_Voltage ;       //W相电压
	     float     PhaseU_Voltage;        //U相电压
	     float     PhaseV_Voltage;        //V相电压
	     float     BUS_Voltage ;          //母线电压DC Bus  Voltage	     
	     float     RP_speed_Voltage ;     // 电位器电压 RP1_Voltage
	     int32_t   Offset_BUS_Curr;       // 电流偏移量
	     BUFFER_TYPE busCur;                             /*!< Buffer to storage bus current in bldc control */
	     int32_t   Coeff_filterK1;        // 一阶低通滤波器系数1
		   int32_t   Coeff_filterK2;        // 一阶低通滤波器系数2
 }ADCSamp;
GLOBAL_ADC_  ADCSamp   ADCSampPare;
GLOBAL_ADC_  volatile  uint16_t  RegularConvData_Tab[2];
GLOBAL_ADC_  float     Current_Filter(void);
GLOBAL_ADC_  uint16_t  UserRequireSpeed;
GLOBAL_ADC_  uint8_t   Falg_Waiting;
GLOBAL_ADC_  uint8_t   DELTADUTYCYCLE;
GLOBAL_ADC_  uint16_t  Global_Current;
GLOBAL_ADC_  uint16_t  Mask_TIME;
  
GLOBAL_ADC_ uint16_t   motor_a_stall_cnt,motor_b_stall_cnt,motor_c_stall_cnt;
GLOBAL_ADC_ sysflag    sysflags;

void ADC_Configuration(void);
void NVIC_Configuration(void);

GLOBAL_ADC_ void FaultMos_OverTemperature(void);
GLOBAL_ADC_ void BemfCheck(void);
GLOBAL_ADC_ void JudgeErrorCommutation(void);
GLOBAL_ADC_ void Fault_OverUnderVoltage(void);
GLOBAL_ADC_ void Cal_AverCurrent(void);
GLOBAL_ADC_ void Instant_Current_Cale(void);
GLOBAL_ADC_ void Fault_Detection(void);
GLOBAL_ADC_ void Fault_Soft_Overcurrent(void);
GLOBAL_ADC_ void Angle_Dealwith(void);

#define	Ku	1.01934			//由于分压电路不一样，该系数为了使分压系数一致

#endif

