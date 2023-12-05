#ifndef   _ADC_FILE
#define   _ADC_FILE

#include "Analog.h"
#include "BLDC.h"
#include "Timer_ISR.h"
#include "Timer.h"
#include "GPIO_Init.h"
#include "PI_Cale.h"
#include "User_Config.h"
#define  ADC1_DR_Address    0x40012440

ADCSamp        ADCSampPare=ADCSamp_DEFAULTS;
CurrentOffset  mcCurOffset= mcCurOffset_DEFAULTS;

uint16_t       UserRequireSpeed=0;
uint8_t        Filter_Count=0;  //

float       Aver_Current=0;
float       DC_TotalCurrent=0;
volatile    uint16_t  RegularConvData_Tab[6]={0};
uint8_t     Flag_Stall=0;
uint8_t     DELTADUTYCYCLE=1;
uint16_t    Global_Current=0;
uint16_t    motor_a_stall_cnt=0,motor_b_stall_cnt=0,motor_c_stall_cnt=0;
uint16_t    Mask_TIME=0;

void NVIC_Configuration(void)//100us        //配置中断函数优先级
{
    NVIC_InitTypeDef NVIC_InitStructure; 
    
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);           //设置中断优先级组
     
    /* Enable the TIM2 Interrupt    用来100us定时用的  */
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;                 //配置中断源
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;       //配置抢占优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;              //配置子优先级
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                 //使能中断通道
    NVIC_Init(&NVIC_InitStructure);                                 //调用初始化函数
    
    /* Enable the TIM4 Interrupt        定时器4用来干什么的?   1ms定时用的      */
    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
     
    /*TIM1_CC_IRQn               定时器1比较中断     注意到定时器1比较特殊，不象通用定时器，通用定时器中断只有1个，而定时器1有4个单独的中断矢量        */
    NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

	/* Enable the DMA Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
     
    /*刹车TIM1_BRK_IRQn*/
#if(MOTOR_BRAKE_ENABLE == 1)//关闭刹车功能   
    NVIC_InitStructure.NVIC_IRQChannel = TIM1_BRK_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
#endif     
}

/*  
* 函数名：ADC1_GPIO_Config  
* 描述  ：使能 ADC1 和 DMA1 的时钟，初始化   
* 输入  : 无  
* 输出  ：无  
* 调用  ：内部调用  
*/
static void ADC1_GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;   
       
    /* Enable DMA clock */  
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);   
       
    /* Enable ADC1 and GPIOC clock */  
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);      
       
    /* Configure PA0  as analog input */     
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4 | GPIO_Pin_5;   
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;   //引脚模式为模拟输入
	//GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);   // PCA,输入时不用设置速率 
    
    /* Configure P13  as analog input */  //TEMP   
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;   
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	//GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);   // PB,输入时不用设置速率 
}

/* 函数名：ADC1_Mode_Config  
* 描述  ：配置 ADC1 的工作模式为 DMA 模式  
* 输入  : 无  
* 输出  ：无  
* 调用  ：内部调用  
*cpu温度值= （1.43V - ADC16值/4096*3.3）/Avg_Slope(4.3mV/摄氏度) + 25
*/ 
static void ADC1_Mode_Config(void)
{
    DMA_InitTypeDef DMA_InitStructure;   
    ADC_InitTypeDef ADC_InitStructure;   
       
    /* DMA channel1 configuration */  
    DMA_DeInit(DMA1_Channel1);   
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC1_DR_Address;  //ADC数据存放源地址    
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; //外设数据宽度  半字
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;		//外设地址始终是ADC
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)RegularConvData_Tab;  //目标缓存地址    
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;//内存的数据宽度
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  //设定寄存器内存地址增加  每次DMA将外设寄存器中的值传到内存	
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;   //外设数据源
	DMA_InitStructure.DMA_BufferSize = 5;     //转运次数，与ADC通道数一致，存放AD的数据大小,本程序一共采集6个数据
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;  //循环传输
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;  //DMA通道优先级     
    DMA_Init(DMA1_Channel1, &DMA_InitStructure);

    /*选择时钟6分频，ADCCLK = 72MHz / 6 = 12MHz*/ 
    RCC_ADCCLKConfig(RCC_PCLK2_Div6); 
    /*配置 ADC1 的通道 0 为 55.5 个采样周期 */    //10us采样一次
	ADC_RegularChannelConfig(ADC1, U_Vol_Channel, 1, ADC_SampleTime_55Cycles5); 
	ADC_RegularChannelConfig(ADC1, V_Vol_Channel, 2, ADC_SampleTime_55Cycles5);     
	ADC_RegularChannelConfig(ADC1, BatVol_Channel, 3, ADC_SampleTime_55Cycles5); 
	ADC_RegularChannelConfig(ADC1, W_Vol_Channel, 4, ADC_SampleTime_55Cycles5);
    ADC_RegularChannelConfig(ADC1, Current_Channel, 5, ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1, PCB_Temp_Channel, 6, ADC_SampleTime_55Cycles5); 
    //ADC_RegularChannelConfig(ADC1, PCB_Temp_Channel, 3, ADC_SampleTime_55Cycles5);  

    /* ADC1 configuration */     
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;  /*独立 ADC 模式*/   //只用ADC1
    ADC_InitStructure.ADC_ScanConvMode = ENABLE ;   /*扫描模式，扫描模式用于多通道采集*/   
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;  /*开启连续转换模式，即不停地进行 ADC 转换*/   
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; /*不使用外部触发转换*/   
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;  /*采集数据右对齐*/   
    ADC_InitStructure.ADC_NbrOfChannel = 5;     /*要转换的通道数目 6*/   
    ADC_Init(ADC1, &ADC_InitStructure);   

	DMA_ITConfig(DMA1_Channel1, DMA_IT_TC , ENABLE);
       
    /* 开启内部温度传感器和 Vrefint 通道 */
    ADC_TempSensorVrefintCmd(ENABLE);
    
    /* Enable ADC1 DMA */  
    ADC_DMACmd(ADC1, ENABLE);   
    DMA_Cmd(DMA1_Channel1, ENABLE);
    /* Enable ADC1 */  
    ADC_Cmd(ADC1, ENABLE);   
        
     /*复位校准寄存器 */      
    ADC_ResetCalibration(ADC1);
    /*等待校准寄存器复位完成 */  
    while(ADC_GetResetCalibrationStatus(ADC1));   
       
    /* ADC 校准 */  
    ADC_StartCalibration(ADC1);   
    /* 等待校准完成*/  
    while(ADC_GetCalibrationStatus(ADC1));   
       
    /* 由于没有采用外部触发，所以使用软件触发 ADC 转换 */    
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}
/*****************************************************************
  * @file     ADC_Configuration
  * @brief    完成ADC 初始化
  * @param    无
  * @retval   无
  ***************************************************************/
void ADC_Configuration(void)
{
	ADC1_GPIO_Config();
	ADC1_Mode_Config();
}
/************************************************************************************
 函 数 名  : StorageDataCyclic
 功能描述  : 数据缓存
 输入参数  : 无
 输出参数  : void
************************************************************************************/
void StorageDataCyclic(BUFFER_TYPE *buffer, uint16_t data)
{
    *(buffer->array + buffer->bufferIndex) = data;
      buffer->bufferIndex++;
    if (buffer->bufferIndex >= BUFFER_SIZE)
    {
        buffer->bufferIndex = 0;
    }
}
/************************************************************************************
 函 数 名  : Filter_AverageCalc
 功能描述  : 平均值滤波
 输入参数  : 无
 输出参数  : void
************************************************************************************/
uint16_t Filter_AverageCalc(uint16_t *buffer, uint16_t n)
{
    uint16_t Max = 0, Min = 0, j = 0;
    uint16_t Temp = 0;

    Max = Min = Temp = *buffer;

    for (j = 1; j < n; j++)
    {
        if (Max < *(buffer + j))
        {
            Max = *(buffer + j);
        }
        else if (Min > *(buffer + j))
        {
            Min = *(buffer + j);
        }
        else
        {
        }

        Temp += *(buffer + j);
    }

    Temp -= (Max + Min);

    if (n > 2)
    {
        Temp /= (n - 2);
    }

    return Temp;
}
/************************************************************************************
 函 数 名  : BemfCheck
 功能描述  : 反电动势过零点检测
 输入参数  : 无
 输出参数  : void
************************************************************************************/
void BemfCheck(void)  
{ 
	static uint16_t  V_Bus_Half=0; 
	if(sysflags.Angle_Mask==0)//等待续流结束
	{	
		if(Motor.PhaseCnt == 3 || Motor.PhaseCnt == 6)		//采样U相
		{
			Motor.Bemf=RegularConvData_Tab[0];
		}
		else if(Motor.PhaseCnt == 2 || Motor.PhaseCnt == 5)	//采样V相
		{
			Motor.Bemf=RegularConvData_Tab[1];
		}
		else if(Motor.PhaseCnt == 1 || Motor.PhaseCnt == 4)	//采样W相
		{
			Motor.Bemf=RegularConvData_Tab[3];
		}
		//Motor.Bemf=RegularConvData_Tab[4]; 		//端电压采样	//待修改 改为0 | 1 | 3，需要先判断导通相
		V_Bus_Half=(uint16_t)(RegularConvData_Tab[2]*0.5);	//母线电压的一半
	}
	if((sysflags.ChangePhase==0)&&(sysflags.Angle_Mask==0))
	{
		switch(Motor.PhaseCnt)
		{
			case 1:
				if(Motor.Bemf<V_Bus_Half)   
				{
					Sysvariable.BlankingCnt++;
					if(Sysvariable.BlankingCnt>=Filter_Count)	//滤波延迟补偿
					{
						Sysvariable.BlankingCnt=0;
						Motor.PhaseCnt = 2;      	//换相		
						CalcSpeedTime();  	
					}									 
				}	 						
				break;
						 
			case 2:
			if(Motor.Bemf>V_Bus_Half)		//为什么是大于？
			{
				Sysvariable.BlankingCnt++;
				if(Sysvariable.BlankingCnt>=Filter_Count)
				{
					Sysvariable.BlankingCnt=0;
					Motor.PhaseCnt =3;
					CalcSpeedTime();
				}
			}
			break;
			case 3:
				if(Motor.Bemf<V_Bus_Half)
				{
					Sysvariable.BlankingCnt++;
					if(Sysvariable.BlankingCnt>=Filter_Count)
					{
						Sysvariable.BlankingCnt=0;
						Motor.PhaseCnt =4;
						CalcSpeedTime();	
					}									 
				}
				break;
			case 4:
				if(Motor.Bemf>V_Bus_Half)
				{
					Sysvariable.BlankingCnt++;
					if(Sysvariable.BlankingCnt>=Filter_Count)
					{
						Sysvariable.BlankingCnt=0;
						Motor.PhaseCnt =5;
						CalcSpeedTime();
					}
				}
				break;
			case 5:
				if(Motor.Bemf<V_Bus_Half)
				{
					Sysvariable.BlankingCnt++;
					if(Sysvariable.BlankingCnt>=Filter_Count)
					{
						Sysvariable.BlankingCnt=0;
						Motor.PhaseCnt =6;
						CalcSpeedTime();	
					}									 
				}
				break;
			case 6:
				if(Motor.Bemf>V_Bus_Half)
				{
					Sysvariable.BlankingCnt++;
					if(Sysvariable.BlankingCnt>=Filter_Count)
					{
						Sysvariable.BlankingCnt=0;
						Motor.PhaseCnt =1;
						CalcSpeedTime();
					}
				}																 
				break;
			default:
				break;
		}
	} 
}
/*****************************************************************************
 函 数 名  : JudgeErrorCommutation
 功能描述  : 判断换相错误/长时间不换相  
 输入参数  : 无
 输出参数  : void
*****************************************************************************/
void JudgeErrorCommutation()
{	
	if(Motor.LastPhase ==  Motor.PhaseCnt)
	{
		if(++Sysvariable.SamePhaseCnt >= 3200)	// 62.5us*n==200ms没有发生换向
		{
			Sysvariable.SamePhaseCnt = 0;	
			Sysvariable.Stop_Time=Motor_DelayTime;				
		  MotorStop();
			mcFault = Overtime_PhaseChange;
		}
	}
	else
	{
		Sysvariable.SamePhaseCnt = 0;
	}
	
	Motor.LastPhase = Motor.PhaseCnt;
}
/************************************************************************************
 函 数 名  : Fault_OverUnderVoltage
 功能描述  : 过压或欠压保护函数
 输入参数  : 无
 输出参数  : void
************************************************************************************/
void Fault_OverUnderVoltage(void)
{
	static	uint16_t  VoltageProCnt = 0;
	ADCSampPare.BUS_Voltage = RegularConvData_Tab[2]*3.3*72.7/4095/4.7;           //DC母线电压		
	if((ADCSampPare.BUS_Voltage < Min_BUS_Voltage) || (ADCSampPare.BUS_Voltage > Max_BUS_Voltage))
	{
		VoltageProCnt ++;
		if(VoltageProCnt >= 1000)
		{
			VoltageProCnt = 0;
	    Sysvariable.Stop_Time=Motor_DelayTime;			
			MotorStop();
			mcFault = OverUnderVoltage;

		}
	}
	else
	{
		VoltageProCnt = 0;
	}
}
/************************************************************************************
 函 数 名  : Cal_AverCurrent
 功能描述  : 计算平均电流函数  !!!!!电流负数的处理
 输入参数  : 无
 输出参数  : void
************************************************************************************/
void Cal_AverCurrent(void)
{
	static  uint32_t  ADTotalCurrent=0;
	static  int8_t Current_AverTimes=0;
  uint16_t	  Aver_Current=0;
   if(Filter_Mode==Filter_Average)
	 {
			StorageDataCyclic(&ADCSampPare.busCur, Global_Current);
			Motor.Current = Filter_AverageCalc(ADCSampPare.busCur.array, BUFFER_SIZE)*27;//  修正系数  0.82  22.3*0.82
	 }
	 else if(Filter_Mode==Filter_Function)
	 {
		 ADTotalCurrent+=Global_Current;
     Current_AverTimes++;
	 	 if(Current_AverTimes>=Filter_Const1)//平均电流计算
	   {
			 Current_AverTimes=0;
			 Aver_Current=27*ADTotalCurrent/Filter_Const1; //MA
	  	 ADTotalCurrent=Global_Current;
		   FirstOrder_LPF_Cacl(Aver_Current,Motor.Current,0.1);

		 }
	 }
	else if(Filter_Mode==Coff_Multi)   
	{
		 Motor.Current=Global_Current;
	}
	 
}



/************************************************************************************
 函 数 名  : Cal_AverCurrent
 功能描述  : 计算平均电流函数 62.5us执行一次
 输入参数  : 无
 输出参数  : void
************************************************************************************/

void  Instant_Current_Cale(void)
{
 if(mcState==mcRun)
 {
	  if(RegularConvData_Tab[4]>mcCurOffset.I_Off_Average)
		{
	    Global_Current=(RegularConvData_Tab[4]-mcCurOffset.I_Off_Average);//*CURRENT_COFF; AD值
		}
		else
		{
			//Global_Current=0;
		}
	  
	}
}
void Fault_Soft_Overcurrent(void)
{
		static  uint16_t   Soft_Time;
		if(Global_Current>Max_BUS_Curr) //10A/0.0248=403
		{
				Soft_Time++;
		}
		else
		{
				Soft_Time=0;
		}
		if(Soft_Time>1000)//1000*1ms=1s  限制长时间超功率运行
		{
			Soft_Time=0;
			Sysvariable.Stop_Time=Motor_DelayTime;			
			MotorStop();
			mcState=mcStop;
			mcFault = OverCurrent;
		}
}
/************************************************************************************
 函 数 名  : PCB_OverTemperature
 功能描述  : PCB过温函数
 输入参数  : 无
 输出参数  : void
************************************************************************************/

void FaultMos_OverTemperature(void) 
{  
    static  uint8_t  OverTempcnt=0;	  
		ADCSampPare.PCB_Temp   =  RegularConvData_Tab[5]*3.3/4095;           //电路板温度	
		if(ADCSampPare.PCB_Temp<=Max_PCB_Temp) //PCB温度
		{
			  OverTempcnt++;
			  if(OverTempcnt>=10)
				{
					OverTempcnt=0;
					Sysvariable.Stop_Time=Motor_DelayTime;			
				  MotorStop();           //立马停止输出 
					mcState=mcStop;


				}		
		}
		else
		{
			OverTempcnt=0;
		}
		
}
/************************************************************************************
 函 数 名  : Fault_Detection
 功能描述  :故障检测函数
 输入参数  : 无
 输出参数  : void
************************************************************************************/
void Fault_Detection(void)
{

		Fault_Soft_Overcurrent();
		Fault_OverUnderVoltage();
		FaultMos_OverTemperature();
}
	

void Angle_Dealwith(void)
{
	if(Motor.Duty<DUTYTHRESHOLD1)//低duty
	{
		Mask_TIME =Low_DutyMask_Time ;  //续流屏蔽时间  time*4us
		Filter_Count=Delay_Filter;
	}
	else//高duty
	{
		Mask_TIME =High_DutyMask_Time; 
		Filter_Count=1;
	}
	
}
#endif
