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
volatile    uint16_t  RegularConvData_Tab[2]={0};
uint8_t     Flag_Stall=0;
uint8_t     DELTADUTYCYCLE=1;
uint16_t    Global_Current=0;
uint16_t    motor_a_stall_cnt=0,motor_b_stall_cnt=0,motor_c_stall_cnt=0;
uint16_t    Mask_TIME=0;

void NVIC_Configuration(void)//100us        //配置中断函数优先级
{
	NVIC_InitTypeDef NVIC_InitStructure; 

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);           //设置中断优先级组
		
	/* Enable the TIM2 Interrupt    用来60ms定时用的  */
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;                 //配置中断源
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;       //配置抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;              //配置子优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                 //使能中断通道
	NVIC_Init(&NVIC_InitStructure);                                 //调用初始化函数

	/* Enable the TIM3 Interrupt        100us定时用的      */
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Enable the TIM4 Interrupt        1ms定时用的      */
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
		
	/*TIM1_CC_IRQn  定时器1比较中断     注意到定时器1比较特殊，不象通用定时器，通用定时器中断只有1个，而定时器1有4个单独的中断矢量        */
	// NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn;
	// NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	// NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	// NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	// NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = ADC1_2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Enable the DMA Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
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

/*****************************************************************
  * @file     ADC_Configuration
  * @brief    完成ADC 初始化
  * @param    无
  * @retval   无
  ***************************************************************/
void ADC_Configuration(void)
{
	/*开启时钟*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);	//开启ADC1的时钟
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);		//开启DMA1的时钟
	
	/*设置ADC时钟*/
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);						//选择时钟6分频，ADCCLK = 72MHz / 6 = 12MHz
	
	/*GPIO初始化*/
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4 | GPIO_Pin_5;
	//GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);					//将PA0、PA1、PA2和PA3引脚初始化为模拟输入
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	//GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);					//将PA0、PA1、PA2和PA3引脚初始化为模拟输入
	
	/*ADC初始化*/
	ADC_InitTypeDef ADC_InitStructure;											//定义结构体变量
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;							//模式，选择独立模式，即单独使用ADC1
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;						//数据对齐，选择右对齐
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;			//外部触发，使用软件触发，不需要外部触发
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;							//连续转换，使能，每转换一次规则组序列后立刻开始下一次转换
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;								//扫描模式，使能，扫描规则组的序列，扫描数量由ADC_NbrOfChannel确定
	ADC_InitStructure.ADC_NbrOfChannel = 2;										//通道数，为4，扫描规则组的前4个通道
	ADC_Init(ADC1, &ADC_InitStructure);											//将结构体变量交给ADC_Init，配置ADC1

		/*规则组通道配置*/
	// ADC_RegularChannelConfig(ADC1, U_Vol_Channel, 1, ADC_SampleTime_41Cycles5); 
	// ADC_RegularChannelConfig(ADC1, V_Vol_Channel, 2, ADC_SampleTime_41Cycles5);     
	// ADC_RegularChannelConfig(ADC1, BatVol_Channel, 1, ADC_SampleTime_41Cycles5); 
	// ADC_RegularChannelConfig(ADC1, W_Vol_Channel, 4, ADC_SampleTime_41Cycles5);
	ADC_RegularChannelConfig(ADC1, Current_Channel, 1, ADC_SampleTime_41Cycles5); 
	ADC_RegularChannelConfig(ADC1, PCB_Temp_Channel, 2, ADC_SampleTime_41Cycles5);
	
	
	ADC_InjectedSequencerLengthConfig(ADC1, 3);  
	ADC_InjectedChannelConfig(ADC1, U_Vol_Channel, 1, ADC_SampleTime_7Cycles5);
	ADC_InjectedChannelConfig(ADC1, V_Vol_Channel, 2, ADC_SampleTime_7Cycles5);
	ADC_InjectedChannelConfig(ADC1, W_Vol_Channel, 3, ADC_SampleTime_7Cycles5);
	// ADC_InjectedChannelConfig(ADC1, Current_Channel, 4, ADC_SampleTime_7Cycles5);
	ADC_ExternalTrigInjectedConvConfig(ADC1, ADC_ExternalTrigInjecConv_T1_CC4); 
	ADC_ExternalTrigInjectedConvCmd(ADC1, ENABLE);
	ADC_ITConfig(ADC1,ADC_IT_JEOC,ENABLE);

	/*DMA初始化*/
	DMA_InitTypeDef DMA_InitStructure;											//定义结构体变量
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;				//外设基地址，给定形参AddrA
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;	//外设数据宽度，选择半字，对应16为的ADC数据寄存器
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;			//外设地址自增，选择失能，始终以ADC数据寄存器为源
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)RegularConvData_Tab;					//存储器基地址，给定存放AD转换结果的全局数组AD_Value
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;			//存储器数据宽度，选择半字，与源数据宽度对应
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;						//存储器地址自增，选择使能，每次转运后，数组移到下一个位置
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;							//数据传输方向，选择由外设到存储器，ADC数据寄存器转到数组
	DMA_InitStructure.DMA_BufferSize = 2;										//转运的数据大小（转运次数），与ADC通道数一致
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;								//模式，选择循环模式，与ADC的连续转换一致
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;								//存储器到存储器，选择失能，数据由ADC外设触发转运到存储器
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;						//优先级，选择中等
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);								//将结构体变量交给DMA_Init，配置DMA1的通道1

	DMA_ITConfig(DMA1_Channel1, DMA_IT_TC , ENABLE);

	/* 开启内部温度传感器和 Vrefint 通道 */
	ADC_TempSensorVrefintCmd(ENABLE);
	
	/*DMA和ADC使能*/
	DMA_Cmd(DMA1_Channel1, ENABLE);							//DMA1的通道1使能
	ADC_DMACmd(ADC1, ENABLE);								//ADC1触发DMA1的信号使能
	ADC_Cmd(ADC1, ENABLE);									//ADC1使能
	
	/*ADC校准*/
	ADC_ResetCalibration(ADC1);								//固定流程，内部有电路会自动执行校准
	while (ADC_GetResetCalibrationStatus(ADC1) == SET);
	ADC_StartCalibration(ADC1);
	while (ADC_GetCalibrationStatus(ADC1) == SET);
	
	/*ADC触发*/
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);	//软件触发ADC开始工作，由于ADC处于连续转换模式，故触发一次后ADC就可以一直连续不断地工作
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
      buffer->bufferIndex++;	//每次存一个值
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
#if SHF_TEST
void BemfCheck(void)  
{
	static uint16_t U_Array[100];
	static uint16_t V_Array[100];
	static uint16_t W_Array[100];
	static uint16_t u_idx;
	if(sysflags.Angle_Mask==0)//等待续流结束
	{	
		Motor.UVoltage = ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_1);
		Motor.VVoltage = ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_2);
		Motor.WVoltage = ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_3);
		Motor.Uab = Motor.UVoltage - Motor.VVoltage;
		Motor.Ubc = Motor.VVoltage - Motor.WVoltage;
		Motor.Uca = Motor.WVoltage - Motor.UVoltage;
		if(Motor.PhaseCnt == 3 || Motor.PhaseCnt == 6)		//采样U相
		{
			Motor.Ea = (Motor.Uab - Motor.Uca) / 2;
		}
		else if(Motor.PhaseCnt == 2 || Motor.PhaseCnt == 5)	//采样V相
		{
			Motor.Eb = (Motor.Ubc - Motor.Uab) / 2;
		}
		else if(Motor.PhaseCnt == 1 || Motor.PhaseCnt == 4)	//采样W相
		{
			Motor.Ec = (Motor.Uca - Motor.Ubc) / 2;
		}
	}
	if((sysflags.ChangePhase==0)&&(sysflags.Angle_Mask==0))
	{
		switch(Motor.PhaseCnt)			//此时万一失步了，相位不就不准了
		{
			case 1:
				if(Motor.Ec < 0)   	//长时间检测不到过零点就强制换相
				{
					Sysvariable.BlankingCnt++;
					if(Sysvariable.BlankingCnt>=Filter_Count)	//这个取0，因为本来就是延迟相位了，再延迟就错了
					{
						Sysvariable.BlankingCnt=0;
						Motor.PhaseCnt = 2;      	//换相		
						CalcSpeedTime();  	
					}									 
				}	 						
				break;
						 
			case 2:
				if(Motor.Eb > 0)		//为什么是大于？
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
				if(Motor.Ea < 0)
				{
					Sysvariable.BlankingCnt++;
					if(Sysvariable.BlankingCnt>=Filter_Count)
					{
						Sysvariable.BlankingCnt=0;
						Motor.PhaseCnt =4;
						CalcSpeedTime();		//检验一下此处的换相时间间隔对不对
					}									 
				}
				break;
			case 4:
				if(Motor.Ec > 0)
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
				if(Motor.Eb < 0)
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
				if(u_idx < 100)
				{
					U_Array[u_idx] = Motor.Ea;
					V_Array[u_idx] = Motor.Eb;
					W_Array[u_idx] = Motor.Ec;		//示波器停下来看一眼
					u_idx ++;
				}
				if(Motor.Ea > 0)
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
#else
void BemfCheck(void)  
{ 
	static uint16_t  V_Bus_Half=0; 
	static uint16_t U_Vol_run;
	static uint16_t V_Vol_run;
	static uint16_t W_Vol_run;
	static uint16_t U_Array[100];
	static uint16_t V_Array[100];
	static uint16_t W_Array[100];
	static uint16_t u_idx;
	if(sysflags.Angle_Mask==0)//等待续流结束
	{	
		if(Motor.PhaseCnt == 3 || Motor.PhaseCnt == 6)		//采样U相
		{
			Motor.Bemf = ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_1);
		}
		else if(Motor.PhaseCnt == 2 || Motor.PhaseCnt == 5)	//采样V相
		{
			Motor.Bemf = ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_2);
		}
		else if(Motor.PhaseCnt == 1 || Motor.PhaseCnt == 4)	//采样W相
		{
			Motor.Bemf = ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_3);
		}
		V_Bus_Half=(uint16_t)(RegularConvData_Tab[0]*0.5);	//母线电压的一半 *Ku待定
		U_Vol_run = ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_1);
		V_Vol_run = ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_2);
		W_Vol_run = ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_3);
		//u_idx ++;
	}
	if((sysflags.ChangePhase==0)&&(sysflags.Angle_Mask==0))
	{
		switch(Motor.PhaseCnt)			//此时万一失步了，相位不就不准了
		{
#if SHF_TEST2
			case 1:
				if((Motor.Bemf<V_Bus_Half) || (TIM2->CNT >= Sysvariable.LastDragTime * 3))   	//长时间检测不到过零点就强制换相
				{
					Sysvariable.BlankingCnt++;
					if(Sysvariable.BlankingCnt>=Filter_Count)	//这个取0，因为本来就是延迟相位了，再延迟就错了
					{
						Sysvariable.BlankingCnt=0;
						Sysvariable.ADCTimeCnt = 0;
						Motor.PhaseCnt = 2;      	//换相		
						CalcSpeedTime();  	
					}									 
				}	 						
				break;
						 
			case 2:
				if((Motor.Bemf>V_Bus_Half) || (TIM2->CNT >= Sysvariable.LastDragTime * 3))		//为什么是大于？
				{
					Sysvariable.BlankingCnt++;
					if(Sysvariable.BlankingCnt>=Filter_Count)
					{
						Sysvariable.BlankingCnt=0;
						Sysvariable.ADCTimeCnt = 0;
						Motor.PhaseCnt =3;
						CalcSpeedTime();
					}
				}
			break;
			case 3:
				if((Motor.Bemf<V_Bus_Half) || (TIM2->CNT >= Sysvariable.LastDragTime * 3))
				{
					Sysvariable.BlankingCnt++;
					if(Sysvariable.BlankingCnt>=Filter_Count)
					{
						Sysvariable.BlankingCnt=0;
						Sysvariable.ADCTimeCnt = 0;
						Motor.PhaseCnt =4;
						CalcSpeedTime();		//检验一下此处的换相时间间隔对不对
					}									 
				}
				break;
			case 4:
				if((Motor.Bemf>V_Bus_Half) || (TIM2->CNT >= Sysvariable.LastDragTime * 3))
				{
					Sysvariable.BlankingCnt++;
					if(Sysvariable.BlankingCnt>=Filter_Count)
					{
						Sysvariable.BlankingCnt=0;
						Sysvariable.ADCTimeCnt = 0;
						Motor.PhaseCnt =5;
						CalcSpeedTime();
					}
				}
				break;
			case 5:
				if(u_idx < 100)
				{
					U_Array[u_idx] = U_Vol_run;
					V_Array[u_idx] = V_Vol_run;
					W_Array[u_idx] = W_Vol_run;		//示波器停下来看一眼
					u_idx ++;
				}
				if((Motor.Bemf<V_Bus_Half) || (TIM2->CNT >= Sysvariable.LastDragTime * 3))
				{
					Sysvariable.BlankingCnt++;
					if(Sysvariable.BlankingCnt>=Filter_Count)
					{
						Sysvariable.BlankingCnt=0;
						Sysvariable.ADCTimeCnt = 0;
						Motor.PhaseCnt =6;
						CalcSpeedTime();	
					}									 
				}
				break;
			case 6:
				if((Motor.Bemf>V_Bus_Half) || (TIM2->CNT >= Sysvariable.LastDragTime * 3))
				{
					Sysvariable.BlankingCnt++;
					if(Sysvariable.BlankingCnt>=Filter_Count)
					{
						Sysvariable.BlankingCnt=0;
						Sysvariable.ADCTimeCnt = 0;
						Motor.PhaseCnt =1;
						CalcSpeedTime();
					}
				}																 
				break;
#else
			case 1:
				if(Motor.Bemf<V_Bus_Half)   	//长时间检测不到过零点就强制换相
				{
					Sysvariable.BlankingCnt++;
					if(Sysvariable.BlankingCnt>=Filter_Count)	//这个取0，因为本来就是延迟相位了，再延迟就错了
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
						CalcSpeedTime();		//检验一下此处的换相时间间隔对不对
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
				if(u_idx < 100)
				{
					U_Array[u_idx] = U_Vol_run;
					V_Array[u_idx] = V_Vol_run;
					W_Array[u_idx] = W_Vol_run;		//示波器停下来看一眼
					u_idx ++;
				}
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
#endif
			default:
				break;
		}
	} 
}
#endif
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
	ADCSampPare.BUS_Voltage = RegularConvData_Tab[0]*3.3*72.7/4095/4.7;           //DC母线电压		
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
 输出参数  : void		//此处直接取电流AD采样值Global_Current
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
	if(ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_4)>mcCurOffset.I_Off_Average)
	{
		Global_Current=(ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_4)-mcCurOffset.I_Off_Average);//*CURRENT_COFF; AD值
	}
	else
	{
		//Global_Current=0;
	}
}
//没用到
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
		ADCSampPare.PCB_Temp   =  RegularConvData_Tab[1]*3.3/4095;           //电路板温度	
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
