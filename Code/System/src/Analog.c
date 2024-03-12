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

void NVIC_Configuration(void)//100us        //�����жϺ������ȼ�
{
	NVIC_InitTypeDef NVIC_InitStructure; 

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);           //�����ж����ȼ���
		
	/* Enable the TIM2 Interrupt    ����60ms��ʱ�õ�  */
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;                 //�����ж�Դ
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;       //������ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;              //���������ȼ�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                 //ʹ���ж�ͨ��
	NVIC_Init(&NVIC_InitStructure);                                 //���ó�ʼ������

	/* Enable the TIM3 Interrupt        100us��ʱ�õ�      */
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Enable the TIM4 Interrupt        1ms��ʱ�õ�      */
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
		
	/*TIM1_CC_IRQn  ��ʱ��1�Ƚ��ж�     ע�⵽��ʱ��1�Ƚ����⣬����ͨ�ö�ʱ����ͨ�ö�ʱ���ж�ֻ��1��������ʱ��1��4���������ж�ʸ��        */
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
     
    /*ɲ��TIM1_BRK_IRQn*/
#if(MOTOR_BRAKE_ENABLE == 1)//�ر�ɲ������   
    NVIC_InitStructure.NVIC_IRQChannel = TIM1_BRK_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
#endif     
}

/*****************************************************************
  * @file     ADC_Configuration
  * @brief    ���ADC ��ʼ��
  * @param    ��
  * @retval   ��
  ***************************************************************/
void ADC_Configuration(void)
{
	/*����ʱ��*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);	//����ADC1��ʱ��
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);		//����DMA1��ʱ��
	
	/*����ADCʱ��*/
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);						//ѡ��ʱ��6��Ƶ��ADCCLK = 72MHz / 6 = 12MHz
	
	/*GPIO��ʼ��*/
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4 | GPIO_Pin_5;
	//GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);					//��PA0��PA1��PA2��PA3���ų�ʼ��Ϊģ������
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	//GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);					//��PA0��PA1��PA2��PA3���ų�ʼ��Ϊģ������
	
	/*ADC��ʼ��*/
	ADC_InitTypeDef ADC_InitStructure;											//����ṹ�����
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;							//ģʽ��ѡ�����ģʽ��������ʹ��ADC1
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;						//���ݶ��룬ѡ���Ҷ���
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;			//�ⲿ������ʹ���������������Ҫ�ⲿ����
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;							//����ת����ʹ�ܣ�ÿת��һ�ι��������к����̿�ʼ��һ��ת��
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;								//ɨ��ģʽ��ʹ�ܣ�ɨ�����������У�ɨ��������ADC_NbrOfChannelȷ��
	ADC_InitStructure.ADC_NbrOfChannel = 2;										//ͨ������Ϊ4��ɨ��������ǰ4��ͨ��
	ADC_Init(ADC1, &ADC_InitStructure);											//���ṹ���������ADC_Init������ADC1

		/*������ͨ������*/
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

	/*DMA��ʼ��*/
	DMA_InitTypeDef DMA_InitStructure;											//����ṹ�����
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;				//�������ַ�������β�AddrA
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;	//�������ݿ�ȣ�ѡ����֣���Ӧ16Ϊ��ADC���ݼĴ���
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;			//�����ַ������ѡ��ʧ�ܣ�ʼ����ADC���ݼĴ���ΪԴ
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)RegularConvData_Tab;					//�洢������ַ���������ADת�������ȫ������AD_Value
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;			//�洢�����ݿ�ȣ�ѡ����֣���Դ���ݿ�ȶ�Ӧ
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;						//�洢����ַ������ѡ��ʹ�ܣ�ÿ��ת�˺������Ƶ���һ��λ��
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;							//���ݴ��䷽��ѡ�������赽�洢����ADC���ݼĴ���ת������
	DMA_InitStructure.DMA_BufferSize = 2;										//ת�˵����ݴ�С��ת�˴���������ADCͨ����һ��
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;								//ģʽ��ѡ��ѭ��ģʽ����ADC������ת��һ��
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;								//�洢�����洢����ѡ��ʧ�ܣ�������ADC���败��ת�˵��洢��
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;						//���ȼ���ѡ���е�
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);								//���ṹ���������DMA_Init������DMA1��ͨ��1

	DMA_ITConfig(DMA1_Channel1, DMA_IT_TC , ENABLE);

	/* �����ڲ��¶ȴ������� Vrefint ͨ�� */
	ADC_TempSensorVrefintCmd(ENABLE);
	
	/*DMA��ADCʹ��*/
	DMA_Cmd(DMA1_Channel1, ENABLE);							//DMA1��ͨ��1ʹ��
	ADC_DMACmd(ADC1, ENABLE);								//ADC1����DMA1���ź�ʹ��
	ADC_Cmd(ADC1, ENABLE);									//ADC1ʹ��
	
	/*ADCУ׼*/
	ADC_ResetCalibration(ADC1);								//�̶����̣��ڲ��е�·���Զ�ִ��У׼
	while (ADC_GetResetCalibrationStatus(ADC1) == SET);
	ADC_StartCalibration(ADC1);
	while (ADC_GetCalibrationStatus(ADC1) == SET);
	
	/*ADC����*/
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);	//�������ADC��ʼ����������ADC��������ת��ģʽ���ʴ���һ�κ�ADC�Ϳ���һֱ�������ϵع���
}
/************************************************************************************
 �� �� ��  : StorageDataCyclic
 ��������  : ���ݻ���
 �������  : ��
 �������  : void
************************************************************************************/
void StorageDataCyclic(BUFFER_TYPE *buffer, uint16_t data)
{
    *(buffer->array + buffer->bufferIndex) = data;
      buffer->bufferIndex++;	//ÿ�δ�һ��ֵ
    if (buffer->bufferIndex >= BUFFER_SIZE)
    {
        buffer->bufferIndex = 0;
    }
}
/************************************************************************************
 �� �� ��  : Filter_AverageCalc
 ��������  : ƽ��ֵ�˲�
 �������  : ��
 �������  : void
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
 �� �� ��  : BemfCheck
 ��������  : ���綯�ƹ������
 �������  : ��
 �������  : void
************************************************************************************/
#if SHF_TEST
void BemfCheck(void)  
{
	static uint16_t U_Array[100];
	static uint16_t V_Array[100];
	static uint16_t W_Array[100];
	static uint16_t u_idx;
	if(sysflags.Angle_Mask==0)//�ȴ���������
	{	
		Motor.UVoltage = ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_1);
		Motor.VVoltage = ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_2);
		Motor.WVoltage = ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_3);
		Motor.Uab = Motor.UVoltage - Motor.VVoltage;
		Motor.Ubc = Motor.VVoltage - Motor.WVoltage;
		Motor.Uca = Motor.WVoltage - Motor.UVoltage;
		if(Motor.PhaseCnt == 3 || Motor.PhaseCnt == 6)		//����U��
		{
			Motor.Ea = (Motor.Uab - Motor.Uca) / 2;
		}
		else if(Motor.PhaseCnt == 2 || Motor.PhaseCnt == 5)	//����V��
		{
			Motor.Eb = (Motor.Ubc - Motor.Uab) / 2;
		}
		else if(Motor.PhaseCnt == 1 || Motor.PhaseCnt == 4)	//����W��
		{
			Motor.Ec = (Motor.Uca - Motor.Ubc) / 2;
		}
	}
	if((sysflags.ChangePhase==0)&&(sysflags.Angle_Mask==0))
	{
		switch(Motor.PhaseCnt)			//��ʱ��һʧ���ˣ���λ���Ͳ�׼��
		{
			case 1:
				if(Motor.Ec < 0)   	//��ʱ���ⲻ��������ǿ�ƻ���
				{
					Sysvariable.BlankingCnt++;
					if(Sysvariable.BlankingCnt>=Filter_Count)	//���ȡ0����Ϊ���������ӳ���λ�ˣ����ӳپʹ���
					{
						Sysvariable.BlankingCnt=0;
						Motor.PhaseCnt = 2;      	//����		
						CalcSpeedTime();  	
					}									 
				}	 						
				break;
						 
			case 2:
				if(Motor.Eb > 0)		//Ϊʲô�Ǵ��ڣ�
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
						CalcSpeedTime();		//����һ�´˴��Ļ���ʱ�����Բ���
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
					W_Array[u_idx] = Motor.Ec;		//ʾ����ͣ������һ��
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
	if(sysflags.Angle_Mask==0)//�ȴ���������
	{	
		if(Motor.PhaseCnt == 3 || Motor.PhaseCnt == 6)		//����U��
		{
			Motor.Bemf = ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_1);
		}
		else if(Motor.PhaseCnt == 2 || Motor.PhaseCnt == 5)	//����V��
		{
			Motor.Bemf = ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_2);
		}
		else if(Motor.PhaseCnt == 1 || Motor.PhaseCnt == 4)	//����W��
		{
			Motor.Bemf = ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_3);
		}
		V_Bus_Half=(uint16_t)(RegularConvData_Tab[0]*0.5);	//ĸ�ߵ�ѹ��һ�� *Ku����
		U_Vol_run = ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_1);
		V_Vol_run = ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_2);
		W_Vol_run = ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_3);
		//u_idx ++;
	}
	if((sysflags.ChangePhase==0)&&(sysflags.Angle_Mask==0))
	{
		switch(Motor.PhaseCnt)			//��ʱ��һʧ���ˣ���λ���Ͳ�׼��
		{
#if SHF_TEST2
			case 1:
				if((Motor.Bemf<V_Bus_Half) || (TIM2->CNT >= Sysvariable.LastDragTime * 3))   	//��ʱ���ⲻ��������ǿ�ƻ���
				{
					Sysvariable.BlankingCnt++;
					if(Sysvariable.BlankingCnt>=Filter_Count)	//���ȡ0����Ϊ���������ӳ���λ�ˣ����ӳپʹ���
					{
						Sysvariable.BlankingCnt=0;
						Sysvariable.ADCTimeCnt = 0;
						Motor.PhaseCnt = 2;      	//����		
						CalcSpeedTime();  	
					}									 
				}	 						
				break;
						 
			case 2:
				if((Motor.Bemf>V_Bus_Half) || (TIM2->CNT >= Sysvariable.LastDragTime * 3))		//Ϊʲô�Ǵ��ڣ�
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
						CalcSpeedTime();		//����һ�´˴��Ļ���ʱ�����Բ���
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
					W_Array[u_idx] = W_Vol_run;		//ʾ����ͣ������һ��
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
				if(Motor.Bemf<V_Bus_Half)   	//��ʱ���ⲻ��������ǿ�ƻ���
				{
					Sysvariable.BlankingCnt++;
					if(Sysvariable.BlankingCnt>=Filter_Count)	//���ȡ0����Ϊ���������ӳ���λ�ˣ����ӳپʹ���
					{
						Sysvariable.BlankingCnt=0;
						Motor.PhaseCnt = 2;      	//����		
						CalcSpeedTime();  	
					}									 
				}	 						
				break;
						 
			case 2:
				if(Motor.Bemf>V_Bus_Half)		//Ϊʲô�Ǵ��ڣ�
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
						CalcSpeedTime();		//����һ�´˴��Ļ���ʱ�����Բ���
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
					W_Array[u_idx] = W_Vol_run;		//ʾ����ͣ������һ��
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
 �� �� ��  : JudgeErrorCommutation
 ��������  : �жϻ������/��ʱ�䲻����  
 �������  : ��
 �������  : void
*****************************************************************************/
void JudgeErrorCommutation()
{	
	if(Motor.LastPhase ==  Motor.PhaseCnt)
	{
		if(++Sysvariable.SamePhaseCnt >= 3200)	// 62.5us*n==200msû�з�������
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
 �� �� ��  : Fault_OverUnderVoltage
 ��������  : ��ѹ��Ƿѹ��������
 �������  : ��
 �������  : void
************************************************************************************/
void Fault_OverUnderVoltage(void)
{
	static	uint16_t  VoltageProCnt = 0;
	ADCSampPare.BUS_Voltage = RegularConvData_Tab[0]*3.3*72.7/4095/4.7;           //DCĸ�ߵ�ѹ		
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
 �� �� ��  : Cal_AverCurrent
 ��������  : ����ƽ����������  !!!!!���������Ĵ���
 �������  : ��
 �������  : void		//�˴�ֱ��ȡ����AD����ֵGlobal_Current
************************************************************************************/
void Cal_AverCurrent(void)
{
	static  uint32_t  ADTotalCurrent=0;
	static  int8_t Current_AverTimes=0;
	uint16_t	  Aver_Current=0;
	if(Filter_Mode==Filter_Average)
	{
		StorageDataCyclic(&ADCSampPare.busCur, Global_Current);
		Motor.Current = Filter_AverageCalc(ADCSampPare.busCur.array, BUFFER_SIZE)*27;//  ����ϵ��  0.82  22.3*0.82
	}
	else if(Filter_Mode==Filter_Function)
	{
		ADTotalCurrent+=Global_Current;
		Current_AverTimes++;
		if(Current_AverTimes>=Filter_Const1)//ƽ����������
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
 �� �� ��  : Cal_AverCurrent
 ��������  : ����ƽ���������� 62.5usִ��һ��
 �������  : ��
 �������  : void
************************************************************************************/

void  Instant_Current_Cale(void)
{
	if(ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_4)>mcCurOffset.I_Off_Average)
	{
		Global_Current=(ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_4)-mcCurOffset.I_Off_Average);//*CURRENT_COFF; ADֵ
	}
	else
	{
		//Global_Current=0;
	}
}
//û�õ�
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
		if(Soft_Time>1000)//1000*1ms=1s  ���Ƴ�ʱ�䳬��������
		{
			Soft_Time=0;
			Sysvariable.Stop_Time=Motor_DelayTime;			
			MotorStop();
			mcState=mcStop;
			mcFault = OverCurrent;
		}
}
/************************************************************************************
 �� �� ��  : PCB_OverTemperature
 ��������  : PCB���º���
 �������  : ��
 �������  : void
************************************************************************************/

void FaultMos_OverTemperature(void) 
{  
    static  uint8_t  OverTempcnt=0;	  
		ADCSampPare.PCB_Temp   =  RegularConvData_Tab[1]*3.3/4095;           //��·���¶�	
		if(ADCSampPare.PCB_Temp<=Max_PCB_Temp) //PCB�¶�
		{
			  OverTempcnt++;
			  if(OverTempcnt>=10)
				{
					OverTempcnt=0;
					Sysvariable.Stop_Time=Motor_DelayTime;			
				  MotorStop();           //����ֹͣ��� 
					mcState=mcStop;


				}		
		}
		else
		{
			OverTempcnt=0;
		}
		
}
/************************************************************************************
 �� �� ��  : Fault_Detection
 ��������  :���ϼ�⺯��
 �������  : ��
 �������  : void
************************************************************************************/
void Fault_Detection(void)
{

		Fault_Soft_Overcurrent();
		Fault_OverUnderVoltage();
		FaultMos_OverTemperature();
}
	

void Angle_Dealwith(void)
{
	if(Motor.Duty<DUTYTHRESHOLD1)//��duty
	{
		Mask_TIME =Low_DutyMask_Time ;  //��������ʱ��  time*4us
		Filter_Count=Delay_Filter;
	}
	else//��duty
	{
		Mask_TIME =High_DutyMask_Time; 
		Filter_Count=1;
	}
	
}
#endif
