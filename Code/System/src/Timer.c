#ifndef  _TIM_FILE
#define  _TIM_FILE

#include "SysConfig.h" 

#include "Timer.h"
#include "GPIO_Init.h"
#include "BLDC.h"
#include "Timer_ISR.h"
#include "User_Config.h"

const  uint16_t StopTime_Buffer[20]={0,100,200,300,400,500,600,700,800,900,1000,
	                                   1100 ,1200,1300,1400,1500,1600,1700,1800,1900
};
/*****************************************************************
  * @file     TIM2_Config
  * @brief    ��ʱ��2��ʼ��   ��ʱ60ms
  * @param    ��
  * @retval   ��
  ***************************************************************/
  void TIM2_Config(void)
  {
    TIM_TimeBaseInitTypeDef TIM_BaseInitStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

  /* Time Base configuration ��ʼ����ʱ��*/
    TIM_DeInit(TIM2);  
    TIM_BaseInitStructure.TIM_Period = 60000 - 1;    //100 * 1us
    TIM_BaseInitStructure.TIM_Prescaler = 71;      //1Mhz  1us
    TIM_BaseInitStructure.TIM_ClockDivision = 0;     
    TIM_BaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up; 
    TIM_TimeBaseInit(TIM2, &TIM_BaseInitStructure); 
      
    TIM_ClearFlag(TIM2, TIM_FLAG_Update);
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);          ////100us�ж�һ��
      
    TIM_Cmd(TIM2, ENABLE);
 
 }
/*****************************************************************
* @file     TIM3_Config
* @brief    ��ʱ��3��ʼ��   ��ʱ100us
* @param    ��
* @retval   ��
***************************************************************/
void TIM3_Config(void)
{
  TIM_TimeBaseInitTypeDef TIM_BaseInitStructure;
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    
  TIM_BaseInitStructure.TIM_Period = 100-1;           
  TIM_BaseInitStructure.TIM_Prescaler = 71;//
  TIM_BaseInitStructure.TIM_ClockDivision = 0;     
  TIM_BaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up; 
  TIM_TimeBaseInit(TIM3, &TIM_BaseInitStructure); 

  TIM_ClearFlag(TIM3, TIM_FLAG_Update);
  TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);         ///�ж�����

  TIM_Cmd(TIM3, ENABLE);

}
 /*****************************************************************
  * @file     TIM4_Config
  * @brief    ��ʱ��4��ʼ��   ��ʱ1ms
  * @param    ��
  * @retval   ��
  ***************************************************************/
 void TIM4_Config(void)
 {
    TIM_TimeBaseInitTypeDef TIM_BaseInitStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
     
    TIM_BaseInitStructure.TIM_Period = 1000-1;           
    TIM_BaseInitStructure.TIM_Prescaler = 71;//
    TIM_BaseInitStructure.TIM_ClockDivision = 0;     
    TIM_BaseInitStructure.TIM_CounterMode = TIM_CounterMode_Down; 
    TIM_TimeBaseInit(TIM4, &TIM_BaseInitStructure); 
    
    TIM_ClearFlag(TIM4, TIM_FLAG_Update);
    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);         ///�ж�����
    
    TIM_Cmd(TIM4, ENABLE);

 }
/*****************************************************************
  * @file     TIM1_Config
  * @brief    ��ʱ��1PWM��ʼ��  ���ڲ���PWM
  * @param    ��
  * @retval   ��
  ***************************************************************/
 void TIM1_Config(void)
{
/* TIM1   GPIO Configuration ---------------------------------------------------*/
  GPIO_InitTypeDef   GPIO_InitStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO, ENABLE); 

    /* ���ö�ʱ��ͨ��1�������ģʽ�������������ģʽ */
  GPIO_InitStructure.GPIO_Pin =  BLDC_TIM_CH1_PIN | BLDC_TIM_CH2_PIN | BLDC_TIM_CH3_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  // ���ö�ʱ������ͨ��1�������ģʽ   
  GPIO_InitStructure.GPIO_Pin =  BLDC_TIM_CH1N_PIN | BLDC_TIM_CH2N_PIN | BLDC_TIM_CH3N_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* Configuration: BKIN pin */
  GPIO_InitStructure.GPIO_Pin = BLDC_TIM_BKIN_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(BLDC_TIM_BKIN_PORT, &GPIO_InitStructure);
  
  /* TIM1 Configuration ---------------------------------------------------*/
  /* Time Base configuration ��ʼ����ʱ��*/
#if 0
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;
  TIM_BDTRInitTypeDef TIM_BDTRInitStructure;

	//TIM_DeInit(TIM1);
  TIM_TimeBaseStructure.TIM_Period = PWM_ARR - 1;    //�����Զ���װ��ֵ 24k  3000  ��Ӧ62.5us   ARR
  TIM_TimeBaseStructure.TIM_Prescaler = 0;      //����Ԥ��Ƶ   ����Ƶ
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;      //ʱ�ӷ�Ƶϵ��
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up ;  //  ���ϼ���ʱ������
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;   //����������ٴβŽ��ж�  0��ʾ���1�� ����1��  �Ķ���1->0
  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);   //TIM1����PWM����

  /* Channel 1, 2,3  Configuration in PWM mode */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;   //PWMģʽ1:CNT<CCR �����Ч
  TIM_OCInitStructure.TIM_Pulse = 0;                  //PWMռ�ձ�
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //���ʹ��
  TIM_OCInitStructure.TIM_OutputNState = TIM_OutputState_Enable;//�������ʹ��
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;     //���ó�ʼ����  -�ߵ�ƽ 
  TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCPolarity_Low;  //�����������--�ߵ�ƽ   ���ű�  �ߵ�ƽ��Ч
  // TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCPolarity_High;
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
  TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Set;
  // TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;
	
  TIM_OC1Init(TIM1, &TIM_OCInitStructure);  //ʹ��ͨ��1  ��ʼ������Ƚϲ���
  TIM_OCInitStructure.TIM_Pulse = 0;        //���ֵ��ռ�ձ�
  TIM_OC2Init(TIM1, &TIM_OCInitStructure);  //ʹ��ͨ��2
  TIM_OCInitStructure.TIM_Pulse = 0;  
  TIM_OC3Init(TIM1, &TIM_OCInitStructure);  //ʹ��ͨ��3

    // �趨TIM1_CH4�Ĺ���״̬ Ϊ�Ƚ����ģʽ,����ADת����ͨ��4�����GPIO������� �����������Ҫ��İ� ��ΪAD������������ ADC_ExternalTrigInjecConv_T1_CC4
  /* Channel 4 Configuration in OC */                               ///PA11ȷʵ�����յ�û�õ����TIM1_CH4   
                
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;  
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
  TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;                  
  TIM_OCInitStructure.TIM_Pulse = PWM_ARR - 1;          ///��������ڽ�����ʱ��  ���CCֵ��ĺ��Ǹ�����ֵ����һ����  ����˼

  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 
  TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;        ///�������Nû�õ���  
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
  TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;   ///�������Nû�õ���
  TIM_OC4Init(BLDC_TIMx, &TIM_OCInitStructure);

  TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
  TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
  TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_OFF;        
  TIM_BDTRInitStructure.TIM_DeadTime = 20; 															//��������0.3us  1/72M * 20 = 13.89ns * 20

#if(MOTOR_BRAKE_ENABLE == 0)//�ر�ɲ������        ������������������ɲ�����ܵ�                    
  TIM_BDTRInitStructure.TIM_Break = TIM_Break_Disable;                                              
  TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_High;                                
#else//����ɲ������                                                                                   
  TIM_BDTRInitStructure.TIM_Break = TIM_Break_Enable;                                                
  TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_High;        //##   ɲ������Ϊ��
#endif
  TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;    
  TIM_BDTRConfig(TIM1, &TIM_BDTRInitStructure); 

  TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);               
  TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
  TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
  // �趨TIM1_CH4�Ĺ���״̬ Ϊ�Ƚ����ģʽ,����ADת����
  TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);

  TIM_ARRPreloadConfig(TIM1, ENABLE);  //ʹ��APRԤװ�ػ�����  

  //ɲ��
  TIM_ClearITPendingBit(TIM1,TIM_IT_Break);
  TIM_ITConfig(TIM1,TIM_IT_CC4,ENABLE);            //��ʱ���жϴ򿪣����ڴ���ADת��
  TIM_ITConfig(TIM1,TIM_IT_Break,DISABLE);              ///ɲ���жϹ��� 
	TIM_Cmd(TIM1, ENABLE);  //ʹ�ܶ�ʱ��
  TIM_CtrlPWMOutputs(TIM1, ENABLE); 

  TIM_CCxCmd(TIM1,TIM_Channel_1,TIM_CCx_Disable);         ///CCxE�����Ƚ�ֹ��������Ϊ��������  CCE=0
  TIM_CCxNCmd(TIM1,TIM_Channel_1,TIM_CCxN_Disable);      ///CCNE  Ҳû��ֹ����                ��CCNE=0
  TIM_CCxCmd(TIM1,TIM_Channel_2,TIM_CCx_Disable);
  TIM_CCxNCmd(TIM1,TIM_Channel_2,TIM_CCxN_Disable);
  TIM_CCxCmd(TIM1,TIM_Channel_3,TIM_CCx_Disable);
  TIM_CCxNCmd(TIM1,TIM_Channel_3,TIM_CCxN_Disable);
  TIM_CCxCmd(TIM1,TIM_Channel_4,TIM_CCx_Disable);
  TIM_CCxNCmd(TIM1,TIM_Channel_4,TIM_CCxN_Disable);
#else
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;
    TIM_BDTRInitTypeDef TIM_BDTRInitStructure;
    
     
    /* ��ʱ����������ʼ�� */		 
    TIM_TimeBaseStructure.TIM_Period = PWM_ARR-1;            ////���ֵ��ʵ��ֵΪ3600-1           ��������20KƵ��  ���ʱ����50us
    /* ����Ԥ��Ƶ����Ԥ��Ƶ����Ϊ72MHz */
    TIM_TimeBaseStructure.TIM_Prescaler = BLDC_TIM_PRESCALER;        ///����Ƶ
    /* ����ʱ�ӷ�Ƶϵ��������Ƶ(�����ò���) */
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1 ;        ///����Ƶ
    /* ���ϼ���ģʽ */
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;     ///��������ϼ���    �����Ǹ�HALL�ɲ���  
    /* �ظ������� */
    TIM_TimeBaseStructure.TIM_RepetitionCounter = BLDC_TIM_REPETITIONCOUNTER;    ////�������0 û�õ�
    TIM_TimeBaseInit(BLDC_TIMx, &TIM_TimeBaseStructure);
  
    /* ��ʱ�����ͨ��1ģʽ���� */
    /* ģʽ���ã�PWMģʽ1 */
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;              ////TIMx_CNT<TIMx_CCR1ʱͨ��1Ϊ��Ч��ƽ
    /* ���״̬���ã�ʹ����� */
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	
    /* ����ͨ�����״̬���ã�ʹ����� */
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable; ///���������ʹ����

    /* ��������ֵ�������������������ֵʱ����ƽ��������    ����Ϊʲô����Ϊ0  ����Ϊ0 ������Ч��ƽ  */
    TIM_OCInitStructure.TIM_Pulse = 0;      ///���ֵ����0�Ļ�������ζ�������Զ�������Ǹ���Ч��ƽ��
    /* ����ʱ������ֵС��CCR1_ValʱΪ�ߵ�ƽ */
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;          ///����������Ǹߵ�ƽ��Ч��������������ǵ͵�ƽ��Ч ���Ե�ʱ����ʱ��һ�����һ����ʹӶ������·
    TIM_OCInitStructure.TIM_OCNPolarity= TIM_OCNPolarity_Low;         //// �м����ֵ=0008  Ҳ���� CCxNP=1  �ʼ�ס����������ֵ��ɵ͵�ƽ��Ч
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;     ///�����������ʲô����  ����ʱ�����Ǹ�λ�� Ҳ���ǵ͵�ƽ    0����MOE=0ʱ�����ʵ����OC1N����������OC1=0��
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Set;//0x0200    /// ����ʱ ���NΪ�ߵ�ƽ ��������MOE�ֵ���0 ����������ʼ�����״̬   //  1����MOE=0ʱ�����ʵ����OC1N����������OC1=1
    /* ��ʼ����ʱ��ͨ��1���PWM */                              /// 1����MOE=0ʱ��������OC1N=1�� Ҳ����˵����MOE=0ʱ�����N������Ǹߵ�ƽ��Ҳ������Ч��ƽ  �����������Ӷ�����ͨ��
    TIM_OC1Init(BLDC_TIMx, &TIM_OCInitStructure);                   //// һ����CCNP��Ϊ�͵�ƽ��Ч��Ҳ���ǽ����CCxNP=1 ��ô��CC1E��CC1NE=0�������:
                                                                     ////                                                    OCx=CCxP=0   OCxN=CCxNP=1   �ʴ�ʱ���Ϲܹرգ����¹�Ҳ�ǹر��ˡ�
    /* ��ʱ�����ͨ��2ģʽ���� */                                    ////�����ڳ�ʼ��������£��������ȫ���ǹرյģ�������������֮�����ֵ�͵��� OCxREF + ���� + ������OCxREF���� + ���� + ������
    /* ����ͨ��2�ĵ�ƽ����ֵ���������һ��ռ�ձȵ�PWM */            ////��������ʱ����� �ڻ���ʱ������6�����ȫ��������ġ�Ψ�����е�����������������4������DISABLE��
    TIM_OCInitStructure.TIM_Pulse = 0;                                              ////��Ҳ����˵��������4�����������Ϊ����ͨ��״̬�ġ�ֻ��������ͨ������һ����һֱͨ����һ���ǰ�ռ�ձ�ͨ��
    /* ��ʼ����ʱ��ͨ��2���PWM */                                                  ///
    TIM_OC2Init(BLDC_TIMx, &TIM_OCInitStructure);
  
    /* ��ʱ�����ͨ��3ģʽ���� */
    /* ����ͨ��3�ĵ�ƽ����ֵ���������һ��ռ�ձȵ�PWM */
    TIM_OCInitStructure.TIM_Pulse = 0;
    /* ��ʼ����ʱ��ͨ��3���PWM */
    TIM_OC3Init(BLDC_TIMx, &TIM_OCInitStructure);
    
    // �趨TIM1_CH4�Ĺ���״̬ Ϊ�Ƚ����ģʽ,����ADת����ͨ��4�����GPIO������� �����������Ҫ��İ� ��ΪAD������������ ADC_ExternalTrigInjecConv_T1_CC4
  /* Channel 4 Configuration in OC */                               ///PA11ȷʵ�����յ�û�õ����TIM1_CH4   
                
   TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;  
   TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
   TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;                  
   TIM_OCInitStructure.TIM_Pulse = PWM_ARR - 1;          ///��������ڽ�����ʱ��  ���CCֵ��ĺ��Ǹ�����ֵ����һ����  ����˼
  
   TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 
   TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;        ///�������Nû�õ���  
   TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
   TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;   ///�������Nû�õ���
   TIM_OC4Init(BLDC_TIMx, &TIM_OCInitStructure);
      // �趨TIM1_CH4�Ĺ���״̬ Ϊ�Ƚ����ģʽ,����ADת����
 
   
 ////######################################################################################################   
    /* Automatic Output enable, Break, dead time and lock configuration*/                              //##
    TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;                                        //##   OSSR=1 
    TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;                                        //##   OSSI=1 
    TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_OFF;                                           //## 
    TIM_BDTRInitStructure.TIM_DeadTime = 20;//1/72M = 13.89ns                                          //##   ���� 0.3us
                                                                                                       //##
#if(MOTOR_BRAKE_ENABLE == 0)//�ر�ɲ������        ������������������ɲ�����ܵ�                       //##
    TIM_BDTRInitStructure.TIM_Break = TIM_Break_Disable;                                               //## 
    TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_High;                                  //##
#else//����ɲ������                                                                                    //##
    TIM_BDTRInitStructure.TIM_Break = TIM_Break_Enable;                                                //##
    TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_High;                                  //##   ɲ������Ϊ��
#endif                                                                                                 //##
                                                                                                       //##
    TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;                            //##     1��MOE�ܱ�����á�1��������һ�������¼����Զ��á�1��(���ɲ��������Ч)
    TIM_BDTRConfig(BLDC_TIMx, &TIM_BDTRInitStructure);                                                 //##
 ////######################################################################################################   
    TIM_OC1PreloadConfig(BLDC_TIMx,TIM_OCPreload_Enable);           ////�������CCֵ��Ԥװ�ع��� �����¼�����ʱ���Ÿ������CC��ֵ 
    TIM_OC2PreloadConfig(BLDC_TIMx,TIM_OCPreload_Enable);          
    TIM_OC3PreloadConfig(BLDC_TIMx,TIM_OCPreload_Enable);
    
    // �趨TIM1_CH4�Ĺ���״̬ Ϊ�Ƚ����ģʽ,����ADת����
    TIM_OC4PreloadConfig(BLDC_TIMx,TIM_OCPreload_Enable);
   /*�������ADC������ʼ��*/
   // MotorCurrentAdcChannel_Init();

    /* ʹ�ܶ�ʱ�����ؼĴ���ARR */
    TIM_ARRPreloadConfig(BLDC_TIMx, ENABLE);
  
    TIM_ITConfig(BLDC_TIMx,TIM_IT_CC4,DISABLE);               
    
    //ɲ��
    TIM_ClearITPendingBit(BLDC_TIMx,TIM_IT_Break);
    TIM_ITConfig(BLDC_TIMx,TIM_IT_Break,DISABLE);              ///ɲ���ж�Ҳ�й���
    /* ʹ�ܶ�ʱ�� */
    TIM_Cmd(BLDC_TIMx, ENABLE);
    
    /* TIM�����ʹ�� */
    TIM_CtrlPWMOutputs(BLDC_TIMx, ENABLE);                ////�������MOE =1   ע�⵽��ʼ��֮�����ֵ����1 ������Ҳû�и�λ���� ����һ�еĵ�ƽ���㣬������MOE=1Ϊ��׼
    
    TIM_CCxCmd(BLDC_TIMx,TIM_Channel_1,TIM_CCx_Disable);         ///CCxE�����Ƚ�ֹ��������Ϊ��������  CCE=0
    TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_1,TIM_CCxN_Disable);      ///CCNE  Ҳû��ֹ����                ��CCNE=0
    TIM_CCxCmd(BLDC_TIMx,TIM_Channel_2,TIM_CCx_Disable);
    TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_2,TIM_CCxN_Disable);
    TIM_CCxCmd(BLDC_TIMx,TIM_Channel_3,TIM_CCx_Disable);
    TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_3,TIM_CCxN_Disable);
    TIM_CCxCmd(BLDC_TIMx,TIM_Channel_4,TIM_CCx_Disable);   //����ֹ�ˣ��ʴ�ʱ6���ܽŵ����ֵȡ����   MOE=1��OSSI=1��OSSR=1��OIS1�޹���ΪMOE=1�ˡ�OIS1N�޹�   ��CC1E=1 CC1NE=1 λ��ֵ
    TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_4,TIM_CCxN_Disable);  
#endif
}

/*****************************************************************
  * @file     Start_Motor
  * @brief    �������
  * @param    ��
  * @retval   ��
  ***************************************************************/
void Start_Motor(void)
{
	TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);
	TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Enable);

	TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);
	TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Enable);

	TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);
	TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Enable);

  TIM_CCxCmd(TIM1, TIM_Channel_4, TIM_CCx_Enable);
  //TIM_CCxNCmd(TIM1, TIM_Channel_4, TIM_CCxN_Enable);
}
/*****************************************************************
  * @file     MOS_Q15PWM
  * @brief    U���Ϲ�ͨ��V����¹�ͨ  �����ر�
  * @param    ��
  * @retval   ��
  ******************************0*********************************/

#if 1
void MOS_Q15PWM(void)      
{   
    TIM_CCxCmd(BLDC_TIMx,TIM_Channel_3,TIM_CCx_Disable);
    TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_3,TIM_CCxN_Disable);    
    /*  Channel1 configuration */
    TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_1,TIM_CCxN_Disable);
    TIM_SetCompare1(BLDC_TIMx,Motor.Duty);
    TIM_CCxCmd(BLDC_TIMx,TIM_Channel_1,TIM_CCx_Enable);
    /*  Channel2 configuration */
    TIM_CCxCmd(BLDC_TIMx,TIM_Channel_2,TIM_CCx_Disable);        
    TIM_SetCompare2(BLDC_TIMx,PWM_ARR);
    TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_2,TIM_CCxN_Enable);
}
 /*****************************************************************
  * @file     MOS_Q16PWM
  * @brief    U���Ϲ�ͨ��W����¹�ͨ  �����ر�
  * @param    ��
  * @retval   ��
  ***************************************************************/
void  MOS_Q16PWM(void)
{   
    TIM_CCxCmd(BLDC_TIMx,TIM_Channel_2,TIM_CCx_Disable);
    TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_2,TIM_CCxN_Disable);

    /*  Channel1 configuration */
    TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_1,TIM_CCxN_Disable);
    TIM_SetCompare1(BLDC_TIMx,Motor.Duty);
    TIM_CCxCmd(BLDC_TIMx,TIM_Channel_1,TIM_CCx_Enable);
    /*  Channel2 configuration */      
    /*  Channel3 configuration */
    TIM_CCxCmd(BLDC_TIMx,TIM_Channel_3,TIM_CCx_Disable);
    TIM_SetCompare3(BLDC_TIMx,PWM_ARR);
    TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_3,TIM_CCxN_Enable);
}
 /*****************************************************************
  * @file     MOS_Q26PWM
  * @brief    V���Ϲ�ͨ��W����¹�ͨ  �����ر�
  * @param    ��
  * @retval   ��
  ***************************************************************/
void MOS_Q26PWM(void)
{    
    TIM_CCxCmd(BLDC_TIMx,TIM_Channel_1,TIM_CCx_Disable);
    TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_1,TIM_CCxN_Disable);        ////��ʱ��1��ͨ��1������

    /*  Channel1 configuration */
    /*  Channel2 configuration */  
    TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_2,TIM_CCxN_Disable);           ///�Ȱ�ͨ��2�ص�

    TIM_SetCompare2(BLDC_TIMx,Motor.Duty);     ///�ٰ�����Ƚ�������һ��   �������ֵ������ô������   ����Ƚ������ֵ�� ǧ��֮speed_dutyռ�ձ�
    TIM_CCxCmd(BLDC_TIMx,TIM_Channel_2,TIM_CCx_Enable);           ///  �ٰ����ͨ��2 ����
    /*  Channel3 configuration */
    TIM_CCxCmd(BLDC_TIMx,TIM_Channel_3,TIM_CCx_Disable);          
    TIM_SetCompare3(BLDC_TIMx,PWM_ARR);                  ////�������ͨ��3N �ıȽ�ֵ����Ǿ�������  20K�Ǹ���50us
    TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_3,TIM_CCxN_Enable);
}
 /*****************************************************************
  * @file     MOS_Q24PWM
  * @brief    V���Ϲ�ͨ��U����¹�ͨ  �����ر�
  * @param    ��
  * @retval   ��
  ***************************************************************/
void MOS_Q24PWM(void) 
{    
    TIM_CCxCmd(BLDC_TIMx,TIM_Channel_3,TIM_CCx_Disable);
    TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_3,TIM_CCxN_Disable);

    /*  Channel1 configuration */
    TIM_CCxCmd(BLDC_TIMx,TIM_Channel_1,TIM_CCx_Disable);
    TIM_SetCompare1(BLDC_TIMx,PWM_ARR);                 ///��������B+�ĵط�����ľ���ռ�ձ�*���ڣ���A-�ĵط�����ľ�������
    TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_1,TIM_CCxN_Enable);

    /*  Channel2 configuration */
    TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_2,TIM_CCxN_Disable);
    TIM_SetCompare2(BLDC_TIMx,Motor.Duty);
    TIM_CCxCmd(BLDC_TIMx,TIM_Channel_2,TIM_CCx_Enable);
}

/*****************************************************************
  * @file     MOS_Q34PWM
  * @brief    W���Ϲ�ͨ��U����¹�ͨ  �����ر�
  * @param    ��
  * @retval   ��
  ***************************************************************/
void MOS_Q34PWM(void)
{
    TIM_CCxCmd(BLDC_TIMx,TIM_Channel_2,TIM_CCx_Disable);
    TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_2,TIM_CCxN_Disable);

    /*  Channel1 configuration */
    TIM_CCxCmd(BLDC_TIMx,TIM_Channel_1,TIM_CCx_Disable);
    TIM_SetCompare1(BLDC_TIMx,PWM_ARR);
    TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_1,TIM_CCxN_Enable);

    /*  Channel2 configuration */ 
    /*  Channel3 configuration */
    TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_3,TIM_CCxN_Disable);
    TIM_SetCompare3(BLDC_TIMx,Motor.Duty);
    TIM_CCxCmd(BLDC_TIMx,TIM_Channel_3,TIM_CCx_Enable);
}
 /*****************************************************************
  * @file     MOS_Q34PWM
  * @brief    W���Ϲ�ͨ��V����¹�ͨ  �����ر�
  * @param    ��
  * @retval   ��
  ***************************************************************/
void MOS_Q35PWM(void)
{  
    TIM_CCxCmd(BLDC_TIMx,TIM_Channel_1,TIM_CCx_Disable);
    TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_1,TIM_CCxN_Disable);    
    /*  Channel1 configuration */      
    /*  Channel2 configuration */  

    TIM_CCxCmd(BLDC_TIMx,TIM_Channel_2,TIM_CCx_Disable);
    TIM_SetCompare2(BLDC_TIMx,PWM_ARR);
    TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_2,TIM_CCxN_Enable);

    /*  Channel3 configuration */          
    TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_3,TIM_CCxN_Disable);
    TIM_SetCompare3(BLDC_TIMx,Motor.Duty);
    TIM_CCxCmd(BLDC_TIMx,TIM_Channel_3,TIM_CCx_Enable);
}
#else
void MOS_Q15PWM(void)      //U���Ϲ�ͨ��V����¹�ͨ  �����ر�
{ 
  TIM1->CCR1 = Motor.Duty;  
	TIM1->CCR2 = 0;         					  
	TIM1->CCR3 = 0;
  TIM1->CCR4 = Motor.Duty / 2;  
	GPIO_ResetBits(GPIOB, GPIO_Pin_13 | GPIO_Pin_15); 
	GPIO_SetBits(GPIOB, GPIO_Pin_14);   //reset�رգ�set��ͨ
}

void MOS_Q16PWM(void)    //U���Ϲ�ͨ��W����¹�ͨ  �����ر�
{ 
  TIM1->CCR1 = Motor.Duty;    
	TIM1->CCR2 = 0;            				  
	TIM1->CCR3 = 0;
  TIM1->CCR4 = Motor.Duty / 2; 
	GPIO_ResetBits(GPIOB, GPIO_Pin_13 | GPIO_Pin_14); 
	GPIO_SetBits(GPIOB, GPIO_Pin_15);
}

void MOS_Q26PWM(void)     //V���Ϲ�ͨ��W����¹�ͨ  �����ر�
{    
	TIM1->CCR1= 0;       
	TIM1->CCR2 = Motor.Duty;					  
	TIM1->CCR3= 0;
  TIM1->CCR4 = Motor.Duty / 2; 
	GPIO_ResetBits(GPIOB, GPIO_Pin_13 | GPIO_Pin_14); 
	GPIO_SetBits(GPIOB, GPIO_Pin_15); 
}

void MOS_Q24PWM(void)     //V���Ϲ�ͨ��U����¹�ͨ  �����ر�
{    
	TIM1->CCR1 = 0;        
	TIM1->CCR2 = Motor.Duty;					  
	TIM1->CCR3 = 0;
  TIM1->CCR4 = Motor.Duty / 2; 
	GPIO_ResetBits(GPIOB, GPIO_Pin_14 | GPIO_Pin_15); 
	GPIO_SetBits(GPIOB, GPIO_Pin_13);
}

void MOS_Q34PWM(void)     //W���Ϲ�ͨ��U����¹�ͨ  �����ر�
{
  TIM1->CCR1 = 0;
	TIM1->CCR2 = 0;
	TIM1->CCR3 = Motor.Duty;	
  TIM1->CCR4 = Motor.Duty / 2; 				  
	GPIO_ResetBits(GPIOB, GPIO_Pin_14 | GPIO_Pin_15); 
	GPIO_SetBits(GPIOB, GPIO_Pin_13); 
}

void MOS_Q35PWM(void)     //W���Ϲ�ͨ��V����¹�ͨ  �����ر�
{  
  TIM1->CCR1 = 0;
	TIM1->CCR2 = 0;
	TIM1->CCR3 = Motor.Duty;
  TIM1->CCR4 = Motor.Duty / 2; 	
  GPIO_ResetBits(GPIOB, GPIO_Pin_13 | GPIO_Pin_15); 
  GPIO_SetBits(GPIOB, GPIO_Pin_14);   
}

#endif

#endif
