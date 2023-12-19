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
  * @brief    定时器2初始化   定时100us
  * @param    无
  * @retval   无
  ***************************************************************/
  void TIM2_Config(void)
  {
    TIM_TimeBaseInitTypeDef TIM_BaseInitStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

  /*****
	计数器的时钟=Fclk/(Prescaler+1)
	PWM周期T= ( PWM_Arr+1  )*(Prescaler+1)/Fclk (HSI配置  Fclk=48MHZ)--得到的是   s
   ChannelxPulse = DutyCycle * (TIM1_Period - 1) / 100
	***/
  /* Time Base configuration 初始化定时器*/
    TIM_DeInit(TIM2);  
    TIM_BaseInitStructure.TIM_Period = 100 - 1;    //100 * 1us
    TIM_BaseInitStructure.TIM_Prescaler = 71;      //1Mhz  1us
    TIM_BaseInitStructure.TIM_ClockDivision = 0;     
    TIM_BaseInitStructure.TIM_CounterMode = TIM_CounterMode_Down; 
    TIM_TimeBaseInit(TIM2, &TIM_BaseInitStructure); 
      
    TIM_ClearFlag(TIM2, TIM_FLAG_Update);
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);          ////100us中断一次
      
    TIM_Cmd(TIM2, ENABLE);
 
 }
 /*****************************************************************
  * @file     TIM4_Config
  * @brief    定时器4初始化   定时1ms
  * @param    无
  * @retval   无
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
    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);         ///中断允许
    
    TIM_Cmd(TIM4, ENABLE);

    // /* Enable the TIM4 Interrupt        定时器4用来干什么的?   1ms定时用的      */
    // NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
    // NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    // NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    // NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    // NVIC_Init(&NVIC_InitStructure);

 }
/*****************************************************************
  * @file     TIM1_Config
  * @brief    定时器1PWM初始化  用于产生PWM
  * @param    无
  * @retval   无
  ***************************************************************/
 void TIM1_Config(void)
{
/* TIM1   GPIO Configuration ---------------------------------------------------*/
  GPIO_InitTypeDef   GPIO_InitStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO, ENABLE); 

    /* 配置定时器通道1输出引脚模式：复用推挽输出模式 */
  GPIO_InitStructure.GPIO_Pin =  BLDC_TIM_CH1_PIN | BLDC_TIM_CH2_PIN | BLDC_TIM_CH3_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  // 配置定时器互补通道1输出引脚模式   
  GPIO_InitStructure.GPIO_Pin =  BLDC_TIM_CH1N_PIN | BLDC_TIM_CH2N_PIN | BLDC_TIM_CH3N_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* Configuration: BKIN pin */
  GPIO_InitStructure.GPIO_Pin = BLDC_TIM_BKIN_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(BLDC_TIM_BKIN_PORT, &GPIO_InitStructure);
  
  /* TIM1 Configuration ---------------------------------------------------*/
  /* Time Base configuration 初始化定时器*/
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;
  TIM_BDTRInitTypeDef TIM_BDTRInitStructure;

	TIM_DeInit(TIM1);
  TIM_TimeBaseStructure.TIM_Period = PWM_ARR - 1;    //设置自动重装载值 24k  3000  对应62.5us   ARR
  TIM_TimeBaseStructure.TIM_Prescaler = 0;      //设置预分频   不分频
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;      //时钟分频系数
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up ;  //  向上计数时候被设置
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;   //计数溢出多少次才进中断  0表示溢出1次 进入1次  改动：1->0
  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);   //TIM1决定PWM周期

  /* Channel 1, 2,3  Configuration in PWM mode */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;   //PWM模式1:CNT<CCR 输出有效
  TIM_OCInitStructure.TIM_Pulse = 0;                  //PWM占空比
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //输出使能
  TIM_OCInitStructure.TIM_OutputNState = TIM_OutputState_Enable;//互补输出使能
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;     //设置初始极性  -高电平 
  TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCPolarity_Low;  //互补输出极性--高电平   下桥臂  高电平有效
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
  TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;
	
  TIM_OC1Init(TIM1, &TIM_OCInitStructure);  //使能通道1  初始化输出比较参数
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OC2Init(TIM1, &TIM_OCInitStructure);  //使能通道2
  TIM_OCInitStructure.TIM_Pulse = 0;  
  TIM_OC3Init(TIM1, &TIM_OCInitStructure);  //使能通道3

  TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);               
  TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
  TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);

  TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
  TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
  TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_OFF;        
  TIM_BDTRInitStructure.TIM_DeadTime = 20; 															//死区不到0.3us  1/72M * 20 = 13.89ns * 20

#if(MOTOR_BRAKE_ENABLE == 0)//关闭刹车功能        这个程序是启用了这个刹车功能的                    
  TIM_BDTRInitStructure.TIM_Break = TIM_Break_Disable;                                              
  TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_Low;                                
#else//开启刹车功能                                                                                   
  TIM_BDTRInitStructure.TIM_Break = TIM_Break_Enable;                                                
  TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_Low;        //##   刹车极性为高
#endif
  TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;    
  TIM_BDTRConfig(TIM1, &TIM_BDTRInitStructure); 

  TIM_ARRPreloadConfig(TIM1, ENABLE);  //使能APR预装载缓冲器  

  //刹车
  TIM_ClearITPendingBit(TIM1,TIM_IT_Break);
  TIM_ITConfig(TIM1,TIM_IT_Break,DISABLE);              ///刹车中断也中关了 
	TIM_Cmd(TIM1, ENABLE);  //使能定时器
  TIM_CtrlPWMOutputs(TIM1, ENABLE); 

  TIM_CCxCmd(TIM1,TIM_Channel_1,TIM_CCx_Disable);         ///CCxE这里先禁止掉，这是为了启动吧  CCE=0
  TIM_CCxNCmd(TIM1,TIM_Channel_1,TIM_CCxN_Disable);      ///CCNE  也没禁止掉，                即CCNE=0
  TIM_CCxCmd(TIM1,TIM_Channel_2,TIM_CCx_Disable);
  TIM_CCxNCmd(TIM1,TIM_Channel_2,TIM_CCxN_Disable);
  TIM_CCxCmd(TIM1,TIM_Channel_3,TIM_CCx_Disable);
  TIM_CCxNCmd(TIM1,TIM_Channel_3,TIM_CCxN_Disable);
}

/*****************************************************************
  * @file     Start_Motor
  * @brief    开启电机
  * @param    无
  * @retval   无
  ***************************************************************/
void Start_Motor(void)
{
	TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);
	TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Enable);

	TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);
	TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Enable);

	TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);
	TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Enable);
}
/*****************************************************************
  * @file     MOS_Q15PWM
  * @brief    U相上管通和V相的下管通  其他关闭
  * @param    无
  * @retval   无
  ******************************0*********************************/

#if 0
void MOS_Q15PWM(void)      
{   
	TIM1->CCR1 = Motor.Duty; //PA10输出pwm	 -U上	
	TIM1->CCR2 = 0;
	TIM1->CCR3 = 0;	  		  

	GPIOB->BSRR = U_Mos_L_Pin;
	GPIOB->BSRR = W_Mos_L_Pin;
	GPIOB->BRR = V_Mos_L_Pin;

	
}
 /*****************************************************************
  * @file     MOS_Q16PWM
  * @brief    U相上管通和W相的下管通  其他关闭
  * @param    无
  * @retval   无
  ***************************************************************/
void  MOS_Q16PWM(void)
{    

  TIM1->CCR1= Motor.Duty;   //PA10输出pwm	 -U上		
	TIM1->CCR2 = 0;
	TIM1->CCR3 = 0; 				  	

	GPIOB->BSRR =  U_Mos_L_Pin | V_Mos_L_Pin;
	GPIOB->BRR = W_Mos_L_Pin;
	
}
 /*****************************************************************
  * @file     MOS_Q26PWM
  * @brief    V相上管通和W相的下管通  其他关闭
  * @param    无
  * @retval   无
  ***************************************************************/
void MOS_Q26PWM(void)
{    
	
	TIM1->CCR1 = 0; 
	TIM1->CCR2 = Motor.Duty;	//PA9输出pwm -V上		
	TIM1->CCR3 = 0;	  
 
	GPIOB->BSRR =  U_Mos_L_Pin | V_Mos_L_Pin;
	GPIOB->BRR = W_Mos_L_Pin;

}
 /*****************************************************************
  * @file     MOS_Q24PWM
  * @brief    V相上管通和U相的下管通  其他关闭
  * @param    无
  * @retval   无
  ***************************************************************/
void MOS_Q24PWM(void) 
{    
	
	
	TIM1->CCR1 = 0;  
	TIM1->CCR2 = Motor.Duty;	 //PA9输出pwm -V上
  TIM1->CCR3 = 0;					  

	GPIOB->BSRR =   V_Mos_L_Pin;
	GPIOB->BSRR =   W_Mos_L_Pin;
	GPIOB->BRR = U_Mos_L_Pin;

}

/*****************************************************************
  * @file     MOS_Q34PWM
  * @brief    W相上管通和U相的下管通  其他关闭
  * @param    无
  * @retval   无
  ***************************************************************/
void MOS_Q34PWM(void)
{

	TIM1->CCR1 = 0;
	TIM1->CCR2 = 0;
	TIM1->CCR3 = Motor.Duty;		    //PA8输出pwm	-W上	  

	GPIOB->BSRR =   V_Mos_L_Pin;
	GPIOB->BSRR =   W_Mos_L_Pin;
  GPIOB->BRR = U_Mos_L_Pin;


	
}
 /*****************************************************************
  * @file     MOS_Q34PWM
  * @brief    W相上管通和V相的下管通  其他关闭
  * @param    无
  * @retval   无
  ***************************************************************/
void MOS_Q35PWM(void)
{  

  TIM1->CCR1 = 0; 
	TIM1->CCR2 = 0;
	TIM1->CCR3 = Motor.Duty;		        //W			  
  
	GPIOB->BSRR =   U_Mos_L_Pin;
	GPIOB->BSRR =   W_Mos_L_Pin;
	GPIOB->BRR = V_Mos_L_Pin;


}
#else
void MOS_Q15PWM(void)      //U相上管通和V相的下管通  其他关闭
{ 
  TIM1->CCR1 = Motor.Duty;  
	TIM1->CCR2 = 0;         					  
	TIM1->CCR3 = 0;
	GPIO_ResetBits(GPIOB, GPIO_Pin_13 | GPIO_Pin_15); 
	GPIO_SetBits(GPIOB, GPIO_Pin_14); 
}

void MOS_Q16PWM(void)    //U相上管通和W相的下管通  其他关闭
{ 
  TIM1->CCR1 = Motor.Duty;    
	TIM1->CCR2 = 0;            				  
	TIM1->CCR3 = 0;
	GPIO_ResetBits(GPIOB, GPIO_Pin_13 | GPIO_Pin_14); 
	GPIO_SetBits(GPIOB, GPIO_Pin_15);
}

void MOS_Q26PWM(void)     //V相上管通和W相的下管通  其他关闭
{    
	TIM1->CCR1= 0;       
	TIM1->CCR2 = Motor.Duty;					  
	TIM1->CCR3= 0;
	GPIO_ResetBits(GPIOB, GPIO_Pin_13 | GPIO_Pin_14); 
	GPIO_SetBits(GPIOB, GPIO_Pin_15); 
}

void MOS_Q24PWM(void)     //V相上管通和U相的下管通  其他关闭
{    
	TIM1->CCR1 = 0;        
	TIM1->CCR2 = Motor.Duty;					  
	TIM1->CCR3 = 0;
	GPIO_ResetBits(GPIOB, GPIO_Pin_14 | GPIO_Pin_15); 
	GPIO_SetBits(GPIOB, GPIO_Pin_13);
}

void MOS_Q34PWM(void)     //W相上管通和U相的下管通  其他关闭
{
  TIM1->CCR1 = 0;
	TIM1->CCR2 = 0;
	TIM1->CCR3 = Motor.Duty;					  
	GPIO_ResetBits(GPIOB, GPIO_Pin_14 | GPIO_Pin_15); 
	GPIO_SetBits(GPIOB, GPIO_Pin_13); 
}

void MOS_Q35PWM(void)     //W相上管通和V相的下管通  其他关闭
{  
  TIM1->CCR1 = 0;
	TIM1->CCR2 = 0;
	TIM1->CCR3 = Motor.Duty;	
  GPIO_ResetBits(GPIOB, GPIO_Pin_13 | GPIO_Pin_15); 
  GPIO_SetBits(GPIOB, GPIO_Pin_14);   
}

#endif

#endif
