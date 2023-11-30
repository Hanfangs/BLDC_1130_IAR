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
   NVIC_InitTypeDef   NVIC_InitStructure;  //

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

  /*****
	计数器的时钟=Fclk/(Prescaler+1)
	PWM周期T= ( PWM_Arr+1  )*(Prescaler+1)/Fclk (HSI配置  Fclk=48MHZ)--得到的是   s
   ChannelxPulse = DutyCycle * (TIM1_Period - 1) / 100
	***/
  /* Time Base configuration 初始化定时器*/
  TIM_DeInit(TIM2);  
  TIM_BaseInitStructure.TIM_Period = 100 - 1;           
  TIM_BaseInitStructure.TIM_Prescaler = 71;      
  TIM_BaseInitStructure.TIM_ClockDivision = 0;     
  TIM_BaseInitStructure.TIM_CounterMode = TIM_CounterMode_Down; 
  TIM_TimeBaseInit(TIM2, &TIM_BaseInitStructure); 
    
  TIM_ClearFlag(TIM2, TIM_FLAG_Update);
  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);          ////100us中断一次
    
  TIM_Cmd(TIM2, ENABLE);

    /* Enable the TIM2 Interrupt    用来100us定时用的  */
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;                 //配置中断源
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;       //配置抢占优先级
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;              //配置子优先级
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                 //使能中断通道
  NVIC_Init(&NVIC_InitStructure);                                 //调用初始化函数
 
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
    NVIC_InitTypeDef NVIC_InitStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
     
    TIM_BaseInitStructure.TIM_Period = 1000-1;           
    TIM_BaseInitStructure.TIM_Prescaler = 71;//
    TIM_BaseInitStructure.TIM_ClockDivision = 0;     
    TIM_BaseInitStructure.TIM_CounterMode = TIM_CounterMode_Down; 
    TIM_TimeBaseInit(TIM4, &TIM_BaseInitStructure); 
    
    TIM_ClearFlag(TIM4, TIM_FLAG_Update);
    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);         ///中断允许
    
    TIM_Cmd(TIM4, ENABLE);

    /* Enable the TIM4 Interrupt        定时器4用来干什么的?   1ms定时用的      */
    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

 }
/*****************************************************************
  * @file     TIM1_Config
  * @brief    定时器1PWM初始化  用于产生PWM
  * @param    无
  * @retval   无
  ***************************************************************/
 void TIM1_Config(void)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;
  GPIO_InitTypeDef   GPIO_InitStructure;
 // NVIC_InitTypeDef   NVIC_InitStructure;  //
  /********************************
	U:H -PA10-CH3     L -PB1-CH3N
  V:H-PA9-CH2       L-PB0-CH2N
  W:H-PA8-CH1       L-PA7-CH1N
	*************************************/
	/* GPIO Configuration ---------------------------------------------------
    GPIOA, Clocks enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO, ENABLE); 

    /* 配置定时器通道1输出引脚模式：复用推挽输出模式 */
  GPIO_InitStructure.GPIO_Pin =  BLDC_TIM_CH1_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(BLDC_TIM_CH1_PORT, &GPIO_InitStructure);
  
    /* 配置定时器通道2输出引脚模式 */
  GPIO_InitStructure.GPIO_Pin =  BLDC_TIM_CH2_PIN;
  GPIO_Init(BLDC_TIM_CH2_PORT, &GPIO_InitStructure);
  
    /* 配置定时器通道3输出引脚模式 */
  GPIO_InitStructure.GPIO_Pin =  BLDC_TIM_CH3_PIN;
  GPIO_Init(BLDC_TIM_CH3_PORT, &GPIO_InitStructure);

    // 配置定时器互补通道1输出引脚模式     
  GPIO_InitStructure.GPIO_Pin =  BLDC_TIM_CH1N_PIN;
  GPIO_Init(BLDC_TIM_CH1N_PORT, &GPIO_InitStructure); 
     
    // 配置定时器互补通道2输出引脚模式 
  GPIO_InitStructure.GPIO_Pin =  BLDC_TIM_CH2N_PIN;
  GPIO_Init(BLDC_TIM_CH2N_PORT, &GPIO_InitStructure); 
     
    // 配置定时器互补通道3输出引脚模式 
  GPIO_InitStructure.GPIO_Pin =  BLDC_TIM_CH3N_PIN;
  GPIO_Init(BLDC_TIM_CH3N_PORT, &GPIO_InitStructure);    
    
    /* Configuration: BKIN pin */
  GPIO_InitStructure.GPIO_Pin = BLDC_TIM_BKIN_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(BLDC_TIM_BKIN_PORT, &GPIO_InitStructure); 
  GPIO_SetBits(GPIOB, U_Mos_L_Pin);
  GPIO_SetBits(GPIOA, W_Mos_L_Pin);
  GPIO_SetBits(GPIOB, V_Mos_L_Pin); 
  
  /* TIM1 Configuration ---------------------------------------------------*/
  /* TIM1 clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 , ENABLE);
  /*****
	计数器的时钟=Fclk/(Prescaler+1)
	PWM周期T=  (PWM_Arr+1 )*(Prescaler+1)/Fclk (HSI配置  Fclk=48MHZ)---单位是s
   ChannelxPulse = DutyCycle * (TIM1_Period - 1) / 100
	***/
  /* Time Base configuration 初始化定时器*/
	TIM_DeInit(TIM1);
  TIM_TimeBaseStructure.TIM_Prescaler =0 ;      //设置预分频   sys/48000
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned1 ;  //  向上计数时候被设置
  TIM_TimeBaseStructure.TIM_Period =(SystemCoreClock/Cent_PWMFRE_16K)-1;    //设置自动重装载值 16k  1500  --16k 对应62.5us 
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;      //时钟分频系数
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 1;   //计数溢出多少次才进中断  0表示溢出1次 进入1次  
  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);   //TIM1决定PWM周期
	//TIM_PrescalerConfig(TIM1,0,TIM_PSCReloadMode_Immediate);  //预分频值即时装入
	//TIM_ARRPreloadConfig(TIM1,ENABLE);//使能APR预装载缓冲器

#if  1
  /* Channel 1, 2,3  Configuration in PWM mode */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;   //PWM模式2:CNT>CCR 输出有效
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //输出使能
  TIM_OCInitStructure.TIM_OutputNState = TIM_OutputState_Enable;//互补输出使能
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;     //设置初始极性  -高电平 
  TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCPolarity_High;  //互补输出极性--高电平   下桥臂
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
  TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;
	
  TIM_OC1Init(TIM1, &TIM_OCInitStructure);  //使能通道1  初始化输出比较参数
	//TIM_OC1PreloadConfig(TIM1,TIM_OCPreload_Enable);//使能外设预转载寄存器

  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OC2Init(TIM1, &TIM_OCInitStructure);  //使能通道2

  TIM_OCInitStructure.TIM_Pulse = 0;  //使能通道3
  TIM_OC3Init(TIM1, &TIM_OCInitStructure);
	
#endif
	#if  1
	TIM_ClearFlag(TIM1, TIM_IT_Update);  
	TIM_ClearITPendingBit(TIM1, TIM_IT_Update);                             //清中断标志位
  TIM_ITConfig(TIM1,TIM_IT_Update ,DISABLE);                              //禁止中断 


//	NVIC_InitStructure.NVIC_IRQChannel =TIM1_BRK_UP_TRG_COM_IRQn;// TIM1_BRK_UP_TRG_COM_IRQn
//  NVIC_InitStructure.NVIC_IRQChannelPriority = 1;
//  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//  NVIC_Init(&NVIC_InitStructure);
	
	#endif  
	TIM_Cmd(TIM1, ENABLE);  //使能定时器
	TIM_SelectOutputTrigger(TIM1, TIM_TRGOSource_Update);   
	TIM_CtrlPWMOutputs(TIM1,ENABLE);


}

/*****************************************************************
  * @file     Start_Motor
  * @brief    开启电机
  * @param    无
  * @retval   无
  ***************************************************************/
void Start_Motor(void)
{
	GPIO_SetBits(GPIOB, U_Mos_L_Pin);
  GPIO_SetBits(GPIOA, W_Mos_L_Pin);
  GPIO_SetBits(GPIOB, V_Mos_L_Pin);
	
	TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);
	TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);
	TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);
}
/*****************************************************************
  * @file     MOS_Q15PWM
  * @brief    U相上管通和V相的下管通  其他关闭
  * @param    无
  * @retval   无
  ******************************0*********************************/
void MOS_Q15PWM(void)      
{   
	TIM1->CCR1 = 0; 
	TIM1->CCR2 = 0;
	TIM1->CCR3 = Motor.Duty;	  //PA10输出pwm	 -U上			  

	GPIOB->BSRR = U_Mos_L_Pin;
	GPIOA->BSRR = W_Mos_L_Pin;
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

  TIM1->CCR1= 0; 
	TIM1->CCR2 = 0;
	TIM1->CCR3 = Motor.Duty;   //PA10输出pwm	 -U上							  	

	GPIOB->BSRR =  U_Mos_L_Pin | V_Mos_L_Pin;
	GPIOA->BRR = W_Mos_L_Pin;
	
}
 /*****************************************************************
  * @file     MOS_Q26PWM
  * @brief    V相上管通和W相的下管通  其他关闭
  * @param    无
  * @retval   无
  ***************************************************************/
void MOS_Q26PWM(void)
{    
	
	TIM1->CCR1=0; 
	TIM1->CCR3=0;
	TIM1->CCR2 = Motor.Duty;	//PA9输出pwm -V上			  
 
	GPIOB->BSRR =  U_Mos_L_Pin | V_Mos_L_Pin;
	GPIOA->BRR = W_Mos_L_Pin;

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
	TIM1->CCR3 = 0;	
	TIM1->CCR2 = Motor.Duty;	 //PA9输出pwm -V上				  

	GPIOB->BSRR =   V_Mos_L_Pin;
	GPIOA->BSRR =   W_Mos_L_Pin;
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

	TIM1->CCR2 = 0;
	TIM1->CCR3 = 0;
	TIM1->CCR1 = Motor.Duty;		    //PA8输出pwm	-W上	  

	GPIOB->BSRR =   V_Mos_L_Pin;
	GPIOA->BSRR =   W_Mos_L_Pin;
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

  TIM1->CCR2 = 0; 
	TIM1->CCR3 = 0;
	TIM1->CCR1 = Motor.Duty;		        //W			  
  
	GPIOB->BSRR =   U_Mos_L_Pin;
	GPIOA->BSRR =   W_Mos_L_Pin;
	GPIOB->BRR = V_Mos_L_Pin;


}

#endif
