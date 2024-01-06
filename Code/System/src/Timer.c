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
  * @brief    定时器2初始化   定时60ms
  * @param    无
  * @retval   无
  ***************************************************************/
  void TIM2_Config(void)
  {
    TIM_TimeBaseInitTypeDef TIM_BaseInitStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

  /* Time Base configuration 初始化定时器*/
    TIM_DeInit(TIM2);  
    TIM_BaseInitStructure.TIM_Period = 60000 - 1;    //100 * 1us
    TIM_BaseInitStructure.TIM_Prescaler = 71;      //1Mhz  1us
    TIM_BaseInitStructure.TIM_ClockDivision = 0;     
    TIM_BaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up; 
    TIM_TimeBaseInit(TIM2, &TIM_BaseInitStructure); 
      
    TIM_ClearFlag(TIM2, TIM_FLAG_Update);
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);          ////100us中断一次
      
    TIM_Cmd(TIM2, ENABLE);
 
 }
/*****************************************************************
* @file     TIM3_Config
* @brief    定时器3初始化   定时100us
* @param    无
* @retval   无
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
  TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);         ///中断允许

  TIM_Cmd(TIM3, ENABLE);

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
  // TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCPolarity_High;
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
  TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Set;
  // TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;
	
  TIM_OC1Init(TIM1, &TIM_OCInitStructure);  //使能通道1  初始化输出比较参数
  TIM_OCInitStructure.TIM_Pulse = 0;        //这个值是占空比
  TIM_OC2Init(TIM1, &TIM_OCInitStructure);  //使能通道2
  TIM_OCInitStructure.TIM_Pulse = 0;  
  TIM_OC3Init(TIM1, &TIM_OCInitStructure);  //使能通道3

    // 设定TIM1_CH4的工作状态 为比较输出模式,触发AD转换。通道4的输出GPIO设过了吗？ 或者这个不需要设的吧 因为AD触发的输入是 ADC_ExternalTrigInjecConv_T1_CC4
  /* Channel 4 Configuration in OC */                               ///PA11确实是悬空的没用到这个TIM1_CH4   
                
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;  
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
  TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;                  
  TIM_OCInitStructure.TIM_Pulse = 0;          ///这个在周期结束的时候  这个CC值设的和那个周期值还是一样的  有意思

  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 
  TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;        ///估计这个N没用到吧  
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
  TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Set;   ///估计这个N没用到吧
  TIM_OC4Init(BLDC_TIMx, &TIM_OCInitStructure);

  TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);               
  TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
  TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
  // 设定TIM1_CH4的工作状态 为比较输出模式,触发AD转换。
  TIM_OC4PreloadConfig(BLDC_TIMx,TIM_OCPreload_Enable);

  TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
  TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
  TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_OFF;        
  TIM_BDTRInitStructure.TIM_DeadTime = 20; 															//死区不到0.3us  1/72M * 20 = 13.89ns * 20

#if(MOTOR_BRAKE_ENABLE == 0)//关闭刹车功能        这个程序是启用了这个刹车功能的                    
  TIM_BDTRInitStructure.TIM_Break = TIM_Break_Disable;                                              
  TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_High;                                
#else//开启刹车功能                                                                                   
  TIM_BDTRInitStructure.TIM_Break = TIM_Break_Enable;                                                
  TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_High;        //##   刹车极性为高
#endif
  TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;    
  TIM_BDTRConfig(TIM1, &TIM_BDTRInitStructure); 

  TIM_ARRPreloadConfig(TIM1, ENABLE);  //使能APR预装载缓冲器  

  //刹车
  TIM_ClearITPendingBit(TIM1,TIM_IT_Break);
  TIM_ITConfig(BLDC_TIMx,TIM_IT_CC4,ENABLE);            //定时器中断打开，用于触发AD转换
  TIM_ITConfig(TIM1,TIM_IT_Break,DISABLE);              ///刹车中断关了 
	TIM_Cmd(TIM1, ENABLE);  //使能定时器
  TIM_CtrlPWMOutputs(TIM1, ENABLE); 

  TIM_CCxCmd(TIM1,TIM_Channel_1,TIM_CCx_Disable);         ///CCxE这里先禁止掉，这是为了启动吧  CCE=0
  TIM_CCxNCmd(TIM1,TIM_Channel_1,TIM_CCxN_Disable);      ///CCNE  也没禁止掉，                即CCNE=0
  TIM_CCxCmd(TIM1,TIM_Channel_2,TIM_CCx_Disable);
  TIM_CCxNCmd(TIM1,TIM_Channel_2,TIM_CCxN_Disable);
  TIM_CCxCmd(TIM1,TIM_Channel_3,TIM_CCx_Disable);
  TIM_CCxNCmd(TIM1,TIM_Channel_3,TIM_CCxN_Disable);
  TIM_CCxCmd(TIM1,TIM_Channel_4,TIM_CCx_Disable);
  TIM_CCxNCmd(TIM1,TIM_Channel_4,TIM_CCxN_Disable);
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

  TIM_CCxCmd(TIM1, TIM_Channel_4, TIM_CCx_Enable);
  //TIM_CCxNCmd(TIM1, TIM_Channel_4, TIM_CCxN_Enable);
}
/*****************************************************************
  * @file     MOS_Q15PWM
  * @brief    U相上管通和V相的下管通  其他关闭
  * @param    无
  * @retval   无
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
  * @brief    U相上管通和W相的下管通  其他关闭
  * @param    无
  * @retval   无
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
  * @brief    V相上管通和W相的下管通  其他关闭
  * @param    无
  * @retval   无
  ***************************************************************/
void MOS_Q26PWM(void)
{    
	
    TIM_CCxCmd(BLDC_TIMx,TIM_Channel_1,TIM_CCx_Disable);
    TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_1,TIM_CCxN_Disable);        ////定时器1的通道1禁掉了

    /*  Channel1 configuration */
    /*  Channel2 configuration */  
    TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_2,TIM_CCxN_Disable);           ///先把通道2关掉

    TIM_SetCompare2(BLDC_TIMx,Motor.Duty);     ///再把这个比较器设置一下   但是这个值又是怎么回事呢   这里比较器设的值是 千分之speed_duty占空比
    TIM_CCxCmd(BLDC_TIMx,TIM_Channel_2,TIM_CCx_Enable);           ///  再把这个通道2 允许
    /*  Channel3 configuration */
    TIM_CCxCmd(BLDC_TIMx,TIM_Channel_3,TIM_CCx_Disable);          
    TIM_SetCompare3(BLDC_TIMx,PWM_ARR);                  ////但是这个通道3N 的比较值设的是就是周期  20K那个，50us
    TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_3,TIM_CCxN_Enable);

}
 /*****************************************************************
  * @file     MOS_Q24PWM
  * @brief    V相上管通和U相的下管通  其他关闭
  * @param    无
  * @retval   无
  ***************************************************************/
void MOS_Q24PWM(void) 
{    
	
	
    TIM_CCxCmd(BLDC_TIMx,TIM_Channel_3,TIM_CCx_Disable);
    TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_3,TIM_CCxN_Disable);

    /*  Channel1 configuration */
    TIM_CCxCmd(BLDC_TIMx,TIM_Channel_1,TIM_CCx_Disable);
    TIM_SetCompare1(BLDC_TIMx,PWM_ARR);                 ///看下来，B+的地方，设的就是占空比*周期，而A-的地方，设的就是周期
    TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_1,TIM_CCxN_Enable);

    /*  Channel2 configuration */
    TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_2,TIM_CCxN_Disable);
    TIM_SetCompare2(BLDC_TIMx,Motor.Duty);
    TIM_CCxCmd(BLDC_TIMx,TIM_Channel_2,TIM_CCx_Enable);

}

/*****************************************************************
  * @file     MOS_Q34PWM
  * @brief    W相上管通和U相的下管通  其他关闭
  * @param    无
  * @retval   无
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
  * @brief    W相上管通和V相的下管通  其他关闭
  * @param    无
  * @retval   无
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
void MOS_Q15PWM(void)      //U相上管通和V相的下管通  其他关闭
{ 
  TIM1->CCR1 = Motor.Duty;  
	TIM1->CCR2 = 0;         					  
	TIM1->CCR3 = 0;
  TIM1->CCR4 = Motor.Duty / 2;  
	GPIO_ResetBits(GPIOB, GPIO_Pin_13 | GPIO_Pin_15); 
	GPIO_SetBits(GPIOB, GPIO_Pin_14);   //reset关闭，set导通
}

void MOS_Q16PWM(void)    //U相上管通和W相的下管通  其他关闭
{ 
  TIM1->CCR1 = Motor.Duty;    
	TIM1->CCR2 = 0;            				  
	TIM1->CCR3 = 0;
  TIM1->CCR4 = Motor.Duty / 2; 
	GPIO_ResetBits(GPIOB, GPIO_Pin_13 | GPIO_Pin_14); 
	GPIO_SetBits(GPIOB, GPIO_Pin_15);
}

void MOS_Q26PWM(void)     //V相上管通和W相的下管通  其他关闭
{    
	TIM1->CCR1= 0;       
	TIM1->CCR2 = Motor.Duty;					  
	TIM1->CCR3= 0;
  TIM1->CCR4 = Motor.Duty / 2; 
	GPIO_ResetBits(GPIOB, GPIO_Pin_13 | GPIO_Pin_14); 
	GPIO_SetBits(GPIOB, GPIO_Pin_15); 
}

void MOS_Q24PWM(void)     //V相上管通和U相的下管通  其他关闭
{    
	TIM1->CCR1 = 0;        
	TIM1->CCR2 = Motor.Duty;					  
	TIM1->CCR3 = 0;
  TIM1->CCR4 = Motor.Duty / 2; 
	GPIO_ResetBits(GPIOB, GPIO_Pin_14 | GPIO_Pin_15); 
	GPIO_SetBits(GPIOB, GPIO_Pin_13);
}

void MOS_Q34PWM(void)     //W相上管通和U相的下管通  其他关闭
{
  TIM1->CCR1 = 0;
	TIM1->CCR2 = 0;
	TIM1->CCR3 = Motor.Duty;	
  TIM1->CCR4 = Motor.Duty / 2; 				  
	GPIO_ResetBits(GPIOB, GPIO_Pin_14 | GPIO_Pin_15); 
	GPIO_SetBits(GPIOB, GPIO_Pin_13); 
}

void MOS_Q35PWM(void)     //W相上管通和V相的下管通  其他关闭
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
