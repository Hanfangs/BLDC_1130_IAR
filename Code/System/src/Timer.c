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
#if 0
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;
  TIM_BDTRInitTypeDef TIM_BDTRInitStructure;

	//TIM_DeInit(TIM1);
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
  TIM_OCInitStructure.TIM_Pulse = PWM_ARR - 1;          ///这个在周期结束的时候  这个CC值设的和那个周期值还是一样的  有意思

  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 
  TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;        ///估计这个N没用到吧  
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
  TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;   ///估计这个N没用到吧
  TIM_OC4Init(BLDC_TIMx, &TIM_OCInitStructure);

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

  TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);               
  TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
  TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
  // 设定TIM1_CH4的工作状态 为比较输出模式,触发AD转换。
  TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);

  TIM_ARRPreloadConfig(TIM1, ENABLE);  //使能APR预装载缓冲器  

  //刹车
  TIM_ClearITPendingBit(TIM1,TIM_IT_Break);
  TIM_ITConfig(TIM1,TIM_IT_CC4,ENABLE);            //定时器中断打开，用于触发AD转换
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
#else
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;
    TIM_BDTRInitTypeDef TIM_BDTRInitStructure;
    
     
    /* 定时器基本参数始终 */		 
    TIM_TimeBaseStructure.TIM_Period = PWM_ARR-1;            ////这个值的实际值为3600-1           经计算是20K频率  这个时间是50us
    /* 设置预分频：不预分频，即为72MHz */
    TIM_TimeBaseStructure.TIM_Prescaler = BLDC_TIM_PRESCALER;        ///不分频
    /* 设置时钟分频系数：不分频(这里用不到) */
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1 ;        ///不分频
    /* 向上计数模式 */
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;     ///这个是向上计数    但是那个HALL可不是  
    /* 重复计算器 */
    TIM_TimeBaseStructure.TIM_RepetitionCounter = BLDC_TIM_REPETITIONCOUNTER;    ////这个等于0 没用到
    TIM_TimeBaseInit(BLDC_TIMx, &TIM_TimeBaseStructure);
  
    /* 定时器输出通道1模式配置 */
    /* 模式配置：PWM模式1 */
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;              ////TIMx_CNT<TIMx_CCR1时通道1为有效电平
    /* 输出状态设置：使能输出 */
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	
    /* 互补通道输出状态设置：使能输出 */
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable; ///两个输出都使能了

    /* 设置跳变值，当计数器计数到这个值时，电平发生跳变    这里为什么设置为0  先设为0 总是无效电平  */
    TIM_OCInitStructure.TIM_Pulse = 0;      ///这个值等于0的话，则意味着这个永远不会有那个有效电平了
    /* 当定时器计数值小于CCR1_Val时为高电平 */
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;          ///正极性输出是高电平有效，而负极性输出是低电平有效 所以当时间来时，一个变高一个变低从而不会短路
    TIM_OCInitStructure.TIM_OCNPolarity= TIM_OCNPolarity_Low;         //// 切记这个值=0008  也就是 CCxNP=1  故记住在这里，将这个值设成低电平有效
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;     ///这个是用来干什么的呢  空闲时，它是复位的 也就是低电平    0：当MOE=0时，如果实现了OC1N，则死区后OC1=0；
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Set;//0x0200    /// 空闲时 这个N为高电平 由于我们MOE现等于0 故这个是设初始化后的状态   //  1：当MOE=0时，如果实现了OC1N，则死区后OC1=1
    /* 初始化定时器通道1输出PWM */                              /// 1：当MOE=0时，死区后OC1N=1。 也就是说，当MOE=0时，这个N的输出是高电平，也就是无效电平  这样两个管子都不导通。
    TIM_OC1Init(BLDC_TIMx, &TIM_OCInitStructure);                   //// 一旦将CCNP设为低电平有效，也就是将这个CCxNP=1 那么在CC1E和CC1NE=0的情况下:
                                                                     ////                                                    OCx=CCxP=0   OCxN=CCxNP=1   故此时是上管关闭，而下管也是关闭了。
    /* 定时器输出通道2模式配置 */                                    ////所以在初始化的情况下，这个管子全部是关闭的，至于真正运行之后，这个值就等于 OCxREF + 极性 + 死区和OCxREF反相 + 极性 + 死区了
    /* 设置通道2的电平跳变值，输出另外一个占空比的PWM */            ////真正运行时的情况 在换相时，我们6个输出全都是设过的。唯有其中的两个是允许，其它的4个都是DISABLE的
    TIM_OCInitStructure.TIM_Pulse = 0;                                              ////这也就是说，其它的4个输出都是置为不导通的状态的。只有两个导通，其中一个是一直通，另一个是按占空比通。
    /* 初始化定时器通道2输出PWM */                                                  ///
    TIM_OC2Init(BLDC_TIMx, &TIM_OCInitStructure);
  
    /* 定时器输出通道3模式配置 */
    /* 设置通道3的电平跳变值，输出另外一个占空比的PWM */
    TIM_OCInitStructure.TIM_Pulse = 0;
    /* 初始化定时器通道3输出PWM */
    TIM_OC3Init(BLDC_TIMx, &TIM_OCInitStructure);
    
    // 设定TIM1_CH4的工作状态 为比较输出模式,触发AD转换。通道4的输出GPIO设过了吗？ 或者这个不需要设的吧 因为AD触发的输入是 ADC_ExternalTrigInjecConv_T1_CC4
  /* Channel 4 Configuration in OC */                               ///PA11确实是悬空的没用到这个TIM1_CH4   
                
   TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;  
   TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
   TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;                  
   TIM_OCInitStructure.TIM_Pulse = PWM_ARR - 1;          ///这个在周期结束的时候  这个CC值设的和那个周期值还是一样的  有意思
  
   TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 
   TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;        ///估计这个N没用到吧  
   TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
   TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;   ///估计这个N没用到吧
   TIM_OC4Init(BLDC_TIMx, &TIM_OCInitStructure);
      // 设定TIM1_CH4的工作状态 为比较输出模式,触发AD转换。
 
   
 ////######################################################################################################   
    /* Automatic Output enable, Break, dead time and lock configuration*/                              //##
    TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;                                        //##   OSSR=1 
    TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;                                        //##   OSSI=1 
    TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_OFF;                                           //## 
    TIM_BDTRInitStructure.TIM_DeadTime = 20;//1/72M = 13.89ns                                          //##   不到 0.3us
                                                                                                       //##
#if(MOTOR_BRAKE_ENABLE == 0)//关闭刹车功能        这个程序是启用了这个刹车功能的                       //##
    TIM_BDTRInitStructure.TIM_Break = TIM_Break_Disable;                                               //## 
    TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_High;                                  //##
#else//开启刹车功能                                                                                    //##
    TIM_BDTRInitStructure.TIM_Break = TIM_Break_Enable;                                                //##
    TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_High;                                  //##   刹车极性为高
#endif                                                                                                 //##
                                                                                                       //##
    TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;                            //##     1：MOE能被软件置’1’或在下一个更新事件被自动置’1’(如果刹车输入无效)
    TIM_BDTRConfig(BLDC_TIMx, &TIM_BDTRInitStructure);                                                 //##
 ////######################################################################################################   
    TIM_OC1PreloadConfig(BLDC_TIMx,TIM_OCPreload_Enable);           ////开启这个CC值的预装载功能 更新事件到来时，才更新这个CC的值 
    TIM_OC2PreloadConfig(BLDC_TIMx,TIM_OCPreload_Enable);          
    TIM_OC3PreloadConfig(BLDC_TIMx,TIM_OCPreload_Enable);
    
    // 设定TIM1_CH4的工作状态 为比较输出模式,触发AD转换。
    TIM_OC4PreloadConfig(BLDC_TIMx,TIM_OCPreload_Enable);
   /*电机电流ADC采样初始化*/
   // MotorCurrentAdcChannel_Init();

    /* 使能定时器重载寄存器ARR */
    TIM_ARRPreloadConfig(BLDC_TIMx, ENABLE);
  
    TIM_ITConfig(BLDC_TIMx,TIM_IT_CC4,DISABLE);               
    
    //刹车
    TIM_ClearITPendingBit(BLDC_TIMx,TIM_IT_Break);
    TIM_ITConfig(BLDC_TIMx,TIM_IT_Break,DISABLE);              ///刹车中断也中关了
    /* 使能定时器 */
    TIM_Cmd(BLDC_TIMx, ENABLE);
    
    /* TIM主输出使能 */
    TIM_CtrlPWMOutputs(BLDC_TIMx, ENABLE);                ////这个就是MOE =1   注意到初始化之后这个值等于1 后面再也没有复位过了 所以一切的电平计算，都是以MOE=1为基准
    
    TIM_CCxCmd(BLDC_TIMx,TIM_Channel_1,TIM_CCx_Disable);         ///CCxE这里先禁止掉，这是为了启动吧  CCE=0
    TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_1,TIM_CCxN_Disable);      ///CCNE  也没禁止掉，                即CCNE=0
    TIM_CCxCmd(BLDC_TIMx,TIM_Channel_2,TIM_CCx_Disable);
    TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_2,TIM_CCxN_Disable);
    TIM_CCxCmd(BLDC_TIMx,TIM_Channel_3,TIM_CCx_Disable);
    TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_3,TIM_CCxN_Disable);
    TIM_CCxCmd(BLDC_TIMx,TIM_Channel_4,TIM_CCx_Disable);   //都禁止了，故此时6个管脚的输出值取决于   MOE=1、OSSI=1、OSSR=1、OIS1无关因为MOE=1了、OIS1N无关   和CC1E=1 CC1NE=1 位的值
    TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_4,TIM_CCxN_Disable);  
#endif
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
