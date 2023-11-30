#ifndef   _User_Config_H
#define   _User_Config_H

#include "SysConfig.h"


//PWM周期    向上计数
#define			DEF_PWMFRE_8K			8000 //48000000/8000=6000       8000是(arr + 1)
#define			DEF_PWMFRE_16K		16000//48000000/16000=3000 0.02U/N
#define			DEF_PWMFRE_20K		20000//48000000/20000=2400
#define			DEF_PWMFRE_25K		25000//48000000/25000=1920
#define			DEF_PWMFRE_30K		30000//48000000/30000=1600

//PWM周期    中间对齐

#define			Cent_PWMFRE_4K			8000 //48000000/8000=6000
#define			Cent_PWMFRE_8K		  16000//48000000/16000=3000 0.02U/N
#define			Cent_PWMFRE_10K		  20000//48000000/20000=2400
#define			Cent_PWMFRE_15K		  30000//48000000/30000=1600
#define		  Cent_PWMFRE_16K		  32000//48000000/32000=1500


/*---------------------设置------------------*/
 
//系统频率
#define   SYS_CLK          (48000000)
#define   PWM_FREQ         (Cent_PWMFRE_16K)
#define   PWM_ADJ          (16)
#define   TIM3_Prescaler   (48)
//启动
#define ALIGNMENTNMS      (0)                 // 定位时间 
#define ALIGNMENTDUTY     (100)               // 定位力矩   0.01-0.1   
 
#define RAMP_TIM_STA      (190)              //  爬坡开始步进时间 // 原本200  数值越小启动较快  ，容易过流
#define RAMP_TIM_END      (20)               //  爬坡结束步进时间 30 根据负载调  如果带载  这个值适当大点
#define RAMP_TIM_STEP     (9)                //  爬坡步进时间递减增量 --跟随RAMP_TIM_STA调整
#define RAMP_DUTY_STA     (100)              //  爬坡开始力矩      
#define RAMP_DUTY_END     (PWM_DUTYCYCLE_20)  //  爬坡结束力矩       
#define RAMP_DUTY_INC     (13)                //  爬坡步进力矩增量--数值太小启动不起来 太大容易过流    
#define START_TIMES       (2)       //启动次数(暂时不用)
#define Run_Times         (100)     //强拖到闭环稳定时间  n*62.5us  (暂时不用)
#define Maul_AutoTime     (30)      //进来先给初值  强拖切自动运行  参数重要(暂时不用)
 
 
 //电感法
#define Lock_Duty        1300       //脉冲duty
#define LongPulse_Time   10        //充电时间
#define ShortPulse_Time  4          //脉冲时间

 //续流屏蔽和换相时间补偿
 #define  Low_DutyMask_Time         (2)     //低占空比续流屏蔽时间   运行过程中无故硬件过流  调整该值
 #define  High_DutyMask_Time        (3)    //高占空比续流屏蔽时间 --运行过程中无故硬件过流  调整该值
 #define  Delay_Filter              (2)      //反电动势采集滤波深度  --影响波形的超前和滞后
 
 //加速限制 
 #define   ADD_DUTY1    (1)       //开环加速幅度
 #define   ADD_DUTY2    (2)       //开环加速幅度
 #define   ADD_DUTY3    (3)       //开环加速幅度
 
 #define   LOW_DUTY_COUNT    (2)   //低占空比开环加速加速度
 #define   HIGH_DUTY_COUNT   (2)   //高占空比开环加速加速度
 #define   DUTYTHRESHOLD1   (0.5*PWM_DUTYCYCLE_100)//阈值设定1
 #define   DUTYTHRESHOLD2   (0.7*PWM_DUTYCYCLE_100)//阈值设定2
 #define   DUTYTHRESHOLD3   (1*PWM_DUTYCYCLE_100)  //阈值设定3
 
 //幅值限制
 #define   MIN_DUTY     PWM_MIN_DUTY       //最小占空比
 #define   MAX_DUTY     PWM_DUTYCYCLE_100   //最大占空比
 
 
 //停机方式
 #define   FREEDOWN     (0)          //自由停车
 #define   BRAKEDOWN    (1)          //强刹
 #define   STOPMODE     (FREEDOWN)   //刹车方式选择
 
 //电流计算方式
 #define    Filter_Function   (0)   //函数
 #define    Filter_Average    (1)   //取平均值
 #define    Coff_Multi        (2)   //直接乘以系数
 #define    Filter_Mode   (Coff_Multi)  //电流计算方式
 
 //调速方式
 #define    DIRECT_GIVE     (0)       //直接给定
 #define    STEPLESS_SPEED  (1)       //无极变速
 #define    ADJ_MODE     (DIRECT_GIVE) 
 /*-------------保护参数设置区----------------------*/
 
 //PCB温度检测保护
#define  Max_PCB_Temp      (1.314*3.3/(10+1.314))      //0.382V--90℃  温度值
//系统电压保护
#define  Max_BUS_Voltage    (42)          //过压
#define  Min_BUS_Voltage    (28)          //低压
//软件过流保护
#define   Current_k   10
#define   Max_BUS_Curr       (15*Current_k)        //公式=(15/CURRENT_COFF)    AD 值10A=400  AD值
#define   Filter_Const1      (30)

/*-------------PID控制区---------------------*/

#define   OPEN_LOOP_Halless     (1)             // 1:开环运行
#define   CLOSED_SPEEDLOOP_Halless   (2)        // 2：速度闭环
#define   Control_Mode  (OPEN_LOOP_Halless)    //模式选择

#define  Motor_MinSpeed    (6000)
#define  Motor_MaxSpeed    (22000)
#define  Motor_UserSpeed   (19000)

/*-------------------硬件参数--------------------*/
//母线电压采样
#define RV_BUS_1      (68.0)         //单位KΩ 分压电阻    Y
#define RV_BUS_2      (4.70)         //单位KΩ 采样电阻    Y


//温度采样

//------------------------------------------------------------------------
//电流采样
#define  Rs            (0.001)     //单位Ω   采样电阻    Y
#define  RI_BUS_1      (10.0)        //单位KΩ  运算放大器电阻 Y
#define  RI_BUS_2      (1.8)          //单位KΩ  运算放大器电阻 Y 
#define  AMP_GAIN     (RI_BUS_1/RI_BUS_2+1)
#define CURRENT_COFF  (3.3/(4095*AMP_GAIN*Rs))         //电流计算系数  0.0248
//电机参数设定
#define POLE_PAIR     (2)         //单位* 电机极对数         Y
#define BASE_VOLTAGE  (36)        //单位V 电机额定电压       Y
#define BASE_SPEED    (20000)     //单位rpm 电机额定转速*1.5 Y
//转速计算因子
#define  SPEEDFACTOR	(5000000)//(60 * SYS_CLK / (6 * POLE_PAIR * TIM3_Prescaler) )=1000 0000/POLE_PAIR// 计算转速的系数  266666 400000
       

#endif


