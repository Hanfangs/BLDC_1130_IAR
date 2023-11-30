#ifndef   _User_Config_H
#define   _User_Config_H

#include "SysConfig.h"


//PWM����    ���ϼ���
#define			DEF_PWMFRE_8K			8000 //48000000/8000=6000       8000��(arr + 1)
#define			DEF_PWMFRE_16K		16000//48000000/16000=3000 0.02U/N
#define			DEF_PWMFRE_20K		20000//48000000/20000=2400
#define			DEF_PWMFRE_25K		25000//48000000/25000=1920
#define			DEF_PWMFRE_30K		30000//48000000/30000=1600

//PWM����    �м����

#define			Cent_PWMFRE_4K			8000 //48000000/8000=6000
#define			Cent_PWMFRE_8K		  16000//48000000/16000=3000 0.02U/N
#define			Cent_PWMFRE_10K		  20000//48000000/20000=2400
#define			Cent_PWMFRE_15K		  30000//48000000/30000=1600
#define		  Cent_PWMFRE_16K		  32000//48000000/32000=1500


/*---------------------����------------------*/
 
//ϵͳƵ��
#define   SYS_CLK          (48000000)
#define   PWM_FREQ         (Cent_PWMFRE_16K)
#define   PWM_ADJ          (16)
#define   TIM3_Prescaler   (48)
//����
#define ALIGNMENTNMS      (0)                 // ��λʱ�� 
#define ALIGNMENTDUTY     (100)               // ��λ����   0.01-0.1   
 
#define RAMP_TIM_STA      (190)              //  ���¿�ʼ����ʱ�� // ԭ��200  ��ֵԽС�����Ͽ�  �����׹���
#define RAMP_TIM_END      (20)               //  ���½�������ʱ�� 30 ���ݸ��ص�  �������  ���ֵ�ʵ����
#define RAMP_TIM_STEP     (9)                //  ���²���ʱ��ݼ����� --����RAMP_TIM_STA����
#define RAMP_DUTY_STA     (100)              //  ���¿�ʼ����      
#define RAMP_DUTY_END     (PWM_DUTYCYCLE_20)  //  ���½�������       
#define RAMP_DUTY_INC     (13)                //  ���²�����������--��ֵ̫С���������� ̫�����׹���    
#define START_TIMES       (2)       //��������(��ʱ����)
#define Run_Times         (100)     //ǿ�ϵ��ջ��ȶ�ʱ��  n*62.5us  (��ʱ����)
#define Maul_AutoTime     (30)      //�����ȸ���ֵ  ǿ�����Զ�����  ������Ҫ(��ʱ����)
 
 
 //��з�
#define Lock_Duty        1300       //����duty
#define LongPulse_Time   10        //���ʱ��
#define ShortPulse_Time  4          //����ʱ��

 //�������κͻ���ʱ�䲹��
 #define  Low_DutyMask_Time         (2)     //��ռ�ձ���������ʱ��   ���й������޹�Ӳ������  ������ֵ
 #define  High_DutyMask_Time        (3)    //��ռ�ձ���������ʱ�� --���й������޹�Ӳ������  ������ֵ
 #define  Delay_Filter              (2)      //���綯�Ʋɼ��˲����  --Ӱ�첨�εĳ�ǰ���ͺ�
 
 //�������� 
 #define   ADD_DUTY1    (1)       //�������ٷ���
 #define   ADD_DUTY2    (2)       //�������ٷ���
 #define   ADD_DUTY3    (3)       //�������ٷ���
 
 #define   LOW_DUTY_COUNT    (2)   //��ռ�ձȿ������ټ��ٶ�
 #define   HIGH_DUTY_COUNT   (2)   //��ռ�ձȿ������ټ��ٶ�
 #define   DUTYTHRESHOLD1   (0.5*PWM_DUTYCYCLE_100)//��ֵ�趨1
 #define   DUTYTHRESHOLD2   (0.7*PWM_DUTYCYCLE_100)//��ֵ�趨2
 #define   DUTYTHRESHOLD3   (1*PWM_DUTYCYCLE_100)  //��ֵ�趨3
 
 //��ֵ����
 #define   MIN_DUTY     PWM_MIN_DUTY       //��Сռ�ձ�
 #define   MAX_DUTY     PWM_DUTYCYCLE_100   //���ռ�ձ�
 
 
 //ͣ����ʽ
 #define   FREEDOWN     (0)          //����ͣ��
 #define   BRAKEDOWN    (1)          //ǿɲ
 #define   STOPMODE     (FREEDOWN)   //ɲ����ʽѡ��
 
 //�������㷽ʽ
 #define    Filter_Function   (0)   //����
 #define    Filter_Average    (1)   //ȡƽ��ֵ
 #define    Coff_Multi        (2)   //ֱ�ӳ���ϵ��
 #define    Filter_Mode   (Coff_Multi)  //�������㷽ʽ
 
 //���ٷ�ʽ
 #define    DIRECT_GIVE     (0)       //ֱ�Ӹ���
 #define    STEPLESS_SPEED  (1)       //�޼�����
 #define    ADJ_MODE     (DIRECT_GIVE) 
 /*-------------��������������----------------------*/
 
 //PCB�¶ȼ�Ᵽ��
#define  Max_PCB_Temp      (1.314*3.3/(10+1.314))      //0.382V--90��  �¶�ֵ
//ϵͳ��ѹ����
#define  Max_BUS_Voltage    (42)          //��ѹ
#define  Min_BUS_Voltage    (28)          //��ѹ
//������������
#define   Current_k   10
#define   Max_BUS_Curr       (15*Current_k)        //��ʽ=(15/CURRENT_COFF)    AD ֵ10A=400  ADֵ
#define   Filter_Const1      (30)

/*-------------PID������---------------------*/

#define   OPEN_LOOP_Halless     (1)             // 1:��������
#define   CLOSED_SPEEDLOOP_Halless   (2)        // 2���ٶȱջ�
#define   Control_Mode  (OPEN_LOOP_Halless)    //ģʽѡ��

#define  Motor_MinSpeed    (6000)
#define  Motor_MaxSpeed    (22000)
#define  Motor_UserSpeed   (19000)

/*-------------------Ӳ������--------------------*/
//ĸ�ߵ�ѹ����
#define RV_BUS_1      (68.0)         //��λK�� ��ѹ����    Y
#define RV_BUS_2      (4.70)         //��λK�� ��������    Y


//�¶Ȳ���

//------------------------------------------------------------------------
//��������
#define  Rs            (0.001)     //��λ��   ��������    Y
#define  RI_BUS_1      (10.0)        //��λK��  ����Ŵ������� Y
#define  RI_BUS_2      (1.8)          //��λK��  ����Ŵ������� Y 
#define  AMP_GAIN     (RI_BUS_1/RI_BUS_2+1)
#define CURRENT_COFF  (3.3/(4095*AMP_GAIN*Rs))         //��������ϵ��  0.0248
//��������趨
#define POLE_PAIR     (2)         //��λ* ���������         Y
#define BASE_VOLTAGE  (36)        //��λV ������ѹ       Y
#define BASE_SPEED    (20000)     //��λrpm ����ת��*1.5 Y
//ת�ټ�������
#define  SPEEDFACTOR	(5000000)//(60 * SYS_CLK / (6 * POLE_PAIR * TIM3_Prescaler) )=1000 0000/POLE_PAIR// ����ת�ٵ�ϵ��  266666 400000
       

#endif

