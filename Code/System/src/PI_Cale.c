
#include "PI_Cale.h"
#include "BLDC.h" 
#include "Analog.h"
#include "SysConfig.h"
#include "Timer_ISR.h"
#include "User_Config.h"

 void bldcSpeedControlTime(int32_t Idle_SpeedValue,int32_t Actual_SpeedValue);  //�ٶȻ�

PIDCONTROL   PID_Speed = {	0.01,                                                 /* p����Ϊ10                    */
							0.005,                                                   /* i����Ϊ3                     */
							0,                                                     /* d����Ϊ0                     */
							1500,                                                  /* ������ֵ9A   */
							300,                                                   /* ��С���ֵ  300->600     */		
							0,
							0,                                                     /* ����ۼƻ���Ϊ0                  */
							0,                                                   /* ��ʼ��ֵ               */
							INIT_PURPOSE,                                       /* ����PIDʱ���¼���������    */
							0,
							0,
							1500,												/* ���ռ�ձ�*/
							300													/* ��Сռ�ձ�*/

}; 

/*****************************************************************************
 �� �� ��  : CalcAverageSpeedTime
 ��������  : ��ת��ʱ����ƽ��	û�ã�
 �������  : ��
 �������  : void
*****************************************************************************/
void CalcAvgSpeedTime(void)
{
	if(!sysflags.flag_SpeedTime)
	{
		Sysvariable.SpeedTime = Sysvariable.SpeedTimeTemp;	
	}
	Sysvariable.SpeedTimeSum += Sysvariable.SpeedTimeTemp;
	
	Sysvariable.SpeedTimeCnt ++;	
	if(Sysvariable.SpeedTimeCnt >= 6)
	{
		if(!sysflags.flag_SpeedTime)
		{
			sysflags.flag_SpeedTime = 1;	
		}
		Sysvariable.SpeedTime = (uint16_t)(Sysvariable.SpeedTimeSum /6);	//һ�λ���ʱ��
		Sysvariable.SpeedTimeSum = 0;
		Sysvariable.SpeedTimeCnt = 0;		
	}
}
/*****************************************************************************
 �� �� ��  : CalcSpeedTime
 ��������  : ����ʱ��[60��]
 �������  : ��
 �������  : void
*****************************************************************************/
 void CalcSpeedTime(void)
{	 
	
	TIM3->ARR=Sysvariable.DelayTime30;		//���¶�ʱ�ˣ���λus
	TIM3->CR1|=0x0001;       //�򿪶�ʱ��
	TIM3->CNT = 0;
	
	Sysvariable.SpeedTimeTemp=TIM2->CNT; //�����ʱ����  60��  ��TIM2�ļ���ֵ��¼һ��  1usһ������	//���ܻ���Ҫ�ĳ�1s��
	TIM2->CNT = 0;
	
	Sysvariable.DelayTime30=(uint16_t)(Sysvariable.SpeedTimeTemp/2); 
	Sysvariable.SpeedTime=Sysvariable.SpeedTimeTemp;
	sysflags.ChangePhase=1; //��ʼ����		����ֻ�Ǹ��˸���־λ����û��ʱ�꣬�ȶ�ʱ�жϵ��˲Ż���
}
/*****************************************************************************
 �� �� ��  : SpeedController
 ��������  : ռ�ձȿ���
 �������  : ��
 �������  : void
*****************************************************************************/
 void SpeedController(void)
{	
	if(	Motor.ControlMode ==CLOSED_SPEEDLOOP_Halless) //�ջ�����
	{
		if(TuneDutyRatioCnt >= SPEEDLOOPCNT)	// �ٶȻ�	10ms����һ��
		{
			TuneDutyRatioCnt = 0;
			Motor.Last_Speed = SPEEDFACTOR / Sysvariable.SpeedTime;   //60��/60��ʱ��
			//FirstOrder_LPF_Cacl(Motor.Last_Speed,Motor.ActualSpeed,0.06);		//�õ�Ӳ���˲�
			//bldcSpeedControlTime(UserRequireSpeed,Motor.ActualSpeed);
			bldcSpeedControlTime(UserRequireSpeed,Motor.Last_Speed);
		}
	}

  else//��������
  {
		  if(TuneDutyRatioCnt >= SPEEDLOOPCNT)	
			{
				 TuneDutyRatioCnt = 0;
				 Motor.StepNum++;
				 Motor.Last_Speed = SPEEDFACTOR / Sysvariable.SpeedTime;

				 FirstOrder_LPF_Cacl(Motor.Last_Speed,Motor.ActualSpeed,0.06);
		  	 if(Motor.Duty < PWM_MIN_DUTY)
				 {
					 
					  if(Motor.StepNum>LOW_DUTY_COUNT)
						{
							 Motor.StepNum=0;
						   Motor.Duty += ADD_DUTY1;		//С����Сռ�ձ�ʱ��ռ�ձ�ÿ����+1			 
					  }
				 }
			   else
				 {
					if(Motor.Duty < UserRequireSpeed)		//С�����ռ�ձ� 1500
					{
						if(Motor.Duty<=DUTYTHRESHOLD1)		//С��0.5*���ռ�ձ�
						{
							if(Motor.StepNum>LOW_DUTY_COUNT)
							{
								 Motor.StepNum=0;
								 Motor.Duty += ADD_DUTY1;	//ռ�ձ�ÿ����+1		 
							}
						}
						else if(Motor.Duty<DUTYTHRESHOLD2)		//С��0.7*���ռ�ձ�
						{
							if(Motor.StepNum>HIGH_DUTY_COUNT)
					   	{
							 Motor.StepNum=0;
						   Motor.Duty += ADD_DUTY2;			//	ռ�ձ�ÿ����+2	 
						  }
						}
						else
						{
							if(Motor.StepNum>HIGH_DUTY_COUNT)
						  {
								 Motor.StepNum=0;
								 Motor.Duty += ADD_DUTY3;		//	ռ�ձ�ÿ����+3		 
						  }
						}
					}
					else if(Motor.Duty  > UserRequireSpeed)		//�������ռ�ձ�
					{
						Motor.Duty -= ADD_DUTY2;
						Motor.StepNum=0;

					} 

				}
			}
			if(Motor.Duty <= MIN_DUTY)	// �������ռ�ձȵ������Сֵ
			{
				Motor.Duty = MIN_DUTY;
			}
			else if(Motor.Duty >= MAX_DUTY)
			{
				Motor.Duty = MAX_DUTY;
			}
	}
}
/*****************************************************************************
 �� �� ��  : bldcSpeedControlTime
 ��������  : �ٶ�PID
 �������  : ��
 �������  : void
*****************************************************************************/
void bldcSpeedControlTime(int32_t Idle_SpeedValue,int32_t Actual_SpeedValue)
{
    float    SpeedTemp;                                                         /* ��ʱ����                     */
	if ( PID_Speed.Purpose == INIT_PURPOSE ) 
	{			  	
		PID_Speed.fpAllErr=0;
		PID_Speed.Purpose = RUN_PURPOSE;
	}	
	PID_Speed.Error=Idle_SpeedValue -Actual_SpeedValue;		 
	PID_Speed.fpAllErr+=PID_Speed.Error*PID_Speed.Kp;
	PID_Speed.Ui=PID_Speed.Ki*PID_Speed.fpAllErr; 
	if ( PID_Speed.fpAllErr > PID_Speed.MaxValue/PID_Speed.Ki) 	/* ���ƻ������� �����������9000MA  */
	{              
		PID_Speed.fpAllErr = PID_Speed.MaxValue/PID_Speed.Ki;
	}
	if ( PID_Speed.fpAllErr < PID_Speed.MinValue/PID_Speed.Ki) /* ���ƻ������� �����С����ֵ*/
	{              
		PID_Speed.fpAllErr = PID_Speed.MinValue/PID_Speed.Ki;
	}
	
	SpeedTemp=PID_Speed.Kp*PID_Speed.Error+PID_Speed.Ui;
		 			
	PID_Speed.Out = SpeedTemp;
			
   if (PID_Speed.Out >= PID_Speed.Max) {                               /* ռ�ձ�����������             */
        PID_Speed.Out = PID_Speed.Max;
    }
    if (PID_Speed.Out < PID_Speed.Min) {
        PID_Speed.Out = PID_Speed.Min;
    }
		Motor.Duty=PID_Speed.Out; 
}



