
#include "PI_Cale.h"
#include "BLDC.h" 
#include "Analog.h"
#include "SysConfig.h"
#include "Timer_ISR.h"
#include "User_Config.h"


 void bldcSpeedControlTime(int32_t Idle_SpeedValue,int32_t Actual_SpeedValue);  //�ٶȻ�

__IO uint16_t Limit_speed_duty = 1500; //������������ռ�ձ�

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
							100													/* ��Сռ�ձ�*/

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
void ErrorCorrection(void)
{

}
/*****************************************************************************
 �� �� ��  : CalcSpeedTime
 ��������  : ����ʱ��[60��]		��û������
 �������  : ��
 �������  : void
*****************************************************************************/
 void CalcSpeedTime(void)
{	
	uint16_t theta; 
	theta = atan(DELAYFACTOR * Motor.ActualSpeed);		//�ӳٽ�
	// Sysvariable.CorrectionTime = 0.8 * Sysvariable.DelayTime30;		//�ӳ�ʱ��
	TIM3->ARR = Sysvariable.DelayTime30 - Sysvariable.CorrectionTime;		//���¶�ʱ�ˣ���λus
	// TIM3->ARR=Sysvariable.DelayTime30;
	TIM3->CR1|=0x0001;       //�򿪶�ʱ��
	TIM3->CNT = 0;
	
	Sysvariable.SpeedTimeTemp=TIM2->CNT; //�����ʱ����  60��  ��TIM2�ļ���ֵ��¼һ��  1usһ������
	TIM2->CNT = 0;
	
	Sysvariable.DelayTime30=(uint16_t)(Sysvariable.SpeedTimeTemp/2); 
	Sysvariable.SpeedTime=Sysvariable.SpeedTimeTemp;
	Sysvariable.LastDragTime = Sysvariable.SpeedTimeTemp;
	sysflags.ChangePhase=1; //��ʼ����		����ֻ�Ǹ��˸���־λ����û��ʱ�꣬�ȶ�ʱ�жϵ��˲Ż���
}
/*****************************************************************************
 �� �� ��  : SpeedController
 ��������  : ռ�ձȿ���
 �������  : ��
 �������  : void
*****************************************************************************/
#if SHF_TEST_SPEED
PID bldc_pid;
void IncPIDInit(void) 
{
	// Motor.motor_speed   = Motor.Duty * 16 / 30 ;			//���Ҫ���ݼ�����ɺ���ٶ�����
	Motor.motor_speed = Motor.ActualSpeed + MOTOR_ACC_DELTA_SPEED;
	bldc_pid.LastError  = 0;                    //Error[-1]
	bldc_pid.PrevError  = 0;                    //Error[-2]
	bldc_pid.Proportion = P_DATA_ACC;              //�������� Proportional Const
	bldc_pid.Integral   = I_DATA_ACC;                //���ֳ���  Integral Const
	bldc_pid.Derivative = D_DATA_ACC;              //΢�ֳ��� Derivative Const
	bldc_pid.SetPoint   = Motor.motor_speed;		//�趨Ŀ��Desired Value     �ղ��Ѿ���Ϊ100��
}

int IncPIDCalc(int NextPoint) 
{
    int iError,iIncpid;                                       //��ǰ���
    
    iError = bldc_pid.SetPoint - NextPoint;                     //��������
    iIncpid = (int)((bldc_pid.Proportion * iError)             /*E[k]��*/\
                -(bldc_pid.Integral * bldc_pid.LastError)     /*E[k-1]��*/\
                +(bldc_pid.Derivative * bldc_pid.PrevError));  /*E[k-2]*/
                
    bldc_pid.PrevError = bldc_pid.LastError;                    //�洢�������´μ���
    bldc_pid.LastError = iError;
    
    return(iIncpid);                                    //��������ֵ
}

//ת��PI���� + ת�ټ���
void SpeedController(void)			
{
	uint16_t tmp_Min_Speed_Duty;
	if(mcState == mcRun || mcState == mcDec)
	{
		TuneDutyRatioCnt ++;	
		if(TuneDutyRatioCnt >= SPEEDLOOPCNT)	// �ٶȻ�	50ms����һ��
		{
			static uint16_t DutyInc;	//�����õ�
			int pid_result;

			// if(sysflags.SwapFlag == 0)
			// {
			// 	Motor.ActualSpeed = Motor.Duty * 16 / 30;	//ռ�ձȺ��ٶȼ��ת��
			// 	sysflags.SwapFlag = 1;
			// }
			// else
			// {
			// Motor.ActualSpeed = (Motor.step_counter + Motor.step_counter_prev) * 25;		//��ǰ50ms��ֵȡ��ֵ��tempֵ���ǵ�ǰ�ٶ�ֵ
			//M������
			// Motor.ActualSpeed = (Motor.step_counter * 60 * 1000)/ (24 * SPEEDLOOPCNT);		//n=(60*m1)/(P*Tg)=60*(cnt/24)*(1/50ms)
			//T������
			Motor.ActualSpeed = SPEEDFACTOR / Sysvariable.SpeedTime;	//60��		SpeedTime����60������������1������ʱ��Ϊ1/72M������Ƶ�й�
			// }

			bldc_pid.SetPoint = Motor.motor_speed;		//motor_speed��һֱ�ڱ�ģ�1ms��һ�Σ�Ҳ����˵ÿ��motor_speed��50
			pid_result = IncPIDCalc(Motor.ActualSpeed);

			pid_result = pid_result * 30 / 16;		//ռ�ձȺ��ٶȵ�һ��ת����ת�٣�0~1600����ռ�ձȣ�0~3000��
			DutyInc = pid_result;
		
			if (mcState == mcDec)
			{
				tmp_Min_Speed_Duty = MOTOR_DEC_MIN_DUTY_SPEED;		//���ٵ���Сռ�ձ�
			} else
			{
				tmp_Min_Speed_Duty = MOTOR_MIN_DUTY_SPEED;			//���ٵ���Сռ�ձ�
			}

			if((pid_result + Motor.Duty) < MOTOR_MIN_DUTY_SPEED)
			{
				Motor.Duty = MOTOR_MIN_DUTY_SPEED;        //ռ�ձ���С120
			}
			else if((pid_result + Motor.Duty) > MOTOR_MAX_DUTY_SPEED)//950->600
			{
				Motor.Duty = MOTOR_MAX_DUTY_SPEED;      //���ռ�ձ�
			}
			else
			{            
				Motor.Duty += pid_result;  
				if(Motor.ActualSpeed <= 100)                 ///��ת�ٱȽϵ͵�����£���������ռ�ձ��ֱȽϴ�Ļ�����ɵ���ܲ��˵�
				{
					if(Motor.Duty >= Limit_speed_duty)         
					{
						Motor.Duty = Limit_speed_duty;
					}
				}
			}

			TuneDutyRatioCnt = 0;
			Motor.step_counter_prev = Motor.step_counter;
			Motor.step_counter = 0;
		}
	}
}

#else
 void SpeedController(void)
{	
	if(mcState == mcRun)
	{
		// TuneDutyRatioCnt ++;	
		if(TuneDutyRatioCnt >= SPEEDLOOPCNT)	// �ٶȻ�	10ms����һ��
		{
			TuneDutyRatioCnt = 0;
			Motor.Last_Speed = SPEEDFACTOR / Sysvariable.SpeedTime;   //60��		SpeedTime����60����������
			// FirstOrder_LPF_Cacl(Motor.Last_Speed,Motor.ActualSpeed,0.06);		//�õ�Ӳ���˲�
			// bldcSpeedControlTime(UserRequireSpeed,Motor.ActualSpeed);
			bldcSpeedControlTime(UserRequireSpeed,Motor.Last_Speed);
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
#endif



