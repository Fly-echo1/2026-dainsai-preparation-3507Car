#include "bsp_track.h"
#include "board.h"
#include "stdio.h"
#include "bsp_motor.h"

float Kp = 700, Ki=0, Kd =6.5;//PID参数
float P = 0, I = 0, D = 0, PID_value = 0;
float new_error = 0, previous_error = 0;
static int initial_motor_speed = 2500;//基础速度

void track_init(void)
{
	printf("Track initialized successfully\r\n");
}

int track_scan(void)
{
	if(!L3 && L2 && L1 && M0 && R1 && R2 && R3)//1 0 0 0 0 0 0
	{
		new_error = -4.5;
	}
	if(!L3 && !L2 && L1 && M0 && R1 && R2 && R3)//1 1 0 0 0 0 0
	{
		new_error = -4.5;
	}
	if(L3 && !L2 && L1 && M0 && R1 && R2 && R3)//0 1 0 0 0 0 0
	{
		new_error = -4;
	}
	if(L3 && !L2 && !L1 && M0 && R1 && R2 && R3)//0 1 1 0 0 00
	{
		new_error = -3;
	}
	if(L3 && L2 && !L1 && M0 && R1 && R2 && R3)//0 0 1 0 0 0 0 
	{
		new_error = -2;
	}
	if(L3 && L2 && !L1 && !M0 && R1 && R2 && R3)//0 0 1 1 0 0 0 
	{
		new_error = -1;
	}
	if(L3 && L2 && !L1 && !M0 && !R1 && R2 && R3)//0 0 1 1 1 0 0 
	{
		new_error = 0;
	}
	if(L3 && L2 && L1 && !M0 && R1 && R2 && R3)//0 0 0 1 0 0 0
	{
		new_error = 0;
	}
	
	if(L3 && L2 && L1 && !M0 && !R1 && R2 && R3)//0 0 0 1 1 0 0 
	{
		new_error = 1;
	}
	if(L3 && L2 && L1 && M0 && !R1 && R2 && R3)//0 0 0 0 1 0 0
	{
		new_error = 2;
	}
	if(L3 && L2 && L1 && M0 && !R1 && !R2 && R3)//0 0 0 0 1 1 0
	{
		new_error = 3;
	}
	if(L3 && L2 && L1 && M0 && R1 && !R2 && R3)//0 0 0 0 0 1 0
	{
		new_error = 4;
	}	
	if(L3 && L2 && L1 && M0 && R1 && !R2 && !R3)//0 0 0 0 0 1 1
	{
		new_error = 4.5;
	}
	if(L3 && L2 && L1 && M0 && R1 && R2 && !R3)//0 0 0 0 0 0 1
	{
		new_error =4.5;
	}
	if(L3 && L2 && L1 && M0 && R1 && R2 && R3)//0 0 0 0 0 0 0
	{
		new_error = 0;
	}
	if (L3 && L2 && L1 && M0 && R1 && R2 && R3)
	{
		return 1;
	}
	else {
		return 0;
	}
}


int track_pid(void)
{  
	P=new_error;	        //当前误差
	I=I+new_error;	        //误差累加
	D=new_error-previous_error;	//当前误差与之前误差的误差
	
	PID_value=(Kp*P)+(Ki*I)+(Kd*D);
	
	previous_error=new_error;//更新之前误差
	
	return PID_value; //返回速度控制值
}

//电机动作
void  motorsWrite(int speedL,int speedR)
{
	if(speedR > 0) 
	{
		//右轮前进
		DL_GPIO_setPins(MOTOR_BIN1_B20_PORT,MOTOR_BIN1_B20_PIN);
		DL_GPIO_clearPins(MOTOR_BIN2_B19_PORT,MOTOR_BIN2_B19_PIN);
		DL_TimerG_setCaptureCompareValue(PWM_MOTOR_INST,ABS(speedR),GPIO_PWM_MOTOR_C1_IDX);
	}
	else	
	{
		//右轮后退
		DL_GPIO_setPins(MOTOR_BIN2_B19_PORT,MOTOR_BIN2_B19_PIN);
		DL_GPIO_clearPins(MOTOR_BIN1_B20_PORT,MOTOR_BIN1_B20_PIN);
		DL_TimerG_setCaptureCompareValue(PWM_MOTOR_INST,ABS(speedR),GPIO_PWM_MOTOR_C1_IDX);
	}
	
	if(speedL > 0)
	{
		//左轮前进
		DL_GPIO_setPins(MOTOR_AIN2_B24_PORT,MOTOR_AIN2_B24_PIN);
		DL_GPIO_clearPins(MOTOR_AIN1_A02_PORT,MOTOR_AIN1_A02_PIN);
		DL_TimerG_setCaptureCompareValue(PWM_MOTOR_INST,ABS(speedL),GPIO_PWM_MOTOR_C0_IDX);
	}
	else
	{
		//左轮后退
		DL_GPIO_setPins(MOTOR_AIN1_A02_PORT,MOTOR_AIN1_A02_PIN);
		DL_GPIO_clearPins(MOTOR_AIN2_B24_PORT,MOTOR_AIN2_B24_PIN);
		DL_TimerG_setCaptureCompareValue(PWM_MOTOR_INST,ABS(speedL),GPIO_PWM_MOTOR_C0_IDX);
	}
}
void track_Set_PWM(int pwm_value)
{
	//基础速度+PID值
	int left_motor_speed = initial_motor_speed+pwm_value;
	int right_motor_speed = initial_motor_speed-pwm_value;
	
	
	if(left_motor_speed>9999) left_motor_speed=9999;
	else if(left_motor_speed<-9999) left_motor_speed=-9999;
	if(right_motor_speed>9999) right_motor_speed=9999;
	else if(right_motor_speed<-9999) right_motor_speed=-9999;
	
	motorsWrite(left_motor_speed,right_motor_speed);
}


int track_control(void)
{
	
	track_scan();
	track_Set_PWM(track_pid());
	if(L3 && L2 && L1 && M0 && R1 && R2 && R3) 
	{return 1;}
	else
	 {return 0;}
}

