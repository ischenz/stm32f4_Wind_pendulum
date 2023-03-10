/*
 ******************************************************
 STM32F407ZG KEIL例程 大越创新开发板

 202207081915	创建例程

 作者：			xc						V1.0
 ******************************************************
 */
#include "sys.h"
#include "led.h"
#include "key.h"
#include "delay.h"
#include "oled.h"
#include "timer.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "mpu6050.h"
#include "mpuexti.h"
#include "motor.h"
#include "control.h"
#include "pid.h"
#include "pidtool.h"
#include "usart.h"

uint8_t mode_num = 1;
extern int16_t Pwm_x, Pwm_y;
extern float Pitch,Roll,Yaw,kalmanFilter_Roll,kalmanFilter_Pitch;
extern uint8_t mode;
int32_t temp = 0;

int main(void)
{
//	u8 KEY_Val;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); 
	delay_init(168);
	protocol_init();
	uart_init(115200);
	LED_Init();
	OLED_Init();
	KEY_Init();
	OLED_ShowString(0,0,"Hello !!!",16,1);
	OLED_Refresh();
	//初始化motor
	Motor_Gpio_Init();
	Timer1_PWM_GPIO_Init(16, 1000);//约10KHz
	OLED_ShowString(0,0,"Motor init OK!!",16,1);
	OLED_Refresh();	
	delay_ms(500);
	OLED_Clear();
	//模式选择
	mode = switch_mode();
	//初始化MPU6050
	MPU_Init();
	DMP_Init(); 
	delay_ms(500);
	angle_calibration();//校准角度
	Init_Timer3();
	TIM_Cmd(TIM3,ENABLE);
	//pid初始化
	PID_param_init(&Roll_PID);
	PID_param_init(&Pitch_PID);
	set_pid_polarity(&Roll_PID, -1, -1, -1);
	set_pid_polarity(&Pitch_PID, 1, 1, 1);
	switch (mode)
	{
		case 1:{
			set_p_i_d(&Roll_PID,80,0,1000);
			set_p_i_d(&Pitch_PID,80,0,1000);
			break;
		}
		case 2:{
			set_p_i_d(&Roll_PID,80,0,1000);
			set_p_i_d(&Pitch_PID,80,0,1000);
			break;
		}
		case 3:{
			set_p_i_d(&Roll_PID,50,0,1000);
			set_p_i_d(&Pitch_PID,50,0,1000);
			break;
		}
		case 4:	{
			set_p_i_d(&Roll_PID,80,0,1000);
			set_p_i_d(&Pitch_PID,80,0,1000);
			break;
		}
		case 5:{
			set_p_i_d(&Roll_PID,50,0,1000);
			set_p_i_d(&Pitch_PID,50,0,1000);
			break;
		}
		default:{
			set_p_i_d(&Roll_PID,80,0,1000);
			set_p_i_d(&Pitch_PID,80,0,1000);
			break;
		}
	}
	
	
	pid_tool_send_param(&Pitch_PID ,CURVES_CH2);
	PID_TimerInit();
	//printf("chen:初始化PID成功 !\r\n");
	
	
	TIM_Cmd(TIM10, ENABLE);//开始PID运算
	set_computer_value(SEND_START_CMD, CURVES_CH1, NULL, 0); 	
	OLED_ShowString(0,0,"Pitch:",8,1);
	OLED_ShowString(0,10,"Roll:",8,1);

	OLED_ShowString(0,30,"P:",8,1);
	OLED_ShowString(0,40,"I:",8,1);
	OLED_ShowString(0,50,"D:",8,1);
	OLED_Refresh();
	
	while(1)
	{
		receiving_process();
//		KEY_Val = KEY_Scan();
//		if(KEY_Val)
//		{
//			if(KEY_Val == WKUP_PRES)
//				LED2 = 0;
//			if(KEY_Val == KEY1_PRES){
//				LED2 = 1;
//				
//				pid_tool_send_param(&Roll_PID ,CURVES_CH1);
//			}		
//		}

		limit_angle();
		OLED_ShowFNum(40,0,kalmanFilter_Roll,4,8,1);
		OLED_ShowFNum(40,10,kalmanFilter_Pitch,4,8,1);
		OLED_ShowFNum(40,30,Pitch_PID.ProportionConstant,4,8,1);
		OLED_ShowFNum(40,40,Pitch_PID.IntegralConstant,4,8,1);
		OLED_ShowFNum(40,50,Pitch_PID.DerivativeConstant,4,8,1);
		OLED_Refresh();
		//printf("%f,%f\r\n",Pitch,kalmanFilter_Pitch);
		
		temp = kalmanFilter_Roll;
		set_computer_value(SEND_FACT_CMD, CURVES_CH1, &temp, 1);
//		temp = kalmanFilter_Pitch;
//		set_computer_value(SEND_FACT_CMD, CURVES_CH2, &temp, 1);
	}
}
