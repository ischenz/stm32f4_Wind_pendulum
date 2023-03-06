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
#include "usart.h"

uint8_t mode_num = 1;
extern int16_t Pwm_x, Pwm_y;
extern float Pitch,Roll,Yaw,kalmanFilter_Roll,kalmanFilter_Pitch,\
			 mechanical_error_Roll,mechanical_error_Pitch;
extern uint8_t mode;

int main(void)
{
	u8 KEY_Val;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); 
	delay_init(168);
	uart_init(115200);
	LED_Init();
	OLED_Init();
	KEY_Init();
	Init_Timer3();//控制LED
	TIM_Cmd(TIM3,ENABLE);
	OLED_ShowString(0,0,"Hello !!!",16,1);
	//printf("Hello chen!!! \r\n \r\n");
	OLED_Refresh();
	//printf("chen:LED初始化成功! \r\n");
	//初始化motor
	Motor_Gpio_Init();
	Timer1_PWM_GPIO_Init(16, 1000);//约10KHz
	OLED_ShowString(0,0,"Motor init OK!!",16,1);
	OLED_Refresh();	
	delay_ms(500);
	OLED_Clear();
	//printf("chen:电机初始化成功! \r\n");
	//模式选择
	mode = switch_mode();
	//printf("chen:选择模式%d! \r\n", mode);
	//初始化MPU6050
	delay_ms(500);
	OLED_Clear();
	MPU_Init();
	DMP_Init(); 
	MPU6050_EXTI_Init();//中断读取角度数据
	//printf("chen:MPU6050初始化成功! \r\n");
	
	//校准角度
	angle_calibration();
	//printf("chen:角度初始化成功! \r\n");
	//pid初始化
	Roll_PID_Init(0);
	Pitch_PID_Init(0);
	PID_TimerInit();
	//printf("chen:初始化PID成功 !\r\n");
	
	
	TIM_Cmd(TIM10, ENABLE);
	
	OLED_ShowString(0,0,"Pitch:",8,1);
	OLED_ShowString(0,10,"Roll:",8,1);

	OLED_ShowString(0,30,"PWM_x:",8,1);
	OLED_ShowString(0,40,"PWM_y:",8,1);
	OLED_ShowString(0,50,"Tim:",8,1);
	OLED_Refresh();
	
	while(1)
	{
		printf("%f,%f\r\n",kalmanFilter_Pitch*100,kalmanFilter_Roll*100-2000);
		OLED_ShowFNum(40,0,kalmanFilter_Roll,4,8,1);
		OLED_ShowFNum(40,10,kalmanFilter_Pitch,4,8,1);
		OLED_Refresh();
		KEY_Val = KEY_Scan();
		if(KEY_Val)
		{
			if(KEY_Val == WKUP_PRES)
				LED2 = 0;
			if(KEY_Val == KEY1_PRES)		
				LED2 = 1;
		}

	}
}
