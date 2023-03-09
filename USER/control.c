#include "control.h"
#include "motor.h"
#include "oled.h"
#include "Kalman.h"
#include "pid.h"
#include "pidtool.h"
#include "math.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "key.h"
#include "usart.h"
#include "motor.h"
#include "delay.h"

extern uint8_t mode_num;
extern float Pitch,Roll,Yaw;
float mechanical_error_Roll = 0, mechanical_error_Pitch = 0;
float use_pitch = 0, use_roll = 0;

uint8_t mode = 1;

PID_TypeDef Roll_PID,Pitch_PID;

#define pi 3.14159f
#define R 20.0f        // �뾶����
#define H 88.0f           // ��������ڵ�����ľ���
#define angle 	 40.0f // �ڶ��Ƕ�����



int16_t Pwm_x = 0, Pwm_y = 0;
extern float kalmanFilter_Roll,kalmanFilter_Pitch;


// x = A * sin(Omega * t + phi);y = A * sin(Omega * t + phi)
// �ı����A�ı�ֵ���Ըı�Ƕȣ�tan(theta) = A_y / A_x
// �ı�Ƶ��Omega�ı�ֵ���Եõ���ͬ��������ͼ��
// �ı���λphi�Ĳ�ֵ����ʹͼ����ֱ�ߡ���Բ��Բ֮��ı�

// ����һ�����Ȳ�����50cm��ֱ�߶�,���нϺõ��ظ���
void mode_1(void)
{
    const float cycle = 1574.0;     // ��������
    static uint32_t Movetime = 0; // ������ʱ��
	
    float Ax = 0.0;             // ���
    float Omega_t = 0.0;       // ����
    float x_roll, y_pitch;

    Movetime += 5;                       // ÿ5ms����һ��
    Ax = atan(R / H) * 57.2958f;          // �����������ʽΪ(360/2��*atan(R/H))
    Omega_t = 2.0f * pi * ((float)Movetime / cycle); // �����t
    x_roll = Ax * sin(Omega_t);           // x = A * sin(�� * t)
    y_pitch = 0;                         // y = 0
	Roll_PID.Target = x_roll;
	Pwm_x = PID_Calculate(&Roll_PID, kalmanFilter_Roll);
	
	Pitch_PID.Target = y_pitch;
	Pwm_y = PID_Calculate(&Pitch_PID, kalmanFilter_Pitch);
	
	PWM_Load(Pwm_x, Pwm_y);
}

// 15s����ɷ��ȿɿ� �İڶ�(30~60cm)
void mode_2(void)
{
	static uint8_t set_status = 0;
	static float x = 0;//�ɿط���
    const float cycle = 1574.0;          // ��������
    static u32 Movetime = 0.0; // ������ʱ��
    //float A = 0.0;             // ���
    float Omega_t = 0.0;       // ����
    float Ax, Ay = 0;
    float x_roll, y_pitch;

	if(set_status == 0){
		set_status = 1;
		x = Set_Length() -  3;
	}
	
    Movetime += 5;                        // ÿ5ms����һ��
    Ax = 57.2958 * atan(x / H);          // �����������ʽΪ(360/2��*atan(R/H))
    Omega_t = 2.0f * pi * ((float)Movetime / cycle); // �����t
    x_roll =  Ax * sin(Omega_t);         // x = A * sin(�� * t)
    y_pitch =  Ay;                        // y = 0
	Roll_PID.Target = x_roll;
	Pwm_x = PID_Calculate(&Roll_PID, kalmanFilter_Roll);
	Pitch_PID.Target = y_pitch;
	Pwm_y = PID_Calculate(&Pitch_PID, kalmanFilter_Pitch);
	PWM_Load(Pwm_x, Pwm_y);
}

//// 15s �ڰ������õķ���(�Ƕ�)�ڶ�(������ 20cm)
void mode_3(void)
{
	float x = 0;//�ɿط���
    const float cycle = 1574.0;          // ��������
    static u32 Movetime = 0.0; // ������ʱ��
    float A = 0.0;             // ���
    float Omega_t = 0.0;       // ����
    float Ax, Ay;
    float x_roll, y_pitch;

    Movetime += 5;                       // ÿ5ms����һ��
    A = 57.2958f * atan(x / H);         // �����������ʽΪ(360/2��*atan(R/H))
    Ax = A * cos(angle * pi / 180);      // ����x����ڷ���������ʽΪ(A*cos(�Ƕ�*��/180))
    Ay = A * sin(angle * pi / 180);      // ����y����ڷ���������ʽΪ(A*sin(�Ƕ�*��/180))
    Omega_t = 2 * pi * Movetime / cycle; // �����t
    x_roll = Ax * sin(Omega_t);          // x = Ax * sin(�� * t)
    y_pitch = Ay * sin(Omega_t);         // y = Ay * sin(�� * t)
	
	Roll_PID.Target = x_roll;
	Pitch_PID.Target = y_pitch;
	Pwm_x = PID_Calculate(&Roll_PID, Roll);
	Pwm_y = PID_Calculate(&Pitch_PID, Pitch);
	PWM_Load(Pwm_x, Pwm_y);
}

//// ����һ���Ƕ�(30��~45��),5s��ʹ�������ƶ��ﵽ��ֹ
void mode_4(void)
{
    Roll_PID.Target = 0;
	Pitch_PID.Target = 0;
	
	Pwm_x = PID_Calculate(&Roll_PID, kalmanFilter_Roll);
	Pwm_y = PID_Calculate(&Pitch_PID, kalmanFilter_Pitch);
	if(kalmanFilter_Roll > -0.5f && kalmanFilter_Roll < 0.5f ){
		PWM_Load(0,0);
	}
	else {
		PWM_Load(Pwm_x, Pwm_y);
	}
		

}

//// �ü�����ڵ��滭Բ(�뾶��15~35cm),̨�ȴ� 5s ���ܹ��� 5s �ڻָ�Բ���˶�
void mode_5(void)
{
	float x = 20;//�ɿط���
    const float cycle = 1574.0;          // ��������
    static u32 Movetime = 0.0; // ������ʱ��
    float A = 0.0;             // ���
    float Omega_t = 0.0;       // ����
    float x_roll, y_pitch;

    Movetime += 5;                       // ÿ5ms����һ��
    A = 57.2958f * atan(x / H);         // �����������ʽΪ(360/2��*atan(R/H))
    Omega_t = 2 * pi * Movetime / cycle; // �����t
    x_roll = A * sin(Omega_t);          // x = Ax * sin(�� * t)
    y_pitch = A * sin(Omega_t + pi/2);         // y = Ay * sin(�� * t)
	
	Roll_PID.Target = x_roll;
	Pitch_PID.Target = y_pitch;
	Pwm_x = PID_Calculate(&Roll_PID, Roll);
	Pwm_y = PID_Calculate(&Pitch_PID, Pitch);
	PWM_Load(Pwm_x, Pwm_y);
}

//// ���Ӳ���
//void mode_6(void)
//{
//}



void TIM1_UP_TIM10_IRQHandler(void)//5msһ��pid����
{
	if(TIM_GetITStatus(TIM10,TIM_IT_Update)==SET) //����ж�
	{
		//��ȡ�Ƕ�,�������˲�
		use_pitch = Pitch + mechanical_error_Pitch;
		use_roll = Roll + mechanical_error_Roll;
		//			kalmanFilter_Roll = kalmanFilter_A(Roll);
		//			kalmanFilter_Pitch = kalmanFilter_A(Pitch);
		kalmanFilter_Roll = use_roll;
		kalmanFilter_Pitch = use_pitch;
		//������Ӧģʽ
		switch(mode)
		{
			case 1:mode_1();break;
			case 2:mode_2();break;
			case 3:mode_3();break;
			case 4:mode_4();break;
			case 5:mode_5();break;
			default:break;
		}		
	}
	TIM_ClearITPendingBit(TIM10,TIM_IT_Update); //����жϱ�־λ	
}


uint8_t Set_Length(void)
{
	uint8_t set_value = 20,key_value = 0;
	OLED_Clear();
	OLED_ShowString(0,0,"Set length:",8,1);
	OLED_Refresh();
	while(1){
		key_value = KEY_Scan();
		if(key_value == KEY0_PRES){
			set_value +=5;
			if(set_value > 30){
				set_value = 20;
			}
			OLED_ShowNum(80,0,set_value,2,8,1);
			OLED_Refresh();
		}
		else if(key_value == KEY1_PRES){
			OLED_ShowString(10,0,"OK!!!",8,1);
			delay_ms(500);
			OLED_Refresh();
			return set_value;
		}
	}
}
uint8_t switch_mode(void)
{
	uint8_t	select_mode = 0, key_num = 0;
	OLED_Clear();
	OLED_ShowString(0, 0, "Select Mode:", 8, 1);
	OLED_ShowNum(60,20,select_mode,1,24,1);
	OLED_Refresh();
	while(1){
		key_num = KEY_Scan();
		if(key_num){
			if(key_num == KEY0_PRES){
				select_mode +=1;
			}
			else if(key_num == KEY1_PRES){
				OLED_ShowString(72,0,"OK!!!",8,1);
				OLED_Refresh();
				delay_ms(700);
				OLED_Clear();
				break;
			}
			if(select_mode > 4 ){
				select_mode = 0;
			}
			OLED_ShowNum(60,20,select_mode,1,24,1);
			OLED_Refresh();
		}
	}
	return select_mode;
}

void angle_calibration(void)
{
	uint8_t temp = 21 ,x = 0;
	float roll_sum = 0,pitch_sum = 0;
	OLED_ShowString(0,0,"Stand pendulum",8,1);
	OLED_ShowString(0,10,"Next push k1",8,1);
	OLED_Refresh();
	while(KEY_Scan() != KEY1_PRES){
	}
	
	OLED_ShowString(0,30,"Wait",8,1);
	OLED_Refresh();
	while(temp--){
		mpu_dmp_get_data(&Pitch,&Roll,&Yaw);
		roll_sum += Roll;
		pitch_sum += Pitch;
		x++;
		OLED_ShowChar(24+(x/4)*6,30,'.',8,1);
		OLED_Refresh();
		delay_ms(100);
	}
	mechanical_error_Roll = - (roll_sum/20);
	mechanical_error_Pitch = - (pitch_sum/20);	
	OLED_ShowString(0,30,"Calibration OK !!!",8,1);
	OLED_Refresh();
	delay_ms(500);
	OLED_Clear();
}

void limit_angle(void)
{
	if(kalmanFilter_Roll < -30 || kalmanFilter_Roll > 30){
		PWM_Load(0, 0);
	}
}
