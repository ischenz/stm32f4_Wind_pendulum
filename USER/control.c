#include "control.h"
#include "math.h"
#include "mpu6050.h"
#include "motor.h"
#include "key.h"
#include "delay.h"

extern uint8_t mode_num;
extern float Pitch,Roll,Yaw;
float kalmanFilter_Roll,kalmanFilter_Pitch;

uint8_t mode = 1;

PID_TypeDef Roll_PID,Pitch_PID;

#define pi 3.14159f
#define R 20.0f        // �뾶����
#define H 88.0f           // ��������ڵ�����ľ���
#define angle 	 40.0f // �ڶ��Ƕ�����

#define Roll_KP 60
#define Roll_KI 0
#define Roll_KD 0

#define Pitch_KP 0
#define Pitch_KI 0
#define Pitch_KD 0

// x = A * sin(Omega * t + phi);y = A * sin(Omega * t + phi)
// �ı����A�ı�ֵ���Ըı�Ƕȣ�tan(theta) = A_y / A_x
// �ı�Ƶ��Omega�ı�ֵ���Եõ���ͬ��������ͼ��
// �ı���λphi�Ĳ�ֵ����ʹͼ����ֱ�ߡ���Բ��Բ֮��ı�

// ����һ�����Ȳ�����50cm��ֱ�߶�,���нϺõ��ظ���
void mode_1(void)
{
    const float cycle = 1574.0;     // ��������
    static uint32_t Movetime = 0; // ������ʱ��
	int16_t Pwm_x = 0, Pwm_y = 0;
    float Ax = 0.0;             // ���
    float Omega_t = 0.0;       // ����
    float x_roll, y_pitch;

    Movetime += 5;                       // ÿ5ms����һ��
    Ax = 57.2958f * atan(R / H);          // �����������ʽΪ(360/2��*atan(R/H))
    Omega_t = 2.0f * pi * ((float)Movetime / cycle); // �����t
    x_roll = Ax * sin(Omega_t);           // x = A * sin(�� * t)
    y_pitch = 0;                         // y = 0
	Roll_PID.Target = x_roll;
	Pwm_x = PID_Calculate(&Roll_PID, kalmanFilter_Roll);
	
	Pitch_PID.Target = y_pitch;
	Pwm_y = PID_Calculate(&Pitch_PID, kalmanFilter_Pitch);
	
	PWM_Load(Pwm_x, Pwm_y);
	
	OLED_ShowSNum(40,30,Pwm_x,4,8,1);
	OLED_ShowSNum(40,40,Pwm_y,4,8,1);
	OLED_ShowNum(40,50,Movetime,4,8,1);
}

// 15s����ɷ��ȿɿ� �İڶ�(30~60cm)
void mode_2(void)
{
	uint16_t Pwm_x = 0, Pwm_y = 0;
	float x = 0;//�ɿط���
    const float cycle = 1574.0;          // ��������
    static u32 Movetime = 0.0; // ������ʱ��
    //float A = 0.0;             // ���
    float Omega_t = 0.0;       // ����
    float Ax, Ay = 0;
    float x_roll, y_pitch;

	x = Set_Length();
	
    Movetime += 5;                        // ÿ5ms����һ��
    Ax = 57.2958 * atan(x / H);          // �����������ʽΪ(360/2��*atan(R/H))
    Omega_t = 2.0f * pi * ((float)Movetime / cycle); // �����t
    x_roll = Ax * sin(Omega_t);           // x = A * sin(�� * t)
    y_pitch = Ay;                         // y = 0
	Roll_PID.Target = -x_roll;
	Pwm_x = PID_Calculate(&Roll_PID, Roll);
	
	Pitch_PID.Target = y_pitch;
	Pwm_y = PID_Calculate(&Pitch_PID, Pitch);
	PWM_Load(Pwm_x, Pwm_y);
}

//// 15s �ڰ������õķ���(�Ƕ�)�ڶ�(������ 20cm)
void mode_3(void)
{
	uint16_t Pwm_x = 0, Pwm_y = 0;
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
	
	Roll_PID.Target = -x_roll;
	Pitch_PID.Target = -y_pitch;
	Pwm_x = PID_Calculate(&Roll_PID, Roll);
	Pwm_y = PID_Calculate(&Pitch_PID, Pitch);
	PWM_Load(-100, Pwm_y);
}

//// ����һ���Ƕ�(30��~45��),5s��ʹ�������ƶ��ﵽ��ֹ
void mode_4(void)
{
	uint16_t Pwm_x = 0, Pwm_y = 0;
    Roll_PID.Target = 0;
	Pitch_PID.Target = 0;
	Pwm_x = PID_Calculate(&Roll_PID, Roll);
	Pwm_y = PID_Calculate(&Pitch_PID, Pitch);
	PWM_Load(Pwm_x, Pwm_y);
}

//// �ü�����ڵ��滭Բ(�뾶��15~35cm),̨�ȴ� 5s ���ܹ��� 5s �ڻָ�Բ���˶�
void mode_5(void)
{
	uint16_t Pwm_x = 0, Pwm_y = 0;
	float x = 0;//�ɿط���
    const float cycle = 1574.0;          // ��������
    static u32 Movetime = 0.0; // ������ʱ��
    float A = 0.0;             // ���
    float Omega_t = 0.0;       // ����
    float x_roll, y_pitch;

    Movetime += 5;                       // ÿ5ms����һ��
    A = 57.2958f * atan(x / H);         // �����������ʽΪ(360/2��*atan(R/H))
    Omega_t = 2 * pi * Movetime / cycle; // �����t
    x_roll = A * sin(Omega_t);          // x = Ax * sin(�� * t)
    y_pitch = A * sin(Omega_t + 2/pi);         // y = Ay * sin(�� * t)
	
	Roll_PID.Target = -x_roll;
	Pitch_PID.Target = y_pitch;
	Pwm_x = PID_Calculate(&Roll_PID, Roll);
	Pwm_y = PID_Calculate(&Pitch_PID, Pitch);
	PWM_Load(Pwm_x, Pwm_y);
}

//// ���Ӳ���
//void mode_6(void)
//{
//}



/**
 * @brief       Roll_PID��ʼ��
 * @param       Ŀ��ֵ
 * @retval      ��
 */
void Roll_PID_Init(float TargetValue)
{
    Roll_PID.Target = TargetValue;//Ŀ��ֵ
	Roll_PID.PID_out = 0;
    Roll_PID.ProportionConstant = Roll_KP;
    Roll_PID.IntegralConstant = Roll_KI;
	Roll_PID.DerivativeConstant = Roll_KD;
    Roll_PID.Err = 0.0f;
    Roll_PID.LastErr = 0.0f;
	Roll_PID.PenultErr = 0.0f;
    Roll_PID.Integral = 0.0f;//����ֵ
}


/**
 * @brief       Pich_PID��ʼ��
 * @param       Ŀ��ֵ
 * @retval      ��
 */
void Pitch_PID_Init(float TargetValue)
{
    Pitch_PID.Target = TargetValue;//Ŀ��ֵ
	Pitch_PID.PID_out = 0;
    Pitch_PID.ProportionConstant = Pitch_KP;
    Pitch_PID.IntegralConstant = Pitch_KI;
	Pitch_PID.DerivativeConstant = Pitch_KD;
    Pitch_PID.Err = 0.0f;
    Pitch_PID.LastErr = 0.0f;
	Pitch_PID.PenultErr = 0.0f;
    Pitch_PID.Integral = 0.0f;//����ֵ
}

/**
 * @brief       pid�ջ����Ƽ���
 * @param       *PID��PID�ṹ�������ַ
 * @param       CurrentValue����ǰ����ֵ
 * @retval      �������ֵ
 */
int16_t PID_Calculate(PID_TypeDef *PID,int16_t CurrentValue)
{
    PID->Err =  PID->Target - CurrentValue;
    PID->Integral += PID->Err;
	/*�����޷�*/
//	if(PID->Integral > 2000){
//		PID->Integral = 2000;
//	}
//	if(PID->Integral < -2000){
//		PID->Integral = -2000;
//	}
    PID->PID_out = PID->ProportionConstant * PID->Err 										/*����*/
				 + PID->IntegralConstant * PID->Integral  									/*����*/
			     + PID->DerivativeConstant * (PID->Err - PID->LastErr);						/*΢��*/

	PID->LastErr = PID->Err;
	PWM_Limit(&(PID->PID_out));
    return PID->PID_out;
}

void PID_TimerInit(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10,ENABLE);
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_TimeBaseInitStruct.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseInitStruct.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_Period=5000-1;//5ms
	TIM_TimeBaseInitStruct.TIM_Prescaler=167;
	TIM_TimeBaseInit(TIM10,&TIM_TimeBaseInitStruct);	
	
	TIM_ITConfig(TIM10,TIM_IT_Update,ENABLE);
	
	NVIC_InitTypeDef NVIC_InitStruct;
	NVIC_InitStruct.NVIC_IRQChannel = TIM1_UP_TIM10_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);
	
	//TIM_Cmd(TIM10,ENABLE);
	
}

void TIM1_UP_TIM10_IRQHandler(void)//5msһ��pid����
{
	if(TIM_GetITStatus(TIM10,TIM_IT_Update)==SET) //����ж�
	{
		//��ȡ�Ƕ�,�������˲�
		kalmanFilter_Roll = kalmanFilter_A(Roll);
		kalmanFilter_Pitch = kalmanFilter_A(Pitch);
		//������Ӧģʽ
		switch(mode)
		{
			case 1:mode_1();break;
			case 2:mode_2();break;
			case 3:mode_3();break;
			case 4:mode_4();break;
			default:break;
		}	
	}
	TIM_ClearITPendingBit(TIM10,TIM_IT_Update); //����жϱ�־λ	
}


uint8_t Set_Length(void)
{
	uint8_t set_value = 30;
	OLED_Clear();
	OLED_ShowString(0,0,"Set length:",8,1);
	while(1){
		if(KEY_Scan(0) == KEY0_PRES){
			set_value +=5;
			if(set_value > 50){
				set_value = 30;
			}
		}
		else if(KEY_Scan(0) == KEY1_PRES){
			OLED_ShowString(10,0,"OK!!!",8,1);
			delay_ms(500);
			OLED_Refresh();
			return set_value;
		}
		OLED_ShowNum(80,0,set_value,2,8,1);
		OLED_Refresh();
	}
}
uint8_t switch_mode(void)
{
	uint8_t	select_mode = 1;
	OLED_Clear();
	OLED_ShowString(0, 0, "Mode:", 8, 1);
	while(1){
		if(KEY_Scan(0) == KEY0_PRES){
			select_mode +=1;
		}
		else if(KEY_Scan(0) == KEY1_PRES){
			OLED_ShowString(10,0,"OK!!!",8,1);
			delay_ms(500);
			OLED_Refresh();
			break;
		}
		if(select_mode > 4 ){
			select_mode = 1;

		}
		OLED_ShowNum(30,0,select_mode,1,8,1);
		OLED_Refresh();
	}
	return select_mode;
}

