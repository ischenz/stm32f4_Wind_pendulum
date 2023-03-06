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
#define R 20.0f        // 半径设置
#define H 88.0f           // 单摆万向节到地面的距离
#define angle 	 40.0f // 摆动角度设置

#define Roll_KP 60
#define Roll_KI 0
#define Roll_KD 0

#define Pitch_KP 0
#define Pitch_KI 0
#define Pitch_KD 0

// x = A * sin(Omega * t + phi);y = A * sin(Omega * t + phi)
// 改变振幅A的比值可以改变角度，tan(theta) = A_y / A_x
// 改变频率Omega的比值可以得到不同的李萨茹图形
// 改变相位phi的差值可以使图形在直线、椭圆、圆之间改变

// 画出一条长度不短于50cm的直线段,具有较好的重复性
void mode_1(void)
{
    const float cycle = 1574.0;     // 单摆周期
    static uint32_t Movetime = 0; // 运行总时长
	int16_t Pwm_x = 0, Pwm_y = 0;
    float Ax = 0.0;             // 振幅
    float Omega_t = 0.0;       // 周期
    float x_roll, y_pitch;

    Movetime += 5;                       // 每5ms运算一次
    Ax = 57.2958f * atan(R / H);          // 计算振幅，公式为(360/2π*atan(R/H))
    Omega_t = 2.0f * pi * ((float)Movetime / cycle); // 计算ωt
    x_roll = Ax * sin(Omega_t);           // x = A * sin(ω * t)
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

// 15s内完成幅度可控 的摆动(30~60cm)
void mode_2(void)
{
	uint16_t Pwm_x = 0, Pwm_y = 0;
	float x = 0;//可控幅度
    const float cycle = 1574.0;          // 单摆周期
    static u32 Movetime = 0.0; // 运行总时长
    //float A = 0.0;             // 振幅
    float Omega_t = 0.0;       // 周期
    float Ax, Ay = 0;
    float x_roll, y_pitch;

	x = Set_Length();
	
    Movetime += 5;                        // 每5ms运算一次
    Ax = 57.2958 * atan(x / H);          // 计算振幅，公式为(360/2π*atan(R/H))
    Omega_t = 2.0f * pi * ((float)Movetime / cycle); // 计算ωt
    x_roll = Ax * sin(Omega_t);           // x = A * sin(ω * t)
    y_pitch = Ay;                         // y = 0
	Roll_PID.Target = -x_roll;
	Pwm_x = PID_Calculate(&Roll_PID, Roll);
	
	Pitch_PID.Target = y_pitch;
	Pwm_y = PID_Calculate(&Pitch_PID, Pitch);
	PWM_Load(Pwm_x, Pwm_y);
}

//// 15s 内按照设置的方向(角度)摆动(不短于 20cm)
void mode_3(void)
{
	uint16_t Pwm_x = 0, Pwm_y = 0;
	float x = 0;//可控幅度
    const float cycle = 1574.0;          // 单摆周期
    static u32 Movetime = 0.0; // 运行总时长
    float A = 0.0;             // 振幅
    float Omega_t = 0.0;       // 周期
    float Ax, Ay;
    float x_roll, y_pitch;

    Movetime += 5;                       // 每5ms运算一次
    A = 57.2958f * atan(x / H);         // 计算振幅，公式为(360/2π*atan(R/H))
    Ax = A * cos(angle * pi / 180);      // 计算x方向摆幅分量，公式为(A*cos(角度*π/180))
    Ay = A * sin(angle * pi / 180);      // 计算y方向摆幅分量，公式为(A*sin(角度*π/180))
    Omega_t = 2 * pi * Movetime / cycle; // 计算ωt
    x_roll = Ax * sin(Omega_t);          // x = Ax * sin(ω * t)
    y_pitch = Ay * sin(Omega_t);         // y = Ay * sin(ω * t)
	
	Roll_PID.Target = -x_roll;
	Pitch_PID.Target = -y_pitch;
	Pwm_x = PID_Calculate(&Roll_PID, Roll);
	Pwm_y = PID_Calculate(&Pitch_PID, Pitch);
	PWM_Load(-100, Pwm_y);
}

//// 拉起一定角度(30°~45°),5s内使风力摆制动达到静止
void mode_4(void)
{
	uint16_t Pwm_x = 0, Pwm_y = 0;
    Roll_PID.Target = 0;
	Pitch_PID.Target = 0;
	Pwm_x = PID_Calculate(&Roll_PID, Roll);
	Pwm_y = PID_Calculate(&Pitch_PID, Pitch);
	PWM_Load(Pwm_x, Pwm_y);
}

//// 用激光笔在地面画圆(半径：15~35cm),台扇吹 5s 后能够在 5s 内恢复圆周运动
void mode_5(void)
{
	uint16_t Pwm_x = 0, Pwm_y = 0;
	float x = 0;//可控幅度
    const float cycle = 1574.0;          // 单摆周期
    static u32 Movetime = 0.0; // 运行总时长
    float A = 0.0;             // 振幅
    float Omega_t = 0.0;       // 周期
    float x_roll, y_pitch;

    Movetime += 5;                       // 每5ms运算一次
    A = 57.2958f * atan(x / H);         // 计算振幅，公式为(360/2π*atan(R/H))
    Omega_t = 2 * pi * Movetime / cycle; // 计算ωt
    x_roll = A * sin(Omega_t);          // x = Ax * sin(ω * t)
    y_pitch = A * sin(Omega_t + 2/pi);         // y = Ay * sin(ω * t)
	
	Roll_PID.Target = -x_roll;
	Pitch_PID.Target = y_pitch;
	Pwm_x = PID_Calculate(&Roll_PID, Roll);
	Pwm_y = PID_Calculate(&Pitch_PID, Pitch);
	PWM_Load(Pwm_x, Pwm_y);
}

//// 发挥部分
//void mode_6(void)
//{
//}



/**
 * @brief       Roll_PID初始化
 * @param       目标值
 * @retval      无
 */
void Roll_PID_Init(float TargetValue)
{
    Roll_PID.Target = TargetValue;//目标值
	Roll_PID.PID_out = 0;
    Roll_PID.ProportionConstant = Roll_KP;
    Roll_PID.IntegralConstant = Roll_KI;
	Roll_PID.DerivativeConstant = Roll_KD;
    Roll_PID.Err = 0.0f;
    Roll_PID.LastErr = 0.0f;
	Roll_PID.PenultErr = 0.0f;
    Roll_PID.Integral = 0.0f;//积分值
}


/**
 * @brief       Pich_PID初始化
 * @param       目标值
 * @retval      无
 */
void Pitch_PID_Init(float TargetValue)
{
    Pitch_PID.Target = TargetValue;//目标值
	Pitch_PID.PID_out = 0;
    Pitch_PID.ProportionConstant = Pitch_KP;
    Pitch_PID.IntegralConstant = Pitch_KI;
	Pitch_PID.DerivativeConstant = Pitch_KD;
    Pitch_PID.Err = 0.0f;
    Pitch_PID.LastErr = 0.0f;
	Pitch_PID.PenultErr = 0.0f;
    Pitch_PID.Integral = 0.0f;//积分值
}

/**
 * @brief       pid闭环控制计算
 * @param       *PID：PID结构体变量地址
 * @param       CurrentValue：当前测量值
 * @retval      期望输出值
 */
int16_t PID_Calculate(PID_TypeDef *PID,int16_t CurrentValue)
{
    PID->Err =  PID->Target - CurrentValue;
    PID->Integral += PID->Err;
	/*积分限幅*/
//	if(PID->Integral > 2000){
//		PID->Integral = 2000;
//	}
//	if(PID->Integral < -2000){
//		PID->Integral = -2000;
//	}
    PID->PID_out = PID->ProportionConstant * PID->Err 										/*比例*/
				 + PID->IntegralConstant * PID->Integral  									/*积分*/
			     + PID->DerivativeConstant * (PID->Err - PID->LastErr);						/*微分*/

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

void TIM1_UP_TIM10_IRQHandler(void)//5ms一次pid运算
{
	if(TIM_GetITStatus(TIM10,TIM_IT_Update)==SET) //溢出中断
	{
		//获取角度,卡尔曼滤波
		kalmanFilter_Roll = kalmanFilter_A(Roll);
		kalmanFilter_Pitch = kalmanFilter_A(Pitch);
		//进入相应模式
		switch(mode)
		{
			case 1:mode_1();break;
			case 2:mode_2();break;
			case 3:mode_3();break;
			case 4:mode_4();break;
			default:break;
		}	
	}
	TIM_ClearITPendingBit(TIM10,TIM_IT_Update); //清除中断标志位	
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

