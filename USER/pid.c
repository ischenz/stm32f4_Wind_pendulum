#include "pid.h"
#include "motor.h"
#include "pidtool.h"

/**
 * @brief       PID��ʼ��
 * @param       Ŀ��ֵ
 * @retval      ��
 */
void PID_param_init(PID_TypeDef *pid)
{
	pid->Target = 0;//Ŀ��ֵ
	pid->PID_out = 0;
    pid->ProportionConstant = 0;
    pid->IntegralConstant = 0;
	pid->DerivativeConstant = 0;
    pid->Err = 0.0f;
    pid->LastErr = 0.0f;
	pid->PenultErr = 0.0f;
    pid->Integral = 0.0f;//����ֵ
}

/**
 * @brief       pid�ջ����Ƽ���
 * @param       *PID��PID�ṹ�������ַ
 * @param       CurrentValue����ǰ����ֵ
 * @retval      �������ֵ
 */
int16_t PID_Calculate(PID_TypeDef *PID,float CurrentValue)
{
    PID->Err =  PID->Target - CurrentValue;
    PID->Integral += PID->Err;
	/*�����޷�*/
	if(PID->Integral > 200){
		PID->Integral = 200;
	}
	if(PID->Integral < -200){
		PID->Integral = -200;
	}
    PID->PID_out = PID->ProportionConstant * PID->Err 										/*����*/
				 + PID->IntegralConstant * PID->Integral  									/*����*/
			     + PID->DerivativeConstant * (PID->Err - PID->LastErr);						/*΢��*/

	PID->LastErr = PID->Err;
	PWM_Limit(&(PID->PID_out));
    return PID->PID_out;
}

void PID_TimerInit(void)
{
	uint32_t temp = 5000;
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
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);
	
	//TIM_Cmd(TIM10,ENABLE);
	set_computer_value(SEND_PERIOD_CMD, CURVES_CH1, &temp, 1);
	set_computer_value(SEND_PERIOD_CMD, CURVES_CH2, &temp, 1);
}

/**
  * @brief  ��ȡĿ��ֵ
  * @param  ��
	*	@note 	��
  * @retval Ŀ��ֵ
  */
float get_pid_target(PID_TypeDef *pid)
{
  return pid->Target;    // ���õ�ǰ��Ŀ��ֵ
}

void set_pid_target(PID_TypeDef *pid, float target)
{
	pid->Target = target;
}

/**
  * @brief  ���ñ��������֡�΢��ϵ��
  * @param  p������ϵ�� P
  * @param  i������ϵ�� i
  * @param  d��΢��ϵ�� d
	*	@note 	��
  * @retval ��
  */
void set_p_i_d(PID_TypeDef *pid, float p, float i, float d)
{
  	pid->ProportionConstant = p;    // ���ñ���ϵ�� P
	pid->IntegralConstant = i;    // ���û���ϵ�� I
	pid->DerivativeConstant = d;    // ����΢��ϵ�� D
}


