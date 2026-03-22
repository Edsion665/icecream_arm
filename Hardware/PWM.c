#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"

/* 50Hz 舵机 PWM：20ms 周期，1us 分辨率，1500us=中位 */
#define PWM_TEST_PERIOD_US  20000u
#define PWM_TEST_CENTER_US  1500u

/**
  * PB0/PB1 PWM 测试：50Hz，1.5ms 脉宽（舵机中位），直接验证舵机引脚
  */
void PWM_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCStructure;
	RCC_ClocksTypeDef RCC_Clocks;
	uint32_t tim_clk, psc, arr;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);

	/* PB0=TIM3_CH3, PB1=TIM3_CH4 复用推挽 */
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	TIM_DeInit(TIM3);
	TIM_InternalClockConfig(TIM3);

	RCC_GetClocksFreq(&RCC_Clocks);
	tim_clk = RCC_Clocks.PCLK1_Frequency;
	if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1)
		tim_clk *= 2u;
	psc = tim_clk / 1000000u;
	if (psc < 1u) psc = 1u;
	psc -= 1u;
	arr = PWM_TEST_PERIOD_US - 1u;
	if (arr > 65535u) arr = 65535u;

	TIM_TimeBaseStructure.TIM_Prescaler         = (uint16_t)psc;
	TIM_TimeBaseStructure.TIM_CounterMode       = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period            = (uint16_t)arr;
	TIM_TimeBaseStructure.TIM_ClockDivision     = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	TIM_OCStructInit(&TIM_OCStructure);
	TIM_OCStructure.TIM_OCMode      = TIM_OCMode_PWM1;
	TIM_OCStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCStructure.TIM_OCPolarity  = TIM_OCPolarity_High;
	TIM_OCStructure.TIM_Pulse       = PWM_TEST_CENTER_US;
	TIM_OC3Init(TIM3, &TIM_OCStructure);
	TIM_OC4Init(TIM3, &TIM_OCStructure);

	TIM_SetCompare3(TIM3, PWM_TEST_CENTER_US);
	TIM_SetCompare4(TIM3, PWM_TEST_CENTER_US);

	TIM_ClearFlag(TIM3, TIM_FLAG_Update);
	TIM_SetCounter(TIM3, 0);
	TIM_Cmd(TIM3, ENABLE);
}

void PWM_SetCompare1(uint16_t us)
{
	TIM_SetCompare3(TIM3, us);
}

void PWM_SetCompare2(uint16_t us)
{
	TIM_SetCompare4(TIM3, us);
}

/* Motor.c 直流电机用：CCR 占空比 0~100（需单独配置对应通道时有效） */
void PWM_SetCompare3(uint16_t Compare)
{
	(void)Compare;  /* PB0/PB1 仅 CH3/CH4，无额外通道 */
}
