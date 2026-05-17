/**
 * 传送带：PB6 数字输出控制启停
 *
 * STM32F103：PB6 默认可作 GPIO（复用为 I2C1_SCL 时须未启用 I2C1）。
 * 本工程将 PB6 配置为推挽输出，驱动传送带使能/继电器类接口。
 */
#include "Conveyor.h"
#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"

/*================ 引脚配置（仅在本文件修改） ================*/
#define CONVEYOR_GPIO_PORT          GPIOB
#define CONVEYOR_GPIO_PIN           GPIO_Pin_6
#define CONVEYOR_GPIO_RCC           RCC_APB2Periph_GPIOB

/* 1=旋转，0=停止 */
#define CONVEYOR_LEVEL_RUN          Bit_SET
#define CONVEYOR_LEVEL_STOP         Bit_RESET

static uint8_t s_run;
static uint8_t s_target_run;

static void conveyor_apply_level(uint8_t run)
{
    if (run) {
        GPIO_WriteBit(CONVEYOR_GPIO_PORT, CONVEYOR_GPIO_PIN, CONVEYOR_LEVEL_RUN);
    } else {
        GPIO_WriteBit(CONVEYOR_GPIO_PORT, CONVEYOR_GPIO_PIN, CONVEYOR_LEVEL_STOP);
    }
}

void Conveyor_Init(void)
{
    GPIO_InitTypeDef gpio;

    s_run = 0u;
    s_target_run = 0u;

    RCC_APB2PeriphClockCmd(CONVEYOR_GPIO_RCC, ENABLE);

    gpio.GPIO_Pin   = CONVEYOR_GPIO_PIN;
    gpio.GPIO_Mode  = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(CONVEYOR_GPIO_PORT, &gpio);

    conveyor_apply_level(0u);
}

void Conveyor_SetTargetRun(uint8_t run)
{
    s_target_run = run ? 1u : 0u;
}

void Conveyor_Update(void)
{
    if (s_run != s_target_run) {
        Conveyor_SetRun(s_target_run);
    }
}

void Conveyor_SetRun(uint8_t run)
{
    s_run = run ? 1u : 0u;
    conveyor_apply_level(s_run);
}

uint8_t Conveyor_GetRun(void)
{
    return s_run;
}
