#ifndef FB_REPORT_TIMER_H
#define FB_REPORT_TIMER_H

#include <stdint.h>
#include "stm32f10x.h"

/*
 * 周期 FB 上行唯一定时源：TIM6（基本定时器，无引脚，主循环轮询 UIF）。
 * 频率由 motor_config.h 的 FB_REPORT_HZ 决定（MIT_HEX_MODE=1 时为 25Hz）。
 * 勿用 TIM2（步进 STEP）/ TIM5（F103C8 无 TIM5）。
 */
#define FB_REPORT_TIM_PERIPH          TIM6
#define FB_REPORT_TIM_PERIPH_RCC      RCC_APB1Periph_TIM6

void FB_ReportTimer_Init(void);
uint8_t FB_ReportTimer_TakePending(void);

void FB_Report_SendLine(void);
void FB_Report_ServicePending(void);

#endif
