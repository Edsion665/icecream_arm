#ifndef FB_REPORT_TIMER_H
#define FB_REPORT_TIMER_H

#include <stdint.h>

void FB_ReportTimer_Init(void);
/* 轮询 TIM3 UIF：到周期返回 1 并清标志（不在 NVIC 里依赖 TIM3 中断） */
uint8_t FB_ReportTimer_TakePending(void);

#endif
