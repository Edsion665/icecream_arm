#ifndef FB_REPORT_TIMER_H
#define FB_REPORT_TIMER_H

#include <stdint.h>

void FB_ReportTimer_Init(void);
/* 轮询 TIM2 UIF：到周期返回 1 并清标志（不在 NVIC 里依赖 TIM2 中断） */
uint8_t FB_ReportTimer_TakePending(void);

/* 发 39 字节二进制 FB；周期 = motor_config.h 中 FB_REPORT_HZ（MIT_HEX=1 时为 25Hz） */
void FB_Report_SendLine(void);
void FB_Report_ServicePending(void);

#endif
