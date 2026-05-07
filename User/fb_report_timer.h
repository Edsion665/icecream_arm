#ifndef FB_REPORT_TIMER_H
#define FB_REPORT_TIMER_H

#include <stdint.h>

void FB_ReportTimer_Init(void);
/* 轮询 TIM2 UIF：到周期返回 1 并清标志（不在 NVIC 里依赖 TIM2 中断） */
uint8_t FB_ReportTimer_TakePending(void);

/* 发 39 字节二进制 FB（与 6fee739 一致）；周期=FB_REPORT_HZ */
void FB_Report_SendLine(void);
/* TakePending 为真时可选 SendLine（MOTOR_DEBUG_LOG_ENABLE）；插补循环内亦调用，避免主循环阻塞时无 FB */
void FB_Report_ServicePending(void);

#endif
