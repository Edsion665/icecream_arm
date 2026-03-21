#ifndef FB_REPORT_TIMER_H
#define FB_REPORT_TIMER_H

#include <stdint.h>

void FB_ReportTimer_Init(void);
/* 轮询 TIM3 UIF：到周期返回 1 并清标志（不在 NVIC 里依赖 TIM3 中断） */
uint8_t FB_ReportTimer_TakePending(void);

/* 发一行 FB（绝对角·度×100，与 DATA 一致）；需已 Init 串口与 TIM3 */
void FB_Report_SendLine(void);
/* TakePending 为真则 SendLine；供插补循环内调用，避免主循环被阻塞时无 FB */
void FB_Report_ServicePending(void);

#endif
