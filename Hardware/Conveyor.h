#ifndef __CONVEYOR_H
#define __CONVEYOR_H

#include <stdint.h>

/*
 * 传送带开关量控制
 * - PB6：推挽输出；高电平=旋转，低电平=停止
 * - 上电 Init 后默认停止（低）
 */

void Conveyor_Init(void);

/* 下行目标：0=停，1=转（由 Conveyor_Update 应用到 PB6） */
void Conveyor_SetTargetRun(uint8_t run);
void Conveyor_Update(void);

/* 立即写 GPIO；run：0=停止(低)，非 0=旋转(高) */
void Conveyor_SetRun(uint8_t run);
uint8_t Conveyor_GetRun(void);

#endif /* __CONVEYOR_H */
