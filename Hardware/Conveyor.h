#ifndef __CONVEYOR_H
#define __CONVEYOR_H

#include <stdint.h>

/*
 * 传送带开关量控制
 * - PB6：推挽输出；高电平=旋转，低电平=停止
 * - 上电 Init 后默认停止（低）
 */

void Conveyor_Init(void);

/* run：0=停止(低)，非 0=旋转(高) */
void Conveyor_SetRun(uint8_t run);

/* 当前输出是否为“旋转”(高电平) */
uint8_t Conveyor_GetRun(void);

#endif /* __CONVEYOR_H */
