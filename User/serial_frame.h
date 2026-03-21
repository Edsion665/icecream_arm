#ifndef SERIAL_FRAME_H
#define SERIAL_FRAME_H

#include <stdint.h>

/*
 * 解析一行：DATA:a0,a1,a2,a3,a4,a5*XX
 * 前 6 个整数：a0..a3 为各轴「绝对角·度×100」（与 FB 一致，无 HOME 偏置）；a4、a5 预留。
 *
 * 校验（与大小写无关：DATA/data 各字母成对异或 0x20 相互抵消）：
 *   从行首第一个字符起至 '*' 之前（不含 '*'）逐字节 unsigned XOR，
 *   结果低 8 位写成两位十六进制 XX（大写小写均可 sscanf）。
 *
 * 例：DATA:8000,0,-2000,2500,0,0  →  XOR = 0x16  →  帧尾 *16 （不是 *39）
 */
int SerialFrame_ParseData(char *line, int16_t raw6[6]);

#endif
