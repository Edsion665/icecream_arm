#ifndef SERIAL_FRAME_H
#define SERIAL_FRAME_H

#include <stdint.h>

/*
 * 解析一行：DATA:a0,a1,a2,a3,a4,a5*XX
 * 前 6 个整数：a0..a3 为各轴「绝对角·度×100」（与 FB 一致，无 HOME 偏置）；a4、a5 预留。
 * 校验：从首字符 'D'（DATA:）到 '*' 之前逐字节 XOR，与 XX（两位十六进制）一致。
 */
int SerialFrame_ParseData(char *line, int16_t raw6[6]);

#endif
