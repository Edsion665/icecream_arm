#ifndef SERIAL_FRAME_H
#define SERIAL_FRAME_H

#include <stdint.h>
#include "../Hardware/Serial.h"
#include "MotorControl/motor_can.h"

typedef struct {
    float p;
    float v;
    float kp;
    float kd;
    float t;
} MitCmd_t;

typedef struct {
    uint16_t wrist_us;
    uint16_t gripper_us;
} ServoCmd_t;

extern const float g_mit_p_min[4];
extern const float g_mit_p_max[4];
extern const float g_mit_v_min[4];
extern const float g_mit_v_max[4];
extern const float g_mit_t_min[4];
extern const float g_mit_t_max[4];

int SerialFrame_ParseBinMit(const RpiBinFrame_t *frame, MitCmd_t cmds[4], ServoCmd_t *servo);

#endif
