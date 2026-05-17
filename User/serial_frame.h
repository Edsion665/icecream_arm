#ifndef SERIAL_FRAME_H
#define SERIAL_FRAME_H

#include <stdint.h>
#include "../Hardware/Serial.h"
#include "MotorControl/motor_can.h"

/* v3 帧内字节偏移（与 pi2stm.md 一致） */
#define RPI_OFF_HEADER0         0u
#define RPI_OFF_MOTOR0          2u
#define RPI_OFF_WRIST_US        34u
#define RPI_OFF_GRIPPER_US      36u
#define RPI_OFF_STEPPER_DEG     38u   /* int16 BE，增量角 ° */
#define RPI_OFF_CONVEYOR_RUN    40u   /* 0=停，1=转 */
#define RPI_OFF_XOR             41u

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

typedef struct {
    int16_t  stepper_delta_deg;
    uint8_t  conveyor_run;
} AuxCmd_t;

extern const float g_mit_p_min[4];
extern const float g_mit_p_max[4];
extern const float g_mit_v_min[4];
extern const float g_mit_v_max[4];
extern const float g_mit_t_min[4];
extern const float g_mit_t_max[4];

uint8_t SerialFrame_VerifyXor(const uint8_t *frame, uint16_t len);
int SerialFrame_ParseBinMit(const RpiBinFrame_t *frame,
                            MitCmd_t cmds[4],
                            ServoCmd_t *servo,
                            AuxCmd_t *aux);

#endif
