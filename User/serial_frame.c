#include "serial_frame.h"
#include "MotorControl/motor_config.h"

const float g_mit_p_min[4] = { MIT_P_MIN, MIT_P_MIN, MIT_P_MIN, MIT_P_MIN };
const float g_mit_p_max[4] = { MIT_P_MAX, MIT_P_MAX, MIT_P_MAX, MIT_P_MAX };
const float g_mit_v_min[4] = { MIT_V_MIN, MIT_V_MIN, MIT_V_MIN, MIT_V_MIN };
const float g_mit_v_max[4] = { MIT_V_MAX, MIT_V_MAX, MIT_V_MAX, MIT_V_MAX };
const float g_mit_t_min[4] = { MIT_T_MIN, MIT_T_MIN, MIT_T_MIN, MIT_T_MIN };
const float g_mit_t_max[4] = { MIT_T_MAX, MIT_T_MAX, MIT_T_MAX, MIT_T_MAX };

uint8_t SerialFrame_VerifyXor(const uint8_t *frame, uint16_t len)
{
    uint8_t xor_calc = 0;
    uint16_t i;

    if (frame == 0 || len < 2u) {
        return 0u;
    }
    for (i = 0; i < (uint16_t)(len - 1u); i++) {
        xor_calc ^= frame[i];
    }
    return (xor_calc == frame[len - 1u]) ? 1u : 0u;
}

int SerialFrame_ParseBinMit(const RpiBinFrame_t *frame,
                            MitCmd_t cmds[4],
                            ServoCmd_t *servo,
                            AuxCmd_t *aux)
{
    const uint8_t *d;
    int i;
    int16_t stepper_deg;

    if (frame == 0 || cmds == 0) {
        return 0;
    }

    d = frame->data;
    if (d[RPI_OFF_HEADER0] != 0xAAu || d[1] != 0x55u) {
        return 0;
    }
    if (!SerialFrame_VerifyXor(d, RPI_BIN_FRAME_LEN)) {
        return 0;
    }

    for (i = 0; i < 4; i++) {
        const uint8_t *b = d + RPI_OFF_MOTOR0 + i * 8;

        uint16_t p_i  = ((uint16_t)b[0] << 8) | b[1];
        uint16_t v_i  = ((uint16_t)b[2] << 4) | (b[3] >> 4);
        uint16_t kp_i = ((uint16_t)(b[3] & 0x0Fu) << 8) | b[4];
        uint16_t kd_i = ((uint16_t)b[5] << 4) | (b[6] >> 4);
        uint16_t t_i  = ((uint16_t)(b[6] & 0x0Fu) << 8) | b[7];

        cmds[i].p  = uint_to_float((int)p_i,  g_mit_p_min[i], g_mit_p_max[i], 16);
        cmds[i].v  = uint_to_float((int)v_i,  g_mit_v_min[i], g_mit_v_max[i], 12);
        cmds[i].kp = uint_to_float((int)kp_i, 0.0f, 500.0f, 12);
        cmds[i].kd = uint_to_float((int)kd_i, 0.0f, 5.0f, 12);
        cmds[i].t  = uint_to_float((int)t_i,  g_mit_t_min[i], g_mit_t_max[i], 12);
    }

    if (servo != 0) {
        servo->wrist_us   = ((uint16_t)d[RPI_OFF_WRIST_US] << 8) | d[RPI_OFF_WRIST_US + 1];
        servo->gripper_us = ((uint16_t)d[RPI_OFF_GRIPPER_US] << 8) | d[RPI_OFF_GRIPPER_US + 1];
    }

    if (aux != 0) {
        stepper_deg = (int16_t)(((uint16_t)d[RPI_OFF_STEPPER_DEG] << 8) |
                                d[RPI_OFF_STEPPER_DEG + 1]);
        if (stepper_deg > 180) {
            stepper_deg = 180;
        } else if (stepper_deg < -180) {
            stepper_deg = -180;
        }
        aux->stepper_delta_deg = stepper_deg;
        aux->conveyor_run = (d[RPI_OFF_CONVEYOR_RUN] != 0u) ? 1u : 0u;
    }

    return 1;
}
