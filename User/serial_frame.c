#include "serial_frame.h"
#include "MotorControl/motor_config.h"

const float g_mit_p_min[4] = { MIT_P_MIN, MIT_P_MIN, MIT_P_MIN, MIT_P_MIN };
const float g_mit_p_max[4] = { MIT_P_MAX, MIT_P_MAX, MIT_P_MAX, MIT_P_MAX };
const float g_mit_v_min[4] = { MIT_V_MIN, MIT_V_MIN, MIT_V_MIN, MIT_V_MIN };
const float g_mit_v_max[4] = { MIT_V_MAX, MIT_V_MAX, MIT_V_MAX, MIT_V_MAX };
const float g_mit_t_min[4] = { MIT_T_MIN, MIT_T_MIN, MIT_T_MIN, MIT_T_MIN };
const float g_mit_t_max[4] = { MIT_T_MAX, MIT_T_MAX, MIT_T_MAX, MIT_T_MAX };

int SerialFrame_ParseBinMit(const RpiBinFrame_t *frame, MitCmd_t cmds[4], ServoCmd_t *servo)
{
    const uint8_t *d = frame->data;
    uint8_t xor_calc = 0;
    int i;

    if (d[0] != 0xAAu || d[1] != 0x55u) {
        return 0;
    }

    for (i = 0; i < (int)(RPI_BIN_FRAME_LEN - 1); i++) {
        xor_calc ^= d[i];
    }
    if (xor_calc != d[RPI_BIN_FRAME_LEN - 1]) {
        return 0;
    }

    for (i = 0; i < 4; i++) {
        const uint8_t *b = d + 2 + i * 8;

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
        servo->wrist_us   = ((uint16_t)d[34] << 8) | d[35];
        servo->gripper_us = ((uint16_t)d[36] << 8) | d[37];
    }

    return 1;
}
