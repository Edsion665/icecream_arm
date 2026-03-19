#include "motor_test.h"
#include "../Coordinate/world_coord.h"
#include "motor_control.h"

/*================ 极限测试流程（硬保持版） ================*/
/*
   1) 电机4 -> 1.7rad
   2) 电机3 -> 2.0rad，同时 电机2 -> 2.4rad
   3) 达到目标后直接进入硬性保持
   4) 电机4 -> 0
   5) 电机3 -> 0
   6) 电机2 -> 0
   7) 最终继续硬性保持
*/
void Extreme_Test_Sequence(void)
{
    /* 初始化世界坐标 HOME（固定值）并同步目标到当前反馈，避免跳动 */
    WorldCoord_InitFixedHomeFromConfig();
    Sync_CurrentTargets_From_Feedback();

    /* 底座视为已就位，并从一开始就刚性保持 */
    Motor_Homed[0] = 1;

    /* 先把腕关节抬起 */
    Move_Motor_To_Rel(3, MOTOR4_PRESET_POS, 1, 0);

    /* 再同步抬起电机3和电机2 */
    Move_Two_Motors_To_Rels(2, MOTOR3_TEST_POS,
                            1, MOTOR2_TEST_POS,
                            1, 1);

    /* 到达展开位后，立即进入硬保持 */
    Extreme_Test_Hold_Active = 1;
    Hold_All_Rigid(FINAL_HOLD_MS);

    /* 电机4 回到 0 */
    Move_Motor_To_Rel(3, 0.0f, 1, 0);
    Hold_All_Rigid(RETURN_SETTLE_MS);

    /* 电机3 回到 0 */
    Move_Motor_To_Rel(2, 0.0f, 1, 0);
    Hold_All_Rigid(RETURN_SETTLE_MS);

    /* 电机2 回到 0 */
    Move_Motor_To_Rel(1, 0.0f, 1, 0);
    Hold_All_Rigid(RETURN_SETTLE_MS);

    /* 在最终姿态的基础上，四个电机再顺时针旋转半圈（180°，相对当前姿态的增量） */
    {
        float cur_rel0 = 0.0f, cur_rel1 = 0.0f, cur_rel2 = 0.0f, cur_rel3 = 0.0f;
        float r0, r1, r2, r3;
        /* 先读取当前位置并换算到世界坐标系下的“当前相对 HOME 角 ” */
        Read_All_Current_Positions();
        WorldCoord_RelFromAbs(0, Motor_States[0].pos, &cur_rel0);
        WorldCoord_RelFromAbs(1, Motor_States[1].pos, &cur_rel1);
        WorldCoord_RelFromAbs(2, Motor_States[2].pos, &cur_rel2);
        WorldCoord_RelFromAbs(3, Motor_States[3].pos, &cur_rel3);

        /* 在当前相对 HOME 的基础上，再增加半圈（+180° = +pi rad） */
        r0 = cur_rel0 + 3.1415926535f;
        r1 = cur_rel1 + 3.1415926535f;
        r2 = cur_rel2 + 3.1415926535f;
        r3 = cur_rel3 + 3.1415926535f;
        uint8_t mark[4] = {1, 1, 1, 1};
        Move_Four_Motors_FromFeedback_To_Rels(0, r0,
                                              1, r1,
                                              2, r2,
                                              3, r3,
                                              mark);
    }
}
