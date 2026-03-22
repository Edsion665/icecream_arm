#ifndef __PWM_H
#define __PWM_H

void PWM_Init(void);              /* PB0/PB1 50Hz PWM 测试 */
void PWM_SetCompare1(uint16_t us); /* CH1 脉宽 us */
void PWM_SetCompare2(uint16_t us); /* CH2 脉宽 us */
void PWM_SetCompare3(uint16_t Compare); /* CH3，Motor.c 直流电机用 0~100 */

#endif
