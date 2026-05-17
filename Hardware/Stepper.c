/**
 * 步进电机：PB7=STEP(TIM4_CH2 PWM)，PA1=DIR(GPIO)
 * 共阴极；4 细分，800 脉冲/转
 */
#include "Stepper.h"
#include "Delay.h"
#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"

/*================ 硬件引脚 / 定时器 ================*/
#define STEPPER_STEP_GPIO_PORT      GPIOB
#define STEPPER_STEP_GPIO_PIN       GPIO_Pin_7
#define STEPPER_DIR_GPIO_PORT       GPIOA
#define STEPPER_DIR_GPIO_PIN        GPIO_Pin_1

#define STEPPER_PWM_TIM             TIM4
#define STEPPER_PWM_TIM_RCC         RCC_APB1Periph_TIM4
#define STEPPER_PWM_CHANNEL         TIM_Channel_2

/*================ 脉冲频率限制 ================*/
#define STEPPER_STEP_HZ_MIN         1u
#define STEPPER_STEP_HZ_MAX         50000u

/*================ 运行时状态 ================*/
static uint32_t s_step_hz;
static uint8_t  s_dir_forward = 1u;
static uint8_t  s_tim_inited;
static int32_t  s_pos_steps;
static float    s_target_delta_deg;
static uint8_t  s_delta_pending;

typedef enum {
    STEPPER_ST_IDLE = 0,
    STEPPER_ST_DIR_WAIT,
    STEPPER_ST_RUNNING,
} StepperMoveState_t;

static StepperMoveState_t s_move_state;
static uint32_t           s_move_pulses_total;
static uint32_t           s_move_pulses_done;
static uint8_t            s_move_forward;
static uint16_t           s_dir_wait_ms;
static uint16_t           s_last_cnt;

/*================ 内部工具 ================*/
static float stepper_fabsf(float x)
{
    return (x < 0.0f) ? -x : x;
}

static uint32_t stepper_deg_to_pulses(float deg_mag)
{
    float pulses_f;

    pulses_f = deg_mag * (float)STEPPER_PULSES_PER_REV / 360.0f;
    if (pulses_f < 0.5f) {
        return 0u;
    }
    return (uint32_t)(pulses_f + 0.5f);
}

static uint32_t stepper_tim_clk_hz(void)
{
    RCC_ClocksTypeDef clk;

    RCC_GetClocksFreq(&clk);
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1) {
        return clk.PCLK1_Frequency * 2u;
    }
    return clk.PCLK1_Frequency;
}

static void stepper_apply_pwm_hz(uint32_t hz)
{
    TIM_TimeBaseInitTypeDef tb;
    TIM_OCInitTypeDef oc;
    uint32_t tim_clk;
    uint32_t target_ticks;
    uint32_t psc;
    uint32_t arr;
    uint16_t pulse;

    if (!s_tim_inited) {
        return;
    }

    if (hz == 0u) {
        TIM_CCxCmd(STEPPER_PWM_TIM, STEPPER_PWM_CHANNEL, TIM_CCx_Disable);
        return;
    }

    tim_clk = stepper_tim_clk_hz();
    target_ticks = tim_clk / hz;
    if (target_ticks < 2u) {
        return;
    }

    psc = (target_ticks + 65535u) / 65536u;
    if (psc < 1u) {
        psc = 1u;
    }
    if (psc > 65536u) {
        psc = 65536u;
    }

    arr = (target_ticks + psc - 1u) / psc;
    if (arr < 2u) {
        arr = 2u;
    }
    if (arr > 65536u) {
        arr = 65536u;
    }

    pulse = (uint16_t)((arr + 1u) / 2u);
    if (pulse < 1u) {
        pulse = 1u;
    }

    TIM_ITConfig(STEPPER_PWM_TIM, TIM_IT_Update, DISABLE);

    tb.TIM_Prescaler         = (uint16_t)(psc - 1u);
    tb.TIM_CounterMode       = TIM_CounterMode_Up;
    tb.TIM_Period            = (uint16_t)(arr - 1u);
    tb.TIM_ClockDivision     = TIM_CKD_DIV1;
    tb.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(STEPPER_PWM_TIM, &tb);

    TIM_OCStructInit(&oc);
    oc.TIM_OCMode      = TIM_OCMode_PWM1;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_OCPolarity  = TIM_OCPolarity_High;
    oc.TIM_Pulse       = pulse;
    TIM_OC2Init(STEPPER_PWM_TIM, &oc);

    TIM_OC2PreloadConfig(STEPPER_PWM_TIM, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(STEPPER_PWM_TIM, ENABLE);

    TIM_SetCounter(STEPPER_PWM_TIM, 0);
    TIM_ClearFlag(STEPPER_PWM_TIM, TIM_FLAG_Update);
    TIM_CCxCmd(STEPPER_PWM_TIM, STEPPER_PWM_CHANNEL, TIM_CCx_Enable);
    TIM_Cmd(STEPPER_PWM_TIM, ENABLE);
}

static void stepper_move_abort(void)
{
    Stepper_Stop();
    s_move_state = STEPPER_ST_IDLE;
    s_move_pulses_done = 0u;
    s_move_pulses_total = 0u;
}

static void stepper_on_pulse_edge(uint8_t forward)
{
    if (forward) {
        s_pos_steps++;
    } else {
        s_pos_steps--;
    }
}

static void stepper_poll_one_pulse(void)
{
    uint16_t cnt = (uint16_t)STEPPER_PWM_TIM->CNT;

    if (cnt < s_last_cnt) {
        s_move_pulses_done++;
        stepper_on_pulse_edge(s_move_forward);
    }
    s_last_cnt = cnt;
}

/*================ 初始化 ================*/
void Stepper_Init(void)
{
    GPIO_InitTypeDef gpio;
    TIM_TimeBaseInitTypeDef tb;
    TIM_OCInitTypeDef oc;

    s_step_hz     = 0u;
    s_dir_forward = 1u;
    s_tim_inited  = 0u;
    s_pos_steps       = 0;
    s_target_delta_deg = 0.0f;
    s_delta_pending   = 0u;
    s_move_state      = STEPPER_ST_IDLE;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(STEPPER_PWM_TIM_RCC, ENABLE);

    gpio.GPIO_Pin   = STEPPER_DIR_GPIO_PIN;
    gpio.GPIO_Mode  = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(STEPPER_DIR_GPIO_PORT, &gpio);
    GPIO_SetBits(STEPPER_DIR_GPIO_PORT, STEPPER_DIR_GPIO_PIN);

    gpio.GPIO_Pin   = STEPPER_STEP_GPIO_PIN;
    gpio.GPIO_Mode  = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(STEPPER_STEP_GPIO_PORT, &gpio);

    TIM_DeInit(STEPPER_PWM_TIM);
    TIM_InternalClockConfig(STEPPER_PWM_TIM);

    tb.TIM_Prescaler         = 0;
    tb.TIM_CounterMode       = TIM_CounterMode_Up;
    tb.TIM_Period            = 0xFFFF;
    tb.TIM_ClockDivision     = TIM_CKD_DIV1;
    tb.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(STEPPER_PWM_TIM, &tb);

    TIM_OCStructInit(&oc);
    oc.TIM_OCMode      = TIM_OCMode_PWM1;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_OCPolarity  = TIM_OCPolarity_High;
    oc.TIM_Pulse       = 0;
    TIM_OC2Init(STEPPER_PWM_TIM, &oc);

    TIM_OC2PreloadConfig(STEPPER_PWM_TIM, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(STEPPER_PWM_TIM, ENABLE);
    TIM_ITConfig(STEPPER_PWM_TIM, TIM_IT_Update, DISABLE);

    TIM_SetCounter(STEPPER_PWM_TIM, 0);
    TIM_ClearFlag(STEPPER_PWM_TIM, TIM_FLAG_Update);
    TIM_CCxCmd(STEPPER_PWM_TIM, STEPPER_PWM_CHANNEL, TIM_CCx_Disable);
    TIM_Cmd(STEPPER_PWM_TIM, ENABLE);

    s_tim_inited = 1u;
}

/*================ 方向 / 速度 ================*/
void Stepper_SetDirection(uint8_t forward)
{
    s_dir_forward = forward ? 1u : 0u;
    if (s_dir_forward) {
        GPIO_SetBits(STEPPER_DIR_GPIO_PORT, STEPPER_DIR_GPIO_PIN);
    } else {
        GPIO_ResetBits(STEPPER_DIR_GPIO_PORT, STEPPER_DIR_GPIO_PIN);
    }
}

uint8_t Stepper_GetDirection(void)
{
    return s_dir_forward;
}

void Stepper_SetStepFrequencyHz(uint32_t hz)
{
    if (hz > STEPPER_STEP_HZ_MAX) {
        hz = STEPPER_STEP_HZ_MAX;
    } else if (hz != 0u && hz < STEPPER_STEP_HZ_MIN) {
        hz = STEPPER_STEP_HZ_MIN;
    }

    s_step_hz = hz;
    stepper_apply_pwm_hz(hz);
}

uint32_t Stepper_GetStepFrequencyHz(void)
{
    return s_step_hz;
}

void Stepper_SetSpeedRPM(float rpm)
{
    float hz_f;
    uint32_t hz;

    if (rpm <= 0.0f) {
        Stepper_SetStepFrequencyHz(0u);
        return;
    }

    hz_f = rpm * (float)STEPPER_PULSES_PER_REV / 60.0f;
    if (hz_f > (float)STEPPER_STEP_HZ_MAX) {
        hz = STEPPER_STEP_HZ_MAX;
    } else {
        hz = (uint32_t)(hz_f + 0.5f);
    }
    Stepper_SetStepFrequencyHz(hz);
}

float Stepper_GetSpeedRPM(void)
{
    if (s_step_hz == 0u) {
        return 0.0f;
    }
    return (float)s_step_hz * 60.0f / (float)STEPPER_PULSES_PER_REV;
}

void Stepper_Stop(void)
{
    Stepper_SetStepFrequencyHz(0u);
}

int32_t Stepper_GetPosSteps(void)
{
    return s_pos_steps;
}

/*================ 下行目标 / 周期更新 ================*/
void Stepper_SetTargetDeltaDeg(float deg)
{
    if (deg > STEPPER_DEG_MAX) {
        deg = STEPPER_DEG_MAX;
    } else if (deg < STEPPER_DEG_MIN) {
        deg = STEPPER_DEG_MIN;
    }

    if (deg == 0.0f) {
        return;
    }

    s_target_delta_deg = deg;
    s_delta_pending = 1u;
}

uint8_t Stepper_IsBusy(void)
{
    return (s_move_state != STEPPER_ST_IDLE) ? 1u : 0u;
}

void Stepper_Update(void)
{
    float deg;
    uint32_t pulses;

    if (!s_tim_inited) {
        return;
    }

    switch (s_move_state) {
    case STEPPER_ST_RUNNING:
        stepper_poll_one_pulse();
        if (s_move_pulses_done >= s_move_pulses_total) {
            stepper_move_abort();
        }
        break;

    case STEPPER_ST_DIR_WAIT:
        if (s_dir_wait_ms > STEPPER_UPDATE_PERIOD_MS) {
            s_dir_wait_ms = (uint16_t)(s_dir_wait_ms - STEPPER_UPDATE_PERIOD_MS);
        } else {
            s_dir_wait_ms = 0u;
        }
        if (s_dir_wait_ms == 0u) {
            Stepper_SetSpeedRPM(STEPPER_MOVE_RPM);
            if (s_step_hz == 0u) {
                stepper_move_abort();
                break;
            }
            TIM_ClearITPendingBit(STEPPER_PWM_TIM, TIM_IT_Update);
            s_last_cnt = (uint16_t)STEPPER_PWM_TIM->CNT;
            s_move_state = STEPPER_ST_RUNNING;
        }
        break;

    case STEPPER_ST_IDLE:
    default:
        break;
    }

    if (s_move_state != STEPPER_ST_IDLE || !s_delta_pending) {
        return;
    }

    s_delta_pending = 0u;
    deg = s_target_delta_deg;
    s_target_delta_deg = 0.0f;

    pulses = stepper_deg_to_pulses(stepper_fabsf(deg));
    if (pulses == 0u) {
        return;
    }

    s_move_forward = (deg > 0.0f) ? 1u : 0u;
    s_move_pulses_total = pulses;
    s_move_pulses_done = 0u;
    Stepper_SetDirection(s_move_forward);
    s_dir_wait_ms = STEPPER_DIR_SETUP_MS;
    s_move_state = STEPPER_ST_DIR_WAIT;
}

int16_t Stepper_GetLogicalDeg(void)
{
    float deg;

    deg = (float)s_pos_steps * 360.0f / (float)STEPPER_PULSES_PER_REV;
    if (deg > 180.0f) {
        deg = 180.0f;
    } else if (deg < -180.0f) {
        deg = -180.0f;
    }
    return (int16_t)(deg + (deg >= 0.0f ? 0.5f : -0.5f));
}

/*================ 阻塞转角（测试） ================*/
void Stepper_MoveDegrees(float deg)
{
    Stepper_SetTargetDeltaDeg(deg);
    while (Stepper_IsBusy() || s_delta_pending) {
        Stepper_Update();
    }
}

/*================ 上电测试 ================*/
#if STEPPER_TEST_ENABLE
void Stepper_TestRunMoveSequence(void)
{
    static const float move_deg[STEPPER_TEST_MOVE_COUNT] = {
        STEPPER_TEST_MOVE1_DEG,
        STEPPER_TEST_MOVE2_DEG,
        STEPPER_TEST_MOVE3_DEG,
        STEPPER_TEST_MOVE4_DEG,
        STEPPER_TEST_MOVE5_DEG,
    };
    uint8_t i;

    for (i = 0u; i < STEPPER_TEST_MOVE_COUNT; i++) {
        Stepper_MoveDegrees(move_deg[i]);
        if ((i + 1u) < STEPPER_TEST_MOVE_COUNT) {
            Delay_ms(STEPPER_TEST_MOVE_PAUSE_MS);
        }
    }
}
#endif
