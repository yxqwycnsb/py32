#include "a4988.h"
#include "stm32f10x.h"

Stepper_State_t stepper1_state = {0};
volatile int32_t stepper1_pulse_count = 0;
volatile uint8_t g_microstep_ratio = 16;

// 兼容旧工程里残留的电机2状态引用，当前版本未实际使用这些逻辑
// 只保留最小定义，避免 ARMCC 因未定义类型直接报错
typedef struct {
    uint8_t is_running;
} motor2_smooth_t;

typedef struct {
    uint8_t is_running;
    uint32_t next_tick;
    uint8_t pulse_high;
    uint8_t direction;
} motor2_const_speed_t;

static motor2_smooth_t motor2_smooth = {0};
static motor2_const_speed_t motor2_const_speed = {0};

static void _a4988_common_gpio_init(GPIO_TypeDef *port, uint16_t step_pin, uint16_t dir_pin, uint16_t en_pin)
{
    GPIO_InitTypeDef gpio;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Pin = dir_pin | en_pin;
    GPIO_Init(port, &gpio);

    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Pin = step_pin;
    GPIO_Init(port, &gpio);
}

uint32_t a4988_get_steps_per_rev(void)
{
    return (uint32_t)STEPS_PER_REV_FULL * (uint32_t)g_microstep_ratio;
}

int32_t a4988_get_pulse_count(void)
{
    return stepper1_pulse_count;
}

void a4988_init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    _a4988_common_gpio_init(MOTOR_PORT, STEP_PIN, DIR_PIN, ENABLE_PIN);
    GPIO_ResetBits(MOTOR_PORT, DIR_PIN);
    GPIO_SetBits(MOTOR_PORT, ENABLE_PIN);

    GPIO_InitTypeDef gpio;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Pin = MS1_PIN;
    GPIO_Init(MS1_PORT, &gpio);
    GPIO_SetBits(MS1_PORT, MS1_PIN);
    g_microstep_ratio = 16;
}

void a4988_set_microstep(MicrostepMode_t mode)
{
    if (mode == MICROSTEP_FULL) {
        GPIO_ResetBits(MS1_PORT, MS1_PIN);
        g_microstep_ratio = 1;
    } else {
        GPIO_SetBits(MS1_PORT, MS1_PIN);
        g_microstep_ratio = 16;
    }
}

void a4988_set_direction(bool direction)
{
    stepper1_state.direction = direction;
    GPIO_WriteBit(MOTOR_PORT, DIR_PIN, direction ? Bit_SET : Bit_RESET);
}

void a4988_enable(bool enable)
{
    GPIO_WriteBit(MOTOR_PORT, ENABLE_PIN, enable ? Bit_RESET : Bit_SET);
    stepper1_state.enabled = enable;
}

void a4988_set_hold_enable(bool enable)
{
    (void)enable;
}

void a4988_stop_motor(void)
{
    TIM_Cmd(TIM2, DISABLE);
    TIM_ITConfig(TIM2, TIM_IT_Update, DISABLE);
}

void a4988_run_rpm(uint16_t rpm)
{
    if (rpm == 0 || rpm > A4988_MAX_RPM) return;

    TIM_Cmd(TIM2, DISABLE);
    TIM_ITConfig(TIM2, TIM_IT_Update, DISABLE);

    uint32_t steps_per_sec = (uint32_t)rpm * a4988_get_steps_per_rev() / 60;
    if (steps_per_sec == 0) return;

    uint32_t period = 1000000 / steps_per_sec;
    if (period < 2) period = 2;

    TIM_TimeBaseInitTypeDef tim;
    tim.TIM_Prescaler = 72 - 1;
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    tim.TIM_Period = period - 1;
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    tim.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM2, &tim);

    TIM_OCInitTypeDef oc;
    oc.TIM_OCMode = TIM_OCMode_PWM1;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_Pulse = period / 2;
    oc.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC2Init(TIM2, &oc);
    TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);

    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

    NVIC_InitTypeDef nvic;
    nvic.NVIC_IRQChannel = TIM2_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 5;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    TIM_Cmd(TIM2, ENABLE);
}

int32_t a4988_get_angle(void)
{
    uint32_t spr = a4988_get_steps_per_rev();
    if (spr == 0U) return 0;
    return (stepper1_pulse_count * 360) / (int32_t)spr;
}

void a4988_reset_angle(void)
{
    stepper1_pulse_count = 0;
}

int32_t degrees_to_steps(int32_t degrees)
{
    uint32_t spr = a4988_get_steps_per_rev();
    return (int32_t)((int64_t)degrees * (int64_t)spr / 360);
}

void a4988_test_pwm_freq(uint32_t freq_hz)
{
    (void)freq_hz;
}
