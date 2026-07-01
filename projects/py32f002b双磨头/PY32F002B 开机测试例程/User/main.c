/**
  ******************************************************************************
  * @file    main.c
  * @brief   PB1 button power control and PB0 PWM speed levels demo.
  *
  *          Function summary:
  *          - PB1 long press: power on / power off
  *          - PB1 short press: switch 3 speed levels
  *          - PB0 output PWM: 50% / 70% / 100%
  *          - Speed levels map to WS2812 colors: red / green / blue
  ******************************************************************************
  */

#include "main.h"
#include "bsp_ws2812b.h"

#define BUTTON_PORT                 GPIOB
#define BUTTON_PIN                  GPIO_PIN_1
#define PWM_PORT                    GPIOB
#define PWM_PIN                     GPIO_PIN_0
#define LED_COUNT                   8u

#define LONG_PRESS_MS               1500u
#define DEBOUNCE_MS                 20u
#define PWM_PERIOD_MS               10u
#define PWM_MAX_LEVEL_MS            10u

#define SPEED_LEVEL_COUNT           3u
static const uint8_t s_speed_duty_ms[SPEED_LEVEL_COUNT] = { 5u, 7u, 10u };
static const uint8_t s_speed_color[SPEED_LEVEL_COUNT][3] = {
    {255u, 0u,   0u  }, /* 50%  -> red   */
    {0u,   255u, 0u  }, /* 70%  -> green */
    {0u,   0u,   255u}  /* 100% -> blue  */
};

static uint8_t  s_power_on = 0u;
static uint8_t  s_speed_index = 0u;
static uint32_t s_last_button_change_ms = 0u;

static void GPIO_Init(void);
static void PWM_OutputOnce(uint8_t duty_ms);
static void PWM_RunForPeriod(uint32_t period_ms, uint8_t duty_ms);
static void ApplyMotorOutput(void);
static void SetPowerState(uint8_t on);
static uint8_t ReadButtonPressed(void);
static void HandleButton(void);
static void SetAllLeds(uint8_t r, uint8_t g, uint8_t b);
static void UpdateIndicators(void);

static void GPIO_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* PB0: software PWM output */
    gpio.Pin = PWM_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(PWM_PORT, &gpio);

    /* PB1: button input, pull-up, active-low */
    gpio.Pin = BUTTON_PIN;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(BUTTON_PORT, &gpio);

    HAL_GPIO_WritePin(PWM_PORT, PWM_PIN, GPIO_PIN_RESET);
}

static uint8_t ReadButtonPressed(void)
{
    return (HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN) == GPIO_PIN_RESET) ? 1u : 0u;
}

static void SetAllLeds(uint8_t r, uint8_t g, uint8_t b)
{
    uint8_t led_buf[LED_COUNT * 3u];
    uint8_t i;

    for (i = 0u; i < LED_COUNT; i++)
    {
        led_buf[i * 3u + 0u] = r;
        led_buf[i * 3u + 1u] = g;
        led_buf[i * 3u + 2u] = b;
    }
    BSP_WS2812B_Send(led_buf, LED_COUNT);
}

static void UpdateIndicators(void)
{
    if (!s_power_on)
    {
        SetAllLeds(0u, 0u, 0u);
        return;
    }

    SetAllLeds(
        s_speed_color[s_speed_index][0],
        s_speed_color[s_speed_index][1],
        s_speed_color[s_speed_index][2]);
}

static void PWM_OutputOnce(uint8_t duty_ms)
{
    HAL_GPIO_WritePin(PWM_PORT, PWM_PIN, GPIO_PIN_SET);
    BSP_WS2812B_DelayMs(duty_ms);
    if (duty_ms < PWM_MAX_LEVEL_MS)
    {
        HAL_GPIO_WritePin(PWM_PORT, PWM_PIN, GPIO_PIN_RESET);
        BSP_WS2812B_DelayMs((uint32_t)(PWM_MAX_LEVEL_MS - duty_ms));
    }
}

static void PWM_RunForPeriod(uint32_t period_ms, uint8_t duty_ms)
{
    uint32_t i;
    for (i = 0u; i < period_ms; i++)
    {
        PWM_OutputOnce(duty_ms);
    }
}

static void ApplyMotorOutput(void)
{
    if (!s_power_on)
    {
        HAL_GPIO_WritePin(PWM_PORT, PWM_PIN, GPIO_PIN_RESET);
        BSP_WS2812B_DelayMs(20u);
        return;
    }

    PWM_RunForPeriod(PWM_PERIOD_MS, s_speed_duty_ms[s_speed_index]);
}

static void SetPowerState(uint8_t on)
{
    s_power_on = on ? 1u : 0u;
    if (!s_power_on)
    {
        HAL_GPIO_WritePin(PWM_PORT, PWM_PIN, GPIO_PIN_RESET);
    }
    UpdateIndicators();
}

static void HandleButton(void)
{
    static uint8_t  last_raw = 0u;
    static uint8_t  stable_state = 0u;
    static uint32_t press_start_ms = 0u;

    uint32_t now = g_systick_ms;
    uint8_t raw = ReadButtonPressed();

    if (raw != last_raw)
    {
        last_raw = raw;
        s_last_button_change_ms = now;
    }

    if ((now - s_last_button_change_ms) < DEBOUNCE_MS)
    {
        return;
    }

    if (raw != stable_state)
    {
        stable_state = raw;

        if (stable_state)
        {
            press_start_ms = now;
        }
        else
        {
            uint32_t press_ms = now - press_start_ms;
            if (press_ms < LONG_PRESS_MS)
            {
                if (s_power_on)
                {
                    s_speed_index++;
                    if (s_speed_index >= SPEED_LEVEL_COUNT)
                    {
                        s_speed_index = 0u;
                    }
                    UpdateIndicators();
                }
            }
        }
    }

    if (stable_state && ((now - press_start_ms) >= LONG_PRESS_MS))
    {
        SetPowerState((uint8_t)!s_power_on);
        while (ReadButtonPressed())
        {
            BSP_WS2812B_DelayMs(10u);
        }
        s_last_button_change_ms = g_systick_ms;
        last_raw = 0u;
        stable_state = 0u;
    }
}

int main(void)
{
    HAL_Init();
    GPIO_Init();
    BSP_WS2812B_Init();

    SetPowerState(0u);
    s_speed_index = 0u;
    UpdateIndicators();

    while (1)
    {
        HandleButton();
        ApplyMotorOutput();
    }
}
