/**
  ******************************************************************************
  * @file    main.c
  * @brief   TIM1双通道PWM（PA5/PA4）+ 长按启动 + 光电往返5次
  ******************************************************************************
  */

#include "main.h"
#include "py32f002b_hal_gpio_ex.h"
#include "Bsp_Uart/Bsp_Uart.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

/* 引脚定义 ------------------------------------------------------------------*/
#define MOTOR_F1_PORT              GPIOA                      /* RZ7899 F1 */
#define MOTOR_F1_PIN               GPIO_PIN_4                 /* TIM1_CH3 (AF2) */
#define MOTOR_B1_PORT              GPIOA                      /* RZ7899 B1 */
#define MOTOR_B1_PIN               GPIO_PIN_5                 /* TIM1_CH1 (AF2) */

#define SW_IN_PORT                 GPIOA                      /* 按钮输入 */
#define SW_IN_PIN                  GPIO_PIN_0                 /* 上拉输入，按下接地 */
#define SW_ON_LEVEL                GPIO_PIN_RESET

#define PHOTO_IN_PORT              GPIOB                      /* 光电传感器 */
#define PHOTO_IN_PIN               GPIO_PIN_4
#define PHOTO_ACTIVE_LEVEL         GPIO_PIN_SET               /* 你确认：被遮挡为高电平 */

#define SERVO_PORT                 GPIOA                      /* MG996R 信号脚 */
#define SERVO_PIN                  GPIO_PIN_1                 /* 默认 PA1，可按实际改 */

/* PWM与控制参数 -------------------------------------------------------------*/
#define MOTOR_PWM_FREQ_HZ          20000U                     /* 20kHz */
#define DUTY_MIN_PERMILLE          300U                       /* 30% */
#define DUTY_MID_PERMILLE          600U                       /* 60% */
#define DUTY_MAX_PERMILLE          1000U                      /* 100% */
#define LONG_PRESS_START_MS        1500U                      /* 长按启动阈值 */
#define DEBUG_PRINT_PERIOD_MS      500U                       /* 串口打印周期 */

#define SENSOR_WAIT_TIMEOUT_MS     10000U                     /* 去程等光电超时 */
#define ROUND_TRIP_REPEAT_COUNT    5U                         /* 来回重复5次 */

/* MG996R：50Hz，0°≈0.5ms，90°≈1.5ms，180°≈2.5ms */
#define SERVO_PWM_FREQ_HZ          50U
#define SERVO_PULSE_0_US           500U
#define SERVO_PULSE_90_US          1500U
#define SERVO_TRIGGER_DELAY_MS     1000U                 /* 光电触发后延时1秒再转90° */

/* 类型定义 ------------------------------------------------------------------*/
typedef enum
{
  SPEED_GEAR_30 = 0,
  SPEED_GEAR_60,
  SPEED_GEAR_100,
  SPEED_GEAR_COUNT
} APP_SpeedGear_t;

typedef enum
{
  APP_RUN_IDLE = 0,
  APP_RUN_TO_SENSOR,
  APP_RUN_BACK_BY_TIME,
  APP_RUN_DONE,
  APP_RUN_FAULT_TIMEOUT
} APP_RunState_t;

typedef struct
{
  bool buttonPrevPressed;
  uint32_t buttonPressTick;
  bool longPressHandled;

  bool motorRunning;
  bool motorForward;                    /* true: 正向(F1 PWM); false: 反向(B1 PWM) */
  APP_SpeedGear_t speedGear;

  APP_RunState_t runState;
  uint32_t stateStartTick;
  uint32_t measuredToSensorMs;
  uint8_t completedRoundTrips;

  bool sensorPrevActive;
  uint32_t lastDebugPrintTick;

  bool servoTurned90;                                  /* 光电触发后舵机已转到90° */
  bool servoPending90;                                 /* 已触发光电，等待延时后转90° */
  uint32_t servoTriggerTick;                           /* 光电触发时刻 */
} APP_Ctrl_t;

/* 私有变量 ------------------------------------------------------------------*/
static TIM_HandleTypeDef htim1;
static uint16_t sPwmArr;

/* 私有函数声明 --------------------------------------------------------------*/
static void APP_GPIO_Init(void);
static void APP_TIM1_Pwm_Init(void);
static uint16_t APP_GetDutyByGear(APP_SpeedGear_t gear);
static void APP_SetDutyPermille_CH1(uint16_t permille);
static void APP_SetDutyPermille_CH3(uint16_t permille);
static void APP_MotorStop(void);
static void APP_MotorApply(bool forward, uint16_t dutyPermille);
static void APP_Servo_SetPulseUs_Blocking(uint16_t pulseUs);/* 舵机按脉宽输出（阻塞） */
static void APP_Servo_SetAngle0_Blocking(void);        /* 舵机回到0° */
static void APP_Servo_SetAngle90_Blocking(void);       /* 舵机转到90°（阻塞式脉冲输出） */
static bool APP_IsSensorActive(void);
static bool APP_IsSensorRisingActive(APP_Ctrl_t *app);
static void APP_StartRoundTripSequence(APP_Ctrl_t *app, uint32_t now);
static void APP_RunRoundTripStep(APP_Ctrl_t *app, uint32_t now);

void APP_OnSysTick1ms(void)
{
  /* 当前逻辑使用HAL_GetTick轮询，不需要1ms任务 */
}

/**
  * @brief  TIM1 PWM底层初始化：PA5=CH1, PA4=CH3
  */
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM1)
  {
    GPIO_InitTypeDef g = {0};

    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    g.Pin = MOTOR_F1_PIN | MOTOR_B1_PIN;
    g.Mode = GPIO_MODE_AF_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    g.Alternate = GPIO_AF2_TIM1;
    HAL_GPIO_Init(GPIOA, &g);
  }
}

/**
  * @brief  TIM1双通道PWM初始化（20kHz）
  */
static void APP_TIM1_Pwm_Init(void)
{
  uint32_t timclk = HAL_RCC_GetHCLKFreq();
  uint32_t psc = 0U;
  uint32_t arr;

  if (timclk == 0U)
  {
    timclk = 24000000U;
  }

  arr = timclk / MOTOR_PWM_FREQ_HZ;
  if (arr > 65536U)
  {
    psc = arr / 65536U;
    arr = timclk / ((psc + 1U) * MOTOR_PWM_FREQ_HZ);
  }
  if (arr < 4U)
  {
    arr = 4U;
  }

  sPwmArr = (uint16_t)(arr - 1U);

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = (uint16_t)psc;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = sPwmArr;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0U;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    APP_ErrorHandler();
  }

  {
    TIM_OC_InitTypeDef oc = {0};
    oc.OCMode = TIM_OCMODE_PWM1;
    oc.Pulse = 0U;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;

    if (HAL_TIM_PWM_ConfigChannel(&htim1, &oc, TIM_CHANNEL_1) != HAL_OK)
    {
      APP_ErrorHandler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &oc, TIM_CHANNEL_3) != HAL_OK)
    {
      APP_ErrorHandler();
    }
  }

  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK)
  {
    APP_ErrorHandler();
  }
  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3) != HAL_OK)
  {
    APP_ErrorHandler();
  }
}

static uint16_t APP_GetDutyByGear(APP_SpeedGear_t gear)
{
  switch (gear)
  {
    case SPEED_GEAR_30:  return DUTY_MIN_PERMILLE;
    case SPEED_GEAR_60:  return DUTY_MID_PERMILLE;
    case SPEED_GEAR_100: return DUTY_MAX_PERMILLE;
    default:             return DUTY_MIN_PERMILLE;
  }
}

static void APP_SetDutyPermille_CH1(uint16_t permille)
{
  uint32_t p = permille;
  uint32_t ccr;

  if (p == 0U)
  {
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0U);
    return;
  }
  if (p < DUTY_MIN_PERMILLE)
  {
    p = DUTY_MIN_PERMILLE;
  }
  if (p > 1000U)
  {
    p = 1000U;
  }

  ccr = ((uint32_t)sPwmArr + 1U) * p / 1000U;
  if (ccr > (uint32_t)sPwmArr)
  {
    ccr = (uint32_t)sPwmArr;
  }
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr);
}

static void APP_SetDutyPermille_CH3(uint16_t permille)
{
  uint32_t p = permille;
  uint32_t ccr;

  if (p == 0U)
  {
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0U);
    return;
  }
  if (p < DUTY_MIN_PERMILLE)
  {
    p = DUTY_MIN_PERMILLE;
  }
  if (p > 1000U)
  {
    p = 1000U;
  }

  ccr = ((uint32_t)sPwmArr + 1U) * p / 1000U;
  if (ccr > (uint32_t)sPwmArr)
  {
    ccr = (uint32_t)sPwmArr;
  }
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, ccr);
}

/**
  * @brief  停机：F1=0, B1=0
  */
static void APP_MotorStop(void)
{
  APP_SetDutyPermille_CH1(0U);
  APP_SetDutyPermille_CH3(0U);
}

/**
  * @brief  电机输出
  * @note   正向: F1(PA4,CH3)=PWM, B1(PA5,CH1)=0
  *         反向: F1(PA4,CH3)=0,   B1(PA5,CH1)=PWM
  */
static void APP_MotorApply(bool forward, uint16_t dutyPermille)
{
  if (forward)
  {
    APP_SetDutyPermille_CH1(0U);
    APP_SetDutyPermille_CH3(dutyPermille);
  }
  else
  {
    APP_SetDutyPermille_CH3(0U);
    APP_SetDutyPermille_CH1(dutyPermille);
  }
}

/*
 * MG996R：阻塞式软PWM脉冲输出，避免占用新的定时器资源。
 */
static void APP_Servo_SetPulseUs_Blocking(uint16_t pulseUs)
{
  uint32_t i;
  uint32_t highMs;
  uint32_t periodMs = 1000U / SERVO_PWM_FREQ_HZ;

  highMs = ((uint32_t)pulseUs + 500U) / 1000U;         /* us转ms并四舍五入 */
  if (highMs == 0U)
  {
    highMs = 1U;
  }
  if (highMs >= periodMs)
  {
    highMs = periodMs - 1U;
  }

  for (i = 0U; i < 25U; i++)
  {
    HAL_GPIO_WritePin(SERVO_PORT, SERVO_PIN, GPIO_PIN_SET);
    HAL_Delay(highMs);
    HAL_GPIO_WritePin(SERVO_PORT, SERVO_PIN, GPIO_PIN_RESET);
    HAL_Delay(periodMs - highMs);
  }
}

static void APP_Servo_SetAngle0_Blocking(void)
{
  APP_Servo_SetPulseUs_Blocking(SERVO_PULSE_0_US);
}

static void APP_Servo_SetAngle90_Blocking(void)
{
  APP_Servo_SetPulseUs_Blocking(SERVO_PULSE_90_US);
}

static bool APP_IsSensorActive(void)
{
  return (HAL_GPIO_ReadPin(PHOTO_IN_PORT, PHOTO_IN_PIN) == PHOTO_ACTIVE_LEVEL);
}

static bool APP_IsSensorRisingActive(APP_Ctrl_t *app)
{
  bool nowActive = APP_IsSensorActive();
  bool edge = ((!app->sensorPrevActive) && nowActive);
  app->sensorPrevActive = nowActive;
  return edge;
}

static void APP_StartRoundTripSequence(APP_Ctrl_t *app, uint32_t now)
{
  app->motorRunning = true;
  app->runState = APP_RUN_TO_SENSOR;
  app->stateStartTick = now;
  app->measuredToSensorMs = 0U;
  app->completedRoundTrips = 0U;
  app->servoTurned90 = false;
  app->servoPending90 = false;
  app->servoTriggerTick = 0U;

  app->motorForward = true;
  APP_MotorApply(app->motorForward, APP_GetDutyByGear(app->speedGear));
}

static void APP_RunRoundTripStep(APP_Ctrl_t *app, uint32_t now)
{
  switch (app->runState)
  {
    case APP_RUN_IDLE:
      break;

    case APP_RUN_TO_SENSOR:
      APP_MotorApply(app->motorForward, APP_GetDutyByGear(app->speedGear));
      if (APP_IsSensorRisingActive(app))
      {
        app->measuredToSensorMs = now - app->stateStartTick;

        if (!app->servoTurned90)
        {
          app->servoPending90 = true;
          app->servoTriggerTick = now;                  /* 记录光电触发时刻，延时后再转90° */
        }

        app->motorForward = !app->motorForward;
        app->stateStartTick = now;
        app->runState = APP_RUN_BACK_BY_TIME;
      }
      else if ((now - app->stateStartTick) > SENSOR_WAIT_TIMEOUT_MS)
      {
        app->runState = APP_RUN_FAULT_TIMEOUT;
      }
      break;

    case APP_RUN_BACK_BY_TIME:
      APP_MotorApply(app->motorForward, APP_GetDutyByGear(app->speedGear));
      if ((now - app->stateStartTick) >= app->measuredToSensorMs)
      {
        app->completedRoundTrips++;
        if (app->completedRoundTrips >= ROUND_TRIP_REPEAT_COUNT)
        {
          app->runState = APP_RUN_DONE;
        }
        else
        {
          app->motorForward = !app->motorForward;
          app->stateStartTick = now;
          app->runState = APP_RUN_TO_SENSOR;
        }
      }
      break;

    case APP_RUN_DONE:
      app->motorRunning = false;
      APP_MotorStop();
      if (app->servoTurned90)
      {
        APP_Servo_SetAngle0_Blocking();                /* 磨刀完成后舵机回退90°(回到0°) */
        app->servoTurned90 = false;
      }
      app->servoPending90 = false;
      break;

    case APP_RUN_FAULT_TIMEOUT:
      app->motorRunning = false;
      APP_MotorStop();
      app->servoPending90 = false;
      break;

    default:
      app->runState = APP_RUN_IDLE;
      app->motorRunning = false;
      APP_MotorStop();
      break;
  }
}

static void APP_GPIO_Init(void)
{
  GPIO_InitTypeDef g = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* 按键 PA0，上拉输入，按下接地 */
  g.Pin = SW_IN_PIN;
  g.Mode = GPIO_MODE_INPUT;
  g.Pull = GPIO_PULLUP;
  g.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SW_IN_PORT, &g);

  /* 光电 PB4，上拉输入 */
  g.Pin = PHOTO_IN_PIN;
  g.Mode = GPIO_MODE_INPUT;
  g.Pull = GPIO_PULLUP;
  g.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(PHOTO_IN_PORT, &g);

  /* 舵机信号脚 PA1，普通推挽输出（软PWM） */
  g.Pin = SERVO_PIN;
  g.Mode = GPIO_MODE_OUTPUT_PP;
  g.Pull = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SERVO_PORT, &g);
  HAL_GPIO_WritePin(SERVO_PORT, SERVO_PIN, GPIO_PIN_RESET);
}

int main(void)
{
  APP_Ctrl_t app = {0};

  HAL_Init();
  APP_GPIO_Init();
  APP_TIM1_Pwm_Init();
  DEBUG_USART_Config(115200);
  printf("app start (TIM1 CH1/CH3)\r\n");

  app.buttonPrevPressed = false;
  app.buttonPressTick = 0U;
  app.longPressHandled = false;

  app.motorRunning = false;
  app.motorForward = true;
  app.speedGear = SPEED_GEAR_30;

  app.runState = APP_RUN_IDLE;
  app.stateStartTick = 0U;
  app.measuredToSensorMs = 0U;
  app.completedRoundTrips = 0U;

  app.sensorPrevActive = APP_IsSensorActive();
  app.lastDebugPrintTick = HAL_GetTick();
  app.servoTurned90 = false;
  app.servoPending90 = false;
  app.servoTriggerTick = 0U;

  APP_Servo_SetAngle0_Blocking();

  APP_MotorStop();

  while (1)
  {
    uint32_t now = HAL_GetTick();
    bool buttonPressed = (HAL_GPIO_ReadPin(SW_IN_PORT, SW_IN_PIN) == SW_ON_LEVEL);

    if (buttonPressed && (!app.buttonPrevPressed))
    {
      app.buttonPressTick = now;
      app.longPressHandled = false;
    }

    if (buttonPressed && (!app.longPressHandled))
    {
      if ((now - app.buttonPressTick) >= LONG_PRESS_START_MS)
      {
        app.longPressHandled = true;
        if (!app.motorRunning)
        {
          APP_StartRoundTripSequence(&app, now);
        }
      }
    }

    if ((!buttonPressed) && app.buttonPrevPressed)
    {
      uint32_t pressTime = now - app.buttonPressTick;
      if (pressTime < LONG_PRESS_START_MS)
      {
        app.speedGear = (APP_SpeedGear_t)(((uint8_t)app.speedGear + 1U) % (uint8_t)SPEED_GEAR_COUNT);
      }
    }

    app.buttonPrevPressed = buttonPressed;

    APP_RunRoundTripStep(&app, now);

    if (app.servoPending90 && (!app.servoTurned90))
    {
      if ((now - app.servoTriggerTick) >= SERVO_TRIGGER_DELAY_MS)
      {
        APP_Servo_SetAngle90_Blocking();
        app.servoTurned90 = true;
        app.servoPending90 = false;
      }
    }

    if (!app.motorRunning)
    {
      APP_MotorStop();
    }

    if ((now - app.lastDebugPrintTick) >= DEBUG_PRINT_PERIOD_MS)
    {
      app.lastDebugPrintTick = now;
      printf("btn=%d run=%d dir=%d gear=%d state=%d sensor=%d t_to=%lu cnt=%u\r\n",
             buttonPressed ? 1 : 0,
             app.motorRunning ? 1 : 0,
             app.motorForward ? 1 : 0,
             (int)app.speedGear,
             (int)app.runState,
             APP_IsSensorActive() ? 1 : 0,
             (unsigned long)app.measuredToSensorMs,
             (unsigned int)app.completedRoundTrips);
    }
  }
}

void APP_ErrorHandler(void)
{
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file;
  (void)line;
  while (1)
  {
  }
}
#endif
