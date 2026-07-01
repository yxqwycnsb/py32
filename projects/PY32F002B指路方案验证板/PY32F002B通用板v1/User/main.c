/**
  ******************************************************************************
  * @file    main.c
  * @author  MCU Application Team
  * @brief   Main program body
  ******************************************************************************
  **/

/* Includes ------------------------------------------------------------------*/
#include "main.h"                                    /* HAL 库与工程主头文件 */
#include "Bsp_Uart/Bsp_Uart.h"                      /* 串口调试封装头文件 */
#include "Bsp-TMI/bsp_tim1_pwm.h"                   /* TIM1 硬件 PWM 驱动接口 */
#include <stdio.h>                                   /* 标准输入输出，供 printf 使用 */
#include <stdbool.h>                                 /* bool 布尔类型定义 */
#include <stdint.h>                                  /* 标准整型类型定义 */

/* Private define ------------------------------------------------------------*/
#define APP_UART_DEBUG_ENABLE      1U               /* 1：开启串口调试，0：关闭 */
#define APP_LED_DIAG_ENABLE        0U                /* 1：开启指示灯诊断模式，0：关闭 */
#define APP_ADC_ENABLE             1U                /* 1：开启 ADC 采样，0：关闭 */
#define APP_STALL_PROTECT_ENABLE   1U                /* 1：开启堵转/过流保护，0：关闭 */
#define APP_BOOST_ENABLE            1U               /* 1：开启升压控制，0：关闭 */
#define APP_LOW_VBAT_PROTECT_ENABLE 1U                /* 1：开启低电压锁定，0：关闭 */
#define APP_CHARGE_TASK_ENABLE      1U               /* 1：开启充电任务，0：关闭 */
/* BOOST_EN_ON_MV / BOOST_EN_OFF_MV 定义见下方阈值区 */
#define BOOST_ENTER_CONFIRM_MS     2000U             /* 升压进入确认时间，低于 3.6V 持续该时间才开启 */
#define BOOST_RELEASE_CONFIRM_MS    2000U             /* 升压退出确认时间，高于 3.7V 持续该时间才关闭 */
#define BOOST_MOTOR_START_DELAY_MS  5000U             /* 电机通电后延时 5s 才允许开启升压 */

#if APP_UART_DEBUG_ENABLE
#define APP_DBG_PRINT(...)         printf(__VA_ARGS__)   /* 调试打印宏 */
#else
#define APP_DBG_PRINT(...)         ((void)0)
#endif

/* 引脚映射 ------------------------------------------------------------------*/
#define MOTOR_EN_PORT              GPIOA             /* 电机 PWM 输出端口 */
#define MOTOR_EN_PIN               GPIO_PIN_0        /* 电机 PWM 输出引脚 PA0 */

#define BOOST_EN_PORT              GPIOA             /* 升压使能端口 */
#define BOOST_EN_PIN               GPIO_PIN_5        /* 升压使能引脚 PA5 */

#define LAMP1_PORT                 GPIOB             /* 指示灯 1 端口 */
#define LAMP1_PIN                  GPIO_PIN_0        /* 指示灯 1 引脚 PB0 */
#define LAMP2_PORT                 GPIOB             /* 指示灯 2 端口 */
#define LAMP2_PIN                  GPIO_PIN_1        /* 指示灯 2 引脚 PB1 */
#define LAMP3_PORT                 GPIOB             /* 指示灯 3 端口 */
#define LAMP3_PIN                  GPIO_PIN_3        /* 指示灯 3 引脚 PB3 */

#define SW_IN1_PORT                GPIOB             /* 按键输入端口 */
#define SW_IN1_PIN                 GPIO_PIN_7        /* 按键输入引脚 PB7 */
#define CHARGE_DET_PORT            GPIOC             /* 充电检测端口 */
#define CHARGE_DET_PIN             GPIO_PIN_1        /* 充电检测引脚 PC1 */

#define TP4056_CHRG_PORT           GPIOB             /* TP4056 充电状态输入端口 */
#define TP4056_CHRG_PIN            GPIO_PIN_2        /* TP4056 CHRG 输入引脚 PB2 */

#define I_SENSE_ADC_CHANNEL        ADC_CHANNEL_1     /* 电流采样 ADC 通道，对应 PA3 */
#define VBAT_SENSE_ADC_CHANNEL     ADC_CHANNEL_3     /* 电池电压采样 ADC 通道，对应 PA6 */

/* 逻辑电平定义 --------------------------------------------------------------*/
#define SW_ON_LEVEL                GPIO_PIN_RESET    /* 按键按下有效电平：低电平有效 */
#define TP4056_ACTIVE_LEVEL        GPIO_PIN_RESET    /* TP4056 充电有效电平：低电平有效 */

/* 采样与阈值参数------------------------------------------------------------*/
#define CURRENT_SAMPLE_PERIOD_MS   20U                /* 电流采样周期 20ms */
#define VBAT_SAMPLE_PERIOD_MS      100U              /* 电池采样周期 100ms */
#define OVERCURRENT_CONFIRM_CNT    10U                 /* 过流连续确认次数（调试阶段降低，便于确认能否触发堵转） */
#define CURRENT_LIMIT_MA_GEAR0     4000U             /* 0 档堵转/过流阈值(mA) */
#define CURRENT_LIMIT_MA_GEAR1     4500U             /* 1 档堵转/过流阈值(mA) */
#define CURRENT_LIMIT_MA_GEAR2     6000U             /* 2 档堵转/过流阈值(mA) */
#define STALL_DETECT_BLANKING_MS   1000U             /* 启动后堵转屏蔽时间 1200ms */

/* ADC 参考电压--------------------------------------------------------------*/
#define ADC_VREF_MV                1500U             /* ADC 参考电压 1.5V，单位 mV */

/* 分流电阻参数 --------------------------------------------------------------*/
#define SHUNT_RESISTOR_MOHM        5U                /* 分流电阻 5mΩ */

/* 电流零点校准参数 ----------------------------------------------------------*/
#define ISENSE_OFFSET_SAMPLES      64U               /* 电流零点采样次数 */
#define ISENSE_OFFSET_SAMPLE_DELAY_MS 1U             /* 零点采样间隔 1ms */
#define ISENSE_OFFSET_RETRY_MAX    5U                /* 零点校准重试次数 */
#define ISENSE_ADC_VALID_MIN       8U                /* 电流 ADC 有效最小值 */
#define ISENSE_ADC_VALID_MAX       4087U             /* 电流 ADC 有效最大值 */
#define ISENSE_LPF_SHIFT           2U                /* 电流一阶低通滤波系数，1/4 */

/* 电流换算系数: I(mA)=adc*ADC_VREF_MV*1000/(4095*R_mohm) ------------------*/
#define CURRENT_MA_NUM             (ADC_VREF_MV * 1000U)      /* 电流换算分子 */
#define CURRENT_MA_DEN             (4095U * SHUNT_RESISTOR_MOHM) /* 电流换算分母 */

/* 电池分压比: 22k:10k ----------------------------------------------------*/
#define VBAT_DIV_NUM               32U               /* 分压上臂 + 下臂 = 22 + 10 */
#define VBAT_DIV_DEN               10U               /* 下臂电阻 10k */

/* 电池电压阈值 --------------------------------------------------------------*/
#define VBAT_EMPTY_MV              3000U             /* 电池空电压阈值 */
#define VBAT_FULL_MV               4100U             /* 电池满电阈值 */
#define VBAT_FULL_RELEASE_MV       4050U             /* 满电释放阈值 */

/* LED 与保护参数 ---------------------------------------------------------*/
#define LED_ALARM_BLINK_MS         250U              /* 告警灯闪烁周期 */
#define CHARGE_LAMP_STEP_MS        300U              /* 充电灯步进周期 */
#define RED_BREATH_STEP_MS         10U               /* 红灯呼吸步进周期 */
#define RED_BREATH_MAX             100U              /* 红灯呼吸最大占空比 */
#define RED_SOFT_PWM_PERIOD_MS     25U               /* 红灯软 PWM 周期 */
#define LOW_VBAT_LOCK_MV           3200U             /* 低电锁定阈值 */
#define LOW_VBAT_RELEASE_MV        3250U             /* 低电释放阈值 */
#define LOW_VBAT_ENTER_CONFIRM_MS  2000U             /* 低电进入确认时间 */
#define LOW_VBAT_BLINK_TIMES       5U                /* 低电提示闪烁次数 */
#define LOW_VBAT_BLINK_MS          150U              /* 低电提示闪烁周期 */
#define GREEN_STALL_BLINK_TOTAL_MS 2000U            /* 堵转提示总时长 */
#define GREEN_STALL_BLINK_MS       150U              /* 堵转提示闪烁周期 */
#define PROG_VERIFY_TOGGLE_MS      2000U             /* 上电自检灯切换周期 */
#define PROG_VERIFY_TOGGLE_TIMES   3U                /* 上电自检灯切换次数 */
#define POWERUP_WAIT_MS            2000U             /* 上电等待时间 */
#define APP_STOP_ENABLE            1U                /* 1：启用 STOP 逻辑，0：关闭 */
#define APP_STOP_DELAY_MS          5000U             /* 停机后进入 STOP 的延时 */
#define BUTTON_LONG_PRESS_MS       1500U              /* 按键长按判定时间 */
#define BUTTON_DEBOUNCE_MS          50U               /* 按键消抖时间 */
#define BOOST_EN_ON_MV             3600U             /* 升压开启阈值 */
#define BOOST_EN_OFF_MV            3700U             /* 升压关闭阈值 */

/* 电机软启动与挡位参数-------------------------------------------------------------*/
#define MOTOR_SOFT_START_DUTY_START  100U            /* 软启动起始占空比(%) */
#define MOTOR_SOFT_START_DUTY_END    100U            /* 软启动结束占空比(%) */
#define MOTOR_SOFT_START_TOTAL_MS    1500U           /* 软启动总时长(ms) */
#define MOTOR_SOFT_START_STEP_MS     40U             /* 软启动步进周期(ms) */
#define MOTOR_GEAR0_DUTY             60U             /* 0 档目标占空比 */
#define MOTOR_GEAR1_DUTY             80U             /* 1 档目标占空比 */
#define MOTOR_GEAR2_DUTY             100U            /* 2 档目标占空比 */
/* 软启动斜率估算：(100-60)/(1500/30) ≈ 0.8%/40ms */

/* 升压控制端口 --------------------------------------------------------------*/
#define BOOST_EN_PORT              GPIOA             /* 升压使能端口 */
#define BOOST_EN_PIN               GPIO_PIN_5        /* 升压使能引脚 PA5 */

/* Private variables ---------------------------------------------------------*/
static ADC_HandleTypeDef hadc1;                      /* ADC1 句柄 */

static volatile uint32_t gTaskTickMs = 0U;           /* 1ms 系统节拍计数 */
static volatile bool gCurrentSampleFlag = false;     /* 电流采样标志(20ms) */
static volatile bool gVbatSampleFlag = false;        /* 电压采样标志(100ms) */

/* 状态枚举 ------------------------------------------------------------*/
typedef enum
{
  APP_BUTTON_DISCONNECT = 0,
  APP_BUTTON_CONNECT
} APP_ButtonState_t;

typedef enum
{
  APP_SYS_NORMAL = 0,
  APP_SYS_IDLE,
  APP_SYS_CHARGING,
  APP_SYS_LOW_BATTERY
} APP_SystemState_t;

typedef enum
{
  APP_MOTOR_STOP = 0,
  APP_MOTOR_RUN,
  APP_MOTOR_OVERCURRENT
} APP_MotorState_t;

typedef enum
{
  APP_STOP_REASON_NONE = 0,
  APP_STOP_REASON_LOW_BATTERY,
  APP_STOP_REASON_STALL
} APP_StopReason_t;

typedef struct
{
  APP_ButtonState_t buttonState;                      /* 当前按键状态 */
  APP_ButtonState_t prevButtonState;                  /* 上一拍按键状态 */
  APP_SystemState_t systemState;                      /* 当前系统状态：正常/空闲/充电/低电 */
  uint8_t lampState;                                  /* 三个指示灯位图：bit0=PB0，bit1=PB1，bit2=PB3 */
  bool chargeActive;                                  /* 当前是否处于充电状态 */
  bool fullCharged;                                   /* 当前是否判定为充满电 */
  bool boostEnabled;                                  /* 升压模块当前是否已使能 */
  bool boostBelowThreshold;                           /* 升压进入阈值以下标志 */
  bool boostAboveThreshold;                           /* 升压退出阈值以上标志 */
  uint32_t boostThresholdTick;                        /* 升压阈值持续计时起点 */
  bool stallFault;                                    /* 堵转/过流故障 */
  APP_MotorState_t motorState;                        /* 电机状态：停止/运行/过流 */
  APP_StopReason_t motorStopReason;                   /* 最近一次停机原因 */
  bool startRequestLatched;                           /* 启动请求锁存 */
  uint32_t motorStartTick;                            /* 最近一次启动时间戳（用于堵转屏蔽） */
  uint32_t boostMotorOnTick;                          /* 电机通电时刻（用于升压延时 5s） */
  bool motorEnable;                                   /* 电机使能输出状态 */
  uint8_t motorGear;                                  /* 当前挡位：0/1/2 */
  uint8_t motorTargetDuty;                            /* 目标占空比(%) */

  uint8_t overCurrentCnt;                             /* 过流连续计数 */
  uint16_t iSenseOffsetAdc;                           /* 电流零点偏置 */
  uint32_t currentmAFiltered;                         /* 电流滤波值 */
  uint16_t iSenseAdcRaw;                              /* 电流通道原始 ADC */
  uint16_t vBatAdcRaw;                                /* 电池通道原始 ADC */
  bool iSenseAdcValid;                                /* 电流 ADC 是否有效 */
  uint32_t vbatmV;                                    /* 电池电压 mV 缓存 */

  /* 电机软启动 */
  bool softStartActive;                               /* 缓启动进行中 */
  uint8_t softStartDuty;                              /* 当前软启动占空比(%) */
  uint32_t lastSoftStartTick;                         /* 上一次软启动步进时间 */

  uint32_t lastCurrentSampleTick;                     /* 上一次电流采样时间 */
  uint32_t lastVbatSampleTick;                        /* 上一次电压采样时间 */
  uint32_t lastPrintTick;                             /* 上一次状态打印时间 */
  uint32_t lastDiagTick;                              /* 上一次诊断信息时间 */
  uint32_t lastGreenBlinkTick;                        /* 绿色指示灯节拍 */
  bool greenBlinkLevel;                               /* 绿色指示灯电平 */
  bool greenStallFlashActive;                         /* 堵转绿色提示是否激活 */
  bool greenStallFlashDone;                           /* 堵转绿色提示是否完成 */
  uint32_t greenStallFlashStartTick;                  /* 堵转绿色提示开始时间 */
  uint32_t lastGreenStallFlashTick;                   /* 堵转闪烁节拍 */
  bool prevStallFault;                                /* 上一拍堵转状态，用于边沿检测 */
  bool buttonPressed;                                 /* 按键是否已按下 */
  uint32_t buttonPressTick;                           /* 按键按下时刻 */
  bool buttonLongPressHandled;                        /* 长按是否已处理 */
  bool stopDelayActive;                               /* STOP 停机延时是否生效 */
  uint32_t stopDelayStartTick;                        /* STOP 延时起始时刻 */
  bool stopModeRequested;                             /* 是否已请求进入 STOP */

  uint32_t lastChargeLampTick;                        /* 充电灯节拍 */
  uint8_t chargeLampStep;                             /* 充电灯流水步进 */

  uint32_t lastRedBreathTick;                         /* 红灯呼吸节拍 */
  uint8_t redBreathDuty;                              /* 红灯呼吸占空比 */
  int8_t redBreathDir;                                /* 红灯呼吸方向 */

  bool lowVbatLockLatched;                            /* 低电锁存标志：满足进入条件后置位，满足释放条件后清除 */
  bool prevLowVbatLockLatched;                        /* 上一拍低电锁存状态，用于边沿检测 */
  uint32_t lowVbatEnterStartTick;                     /* 低电进入确认起始 tick */
  bool lowVbatBelowThreshold;                         /* 当前是否处于低压阈值以下 */

  bool lowVbatFlashActive;                            /* 低电提示闪烁进行中 */
  bool lowVbatFlashDone;                              /* 低电提示是否已完成（闪 5 次后置位） */
  uint8_t lowVbatFlashOnCnt;                          /* 低电提示已亮次数 */
  uint32_t lastLowVbatFlashTick;                      /* 低电闪烁节拍 */
  bool lowVbatFlashLevel;                             /* 低电闪烁电平 */

  bool stallFlashActive;                              /* 堵转提示闪烁进行中 */
  bool stallFlashDone;                                /* 堵转提示是否已完成（闪 5 次后置位） */
  uint8_t stallFlashOnCnt;                            /* 堵转提示已亮次数 */
  uint32_t lastStallFlashTick;                        /* 堵转闪烁节拍 */
  bool stallFlashLevel;                               /* 堵转闪烁电平 */
} APP_StateMachine_t;

static APP_StateMachine_t app;                        /* 应用状态机实例（全局） */

/* Private function prototypes -----------------------------------------------*/
static void APP_GPIO_Init(void);                     /* GPIO 初始化 */
static void APP_ADC1_Init(void);                     /* ADC1 初始化 */
static void APP_SysTickAppInit(void);                /* SysTick 业务节拍初始化 */
static void APP_SetMotorPwmDuty(uint8_t duty);       /* 设置 PA0 PWM 占空比 */
static void APP_SetBoostEnable(bool enable);         /* 升压 EN 控制 */
static uint16_t APP_ADC_ReadChannel(uint32_t channel); /* 读取指定 ADC 通道 */
static uint32_t APP_AdcToMilliVolt(uint16_t adc);    /* ADC 码值转毫伏 */
static uint32_t APP_IsenseToMilliAmp(uint16_t adc);  /* 电流采样码值转毫安 */
static uint32_t APP_VbatToMilliVolt(uint16_t adc);   /* 电池采样码值转毫伏 */
static uint16_t APP_IsenseCalibrateOffset(void);     /* 电流 ADC 零点校准 */
static void APP_ProgVerifyBlink(void);               /* 上电程序运行指示 */
void APP_SysTickAppTick(void);                       /* SysTick 业务节拍回调 */
static void APP_EnterStopMode(void);                 /* 进入 STOP 模式 */
static void APP_WakeupFromStop(void);                /* 从 STOP 模式唤醒后恢复 */
static void SystemClock_Config(void);                /* 系统时钟配置（唤醒后重配） */

/**
  * @brief  ADC MSP Init
  * @param  hadc ADC handle
  */
void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1)                            /* 仅初始化 ADC1 */
  {
    GPIO_InitTypeDef GPIO_InitStruct;                    /* GPIO 配置结构体 */

    __HAL_RCC_ADC_CLK_ENABLE();                          /* 使能 ADC 时钟 */
    __HAL_RCC_GPIOA_CLK_ENABLE();                        /* 使能 GPIOA 时钟 */

    GPIO_InitStruct.Pin = GPIO_PIN_3 | GPIO_PIN_6;      /* PA3/PA6 作为模拟输入 */
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;             /* 模拟模式 */
    GPIO_InitStruct.Pull = GPIO_NOPULL;                  /* 无上下拉 */
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);              /* 初始化 GPIO */
  }
}

/* SysTick 业务节拍初始化，直接复用 HAL 的 1ms Tick */
static void APP_SysTickAppInit(void)
{
  /* 当前工程直接使用 HAL 已配置好的 SysTick，无需额外处理 */
}

/* SysTick 业务节拍回调 */
void APP_SysTickAppTick(void)
{
  static uint8_t currentDiv = 0U;
  static uint8_t vbatDiv = 0U;

  gTaskTickMs++;

  currentDiv++;
  if (currentDiv >= CURRENT_SAMPLE_PERIOD_MS)          /* 电流采样分频，到周期后置位采样标志 */
  {
    currentDiv = 0U;
    gCurrentSampleFlag = true;
  }

  vbatDiv++;
  if (vbatDiv >= VBAT_SAMPLE_PERIOD_MS)                /* 电池电压采样分频，到周期后置位采样标志 */
  {
    vbatDiv = 0U;
    gVbatSampleFlag = true;
  }
}

/**
  * @brief  Application Entry Function
  * @retval int
  */
int main(void)
{
  HAL_Init();                                            /* HAL 初始化 */

  APP_GPIO_Init();                                       /* 先初始化 GPIO */

  APP_SysTickAppInit();                                  /* 初始化 SysTick 业务节拍（复用 HAL 1ms） */
  BSP_TIM1_PWM_Init((uint16_t)((HAL_RCC_GetHCLKFreq() / 1000000U) - 1U), 99U); /* PA0 硬件 PWM，10kHz */

#if APP_ADC_ENABLE
  APP_ADC1_Init();                                       /* 按开关决定是否初始化 ADC */
#endif

#if APP_UART_DEBUG_ENABLE
  DEBUG_USART_Config(115200);                            /* 初始化调试串口 115200 */
  printf("app start\r\n");                            /* 打印启动标识 */
#endif

  APP_ProgVerifyBlink();                                 /* 上电闪灯提示程序在运行 */

  app = (APP_StateMachine_t){0};                         /* 应用状态机实例 */

  app.buttonState = APP_BUTTON_DISCONNECT;
  app.prevButtonState = APP_BUTTON_DISCONNECT;
  app.systemState = APP_SYS_NORMAL;
  app.lampState = 0U;
  app.motorState = APP_MOTOR_STOP;
  app.motorStopReason = APP_STOP_REASON_NONE;
  app.startRequestLatched = false;
  app.boostEnabled = false;
  app.boostBelowThreshold = false;
  app.boostAboveThreshold = false;
  app.boostThresholdTick = 0U;
  app.motorGear = 0U;
  app.motorTargetDuty = MOTOR_GEAR0_DUTY;
  app.softStartDuty = MOTOR_SOFT_START_DUTY_START;
  app.prevStallFault = false;
  app.stopDelayActive = false;
  app.stopDelayStartTick = 0U;
  app.stopModeRequested = false;
  app.lastDiagTick = 0U;
  app.lastChargeLampTick = 0U;
  app.chargeLampStep = 0U;
  app.buttonPressed = false;
  app.buttonPressTick = 0U;
  app.buttonLongPressHandled = false;

  /* 以下变量用于记录上一次状态，便于调试打印状态变化，APP_UART_DEBUG_ENABLE=0 时被优化掉 */
  APP_MotorState_t prevMotorState = APP_MOTOR_STOP;
  uint8_t prevMotorGear = 0U; 
  APP_ButtonState_t prevButtonState = APP_BUTTON_DISCONNECT;
  bool prevStartRequestLatched = false;
  bool prevStallFault = false;

  APP_DBG_PRINT("[INIT] gpio/timer init done\r\n");
  APP_DBG_PRINT("[INIT] pwm debug mode: gear->duty, output stability only\r\n");

  HAL_Delay(POWERUP_WAIT_MS);                            /* 上电等待一段时间，避免电源未稳定 */
  APP_DBG_PRINT("[INIT] powerup wait done\r\n");

#if APP_ADC_ENABLE
  app.iSenseOffsetAdc = APP_IsenseCalibrateOffset();     /* 电流采样零点校准 */
#endif

  while (1)                                               /* 主循环 */
  {
    uint32_t now = HAL_GetTick();                         /* 获取当前系统 tick，单位 ms */

    /* 读取按钮输入：按下为 true，松开为 false */
    bool buttonRawPressed = (HAL_GPIO_ReadPin(SW_IN1_PORT, SW_IN1_PIN) == SW_ON_LEVEL);
#if APP_CHARGE_TASK_ENABLE
    app.chargeActive = (HAL_GPIO_ReadPin(CHARGE_DET_PORT, CHARGE_DET_PIN) == GPIO_PIN_SET); /* 读取充电检测脚，判断是否正在充电 */
#else
    app.chargeActive = false;                        /* 关闭充电功能时，强制认为未充电 */
#endif

    /* 按键消抖：电平变化后需稳定 50ms 才认可 */
    static bool buttonStablePressed = false;
    static bool buttonLastRaw = false;
    static uint32_t buttonLastChangeTick = 0U;

    if (buttonRawPressed != buttonLastRaw)
    {
      buttonLastRaw = buttonRawPressed;
      buttonLastChangeTick = now;
    }

    if ((now - buttonLastChangeTick) >= BUTTON_DEBOUNCE_MS)
    {
      buttonStablePressed = buttonLastRaw;
    }

    /* 充电与按键互斥：充电期间忽略按键输入，避免误触发挡位切换或启停 */
    bool buttonPressedNow = (app.chargeActive) ? false : buttonStablePressed;
    bool buttonReleasedEdge = (app.buttonPressed && (!buttonPressedNow));

    app.buttonState = buttonPressedNow ? APP_BUTTON_CONNECT : APP_BUTTON_DISCONNECT;

    if (buttonPressedNow)
    {
      if (!app.buttonPressed)
      {
        app.buttonPressed = true;
        app.buttonPressTick = now;
        app.buttonLongPressHandled = false;
        /* 按键按下只记录状态，不打印时间，减少串口输出 */
      }
      else if ((!app.buttonLongPressHandled) && ((now - app.buttonPressTick) >= BUTTON_LONG_PRESS_MS))
      {
        app.buttonLongPressHandled = true;
        /* 长按检测只保留逻辑，不单独打印，避免刷屏 */
        if (app.motorState == APP_MOTOR_RUN)
        {
          app.startRequestLatched = false;
          app.motorEnable = false;
          app.softStartActive = false;
          app.softStartDuty = MOTOR_SOFT_START_DUTY_START;
          app.stopDelayActive = true;
          app.stopDelayStartTick = now;
          app.stopModeRequested = false;
          APP_DBG_PRINT("[MOTOR] long press -> stop delay start t=%lu\r\n", (unsigned long)now);
        }
        else
        {
          app.stopDelayActive = false;
          app.stopModeRequested = false;
          app.motorStopReason = APP_STOP_REASON_NONE;
          app.startRequestLatched = true;
          app.motorEnable = true;
          app.motorStartTick = 0U;
          APP_DBG_PRINT("[MOTOR] long press -> start request latched t=%lu\r\n", (unsigned long)now);
        }
      }
    }

    if (buttonReleasedEdge)
    {
      /* 松开沿只记录状态，不打印时间 */
      if (!app.buttonLongPressHandled)
      {
        uint8_t oldGear = app.motorGear;
        app.motorGear++;
        if (app.motorGear >= 3U)
        {
          app.motorGear = 0U;
        }
        APP_DBG_PRINT("[GEAR] shift %u -> %u on button release t=%lu\r\n",
                      (unsigned int)oldGear,
                      (unsigned int)app.motorGear,
                      (unsigned long)now);
      }
      app.buttonPressed = false;
      app.buttonLongPressHandled = false;
    }

    if (app.stopDelayActive && (!app.stopModeRequested) && (app.motorState == APP_MOTOR_STOP) &&
        ((now - app.stopDelayStartTick) >= APP_STOP_DELAY_MS))
    {
      app.stopModeRequested = true;
      APP_EnterStopMode();
      APP_WakeupFromStop();
      app.stopDelayActive = false;
    }


    if (gCurrentSampleFlag || gVbatSampleFlag)           /* SysTick 触发了采样标志，进入周期性采样处理 */
    {
      bool doCurrentSample = gCurrentSampleFlag;
      bool doVbatSample = gVbatSampleFlag;

      gCurrentSampleFlag = false;
      gVbatSampleFlag = false;

#if APP_ADC_ENABLE
      if (doCurrentSample)
      {
        app.lastCurrentSampleTick = now;
        app.iSenseAdcRaw = APP_ADC_ReadChannel(I_SENSE_ADC_CHANNEL);         /* 读取电流采样 ADC 原始值 */
        app.iSenseAdcValid = (app.iSenseAdcRaw >= ISENSE_ADC_VALID_MIN) && (app.iSenseAdcRaw <= ISENSE_ADC_VALID_MAX);
        {
          uint16_t iSenseCompAdc = 0U;
          uint32_t currentmA = 0U;

          if (app.iSenseAdcValid)
          {
            /* 用校准零点修正原始值，再换算成电流值 */
            iSenseCompAdc = (app.iSenseAdcRaw > app.iSenseOffsetAdc) ?
                            (uint16_t)(app.iSenseAdcRaw - app.iSenseOffsetAdc) : 0U;
            currentmA = APP_IsenseToMilliAmp(iSenseCompAdc);
          }
          else
          {
            /* 采样异常时沿用上一次滤波结果，避免突变 */
            currentmA = app.currentmAFiltered;
          }

          if (app.currentmAFiltered == 0U)
          {
            app.currentmAFiltered = currentmA;
          }
          else
          {
            /* 一阶低通滤波，平滑电流波动 */
            int32_t delta = (int32_t)currentmA - (int32_t)app.currentmAFiltered;
            app.currentmAFiltered = (uint32_t)((int32_t)app.currentmAFiltered +
                                   (delta >> ISENSE_LPF_SHIFT));
          }
        }
      }

      if (doVbatSample)
      {
        app.lastVbatSampleTick = now;
        app.vBatAdcRaw = APP_ADC_ReadChannel(VBAT_SENSE_ADC_CHANNEL);        /* 读取电池电压采样 ADC 原始值 */
        app.vbatmV = APP_VbatToMilliVolt(app.vBatAdcRaw);                    /* 换算为毫伏 */
      }
#else
      app.iSenseAdcRaw = 0U;
      app.vBatAdcRaw = 0U;
      app.iSenseAdcValid = true;
      app.currentmAFiltered = 0U;
      app.vbatmV = 0U;
#endif

      /* 过流/堵转保护：仅在电机运行时检测，并按挡位使用不同阈值 */
#if APP_STALL_PROTECT_ENABLE
      if (app.motorState == APP_MOTOR_RUN)
      {
        bool stallDetectBlanking = ((now - app.motorStartTick) < STALL_DETECT_BLANKING_MS);
        uint32_t currentLimit = CURRENT_LIMIT_MA_GEAR2;

        if (app.motorGear == 0U)
        {
          currentLimit = CURRENT_LIMIT_MA_GEAR0;
        }
        else if (app.motorGear == 1U)
        {
          currentLimit = CURRENT_LIMIT_MA_GEAR1;
        }

        /* 调试日志：每次进入堵转判断都打印当前条件，方便确认卡在哪一步 */
        APP_DBG_PRINT("[STALL-CHECK] gear=%u current=%lu limit=%lu cnt=%u valid=%u blanking=%u state=%u\r\n",
                      (unsigned int)app.motorGear,
                      (unsigned long)app.currentmAFiltered,
                      (unsigned long)currentLimit,
                      (unsigned int)app.overCurrentCnt,
                      app.iSenseAdcValid ? 1U : 0U,
                      stallDetectBlanking ? 1U : 0U,
                      (unsigned int)app.motorState);

        if (stallDetectBlanking || (!app.iSenseAdcValid))
        {
          /* 启动初期或采样无效时，不计入过流判断 */
          app.overCurrentCnt = 0U;
        }
        else
        {
          if (app.currentmAFiltered >= currentLimit)
          {
            if (app.overCurrentCnt < 0xFFU)
            {
              app.overCurrentCnt++;
            }
            APP_DBG_PRINT("[STALL-HIT] gear=%u current=%lu limit=%lu cnt=%u\r\n",
                          (unsigned int)app.motorGear,
                          (unsigned long)app.currentmAFiltered,
                          (unsigned long)currentLimit,
                          (unsigned int)app.overCurrentCnt);
            if (app.overCurrentCnt >= OVERCURRENT_CONFIRM_CNT)
            {
              app.stallFault = true;
            }
          }
          else
          {
            app.overCurrentCnt = 0U;
          }
        }
      }
#else
      app.stallFault = false;
      app.overCurrentCnt = 0U;
#endif

      /* 低电压保护：低于阈值持续一段时间后锁定 */
#if APP_LOW_VBAT_PROTECT_ENABLE
      if (!app.stallFault)
      {
        if (app.vbatmV < LOW_VBAT_LOCK_MV)
        {
          if (!app.lowVbatBelowThreshold)
          {
            app.lowVbatBelowThreshold = true;
            app.lowVbatEnterStartTick = now;
          }
          else if ((now - app.lowVbatEnterStartTick) >= LOW_VBAT_ENTER_CONFIRM_MS)
          {
            app.lowVbatLockLatched = true;
          }
        }
        else
        {
          app.lowVbatBelowThreshold = false;

          if (app.vbatmV >= LOW_VBAT_RELEASE_MV)
          {
            app.lowVbatLockLatched = false;
          }
        }
      }
#else
      app.lowVbatBelowThreshold = false;
      app.lowVbatLockLatched = false;
#endif

      /* 充电满电判断：充电且电压达到满电阈值时置位 */
      if (app.chargeActive && (app.vbatmV >= VBAT_FULL_MV))
      {
        app.fullCharged = true;
      }
      else if (!app.chargeActive)
      {
        app.fullCharged = false;
      }
      else if (app.vbatmV <= VBAT_FULL_RELEASE_MV)
      {
        app.fullCharged = false;
      }

      /* 5) 系统状态：低电优先，其次充电，其次正常/空闲 */
      if (app.lowVbatLockLatched)
      {
        app.systemState = APP_SYS_LOW_BATTERY;
      }
      else if (app.chargeActive)
      {
        app.systemState = APP_SYS_CHARGING;
      }
      else if (app.buttonState == APP_BUTTON_CONNECT)
      {
        app.systemState = APP_SYS_NORMAL;
      }
      else
      {
        app.systemState = APP_SYS_IDLE;
      }

      if ((now - app.lastPrintTick) >= 3000U)
      {
        app.lastPrintTick = now;
        APP_DBG_PRINT("[CURRENT] current=%lumA adc=%u valid=%u vbat=%lumV\r\n",
                      (unsigned long)app.currentmAFiltered,
                      (unsigned int)app.iSenseAdcRaw,
                      app.iSenseAdcValid ? 1U : 0U,
                      (unsigned long)app.vbatmV);
      }

      /* 低电锁定进入时，启动 5 次三灯闪烁提示 */
      if (app.lowVbatLockLatched && (!app.prevLowVbatLockLatched))
      {
        app.lowVbatFlashActive = true;
        app.lowVbatFlashDone = false;
        app.lowVbatFlashOnCnt = 0U;
        app.lowVbatFlashLevel = false;
        app.lastLowVbatFlashTick = now;
        APP_DBG_PRINT("[LOWV] lock entered t=%lu vbat=%lu mV\r\n",
                      (unsigned long)now,
                      (unsigned long)app.vbatmV);
      }
      app.prevLowVbatLockLatched = app.lowVbatLockLatched;

      /* 堵转进入时，启动 5 次当前挡位灯闪烁提示 */
      if (app.stallFault && (!app.prevStallFault))
      {
        app.stallFlashActive = true;
        app.stallFlashDone = false;
        app.stallFlashOnCnt = 0U;
        app.stallFlashLevel = false;
        app.lastStallFlashTick = now;
        APP_DBG_PRINT("[STALL] fault entered t=%lu gear=%u\r\n",
                      (unsigned long)now,
                      (unsigned int)app.motorGear);
      }
      else if ((!app.stallFault) && app.prevStallFault)
      {
        app.stallFlashActive = false;
        app.stallFlashDone = false;
        app.stallFlashOnCnt = 0U;
        app.stallFlashLevel = false;
      }
      app.prevStallFault = app.stallFault;

#if APP_BOOST_ENABLE
      /* 升压控制：低于 3.6V 持续一段时间才开启，高于 3.7V 持续一段时间才关闭 */
      if (app.vbatmV <= BOOST_EN_ON_MV)
      {
        if (!app.boostBelowThreshold)
        {
          app.boostBelowThreshold = true;
          app.boostThresholdTick = now;
        }
        else if ((!app.boostEnabled) && ((now - app.boostThresholdTick) >= BOOST_ENTER_CONFIRM_MS)
                 && ((now - app.boostMotorOnTick) >= BOOST_MOTOR_START_DELAY_MS))
        {
          APP_SetBoostEnable(true);
          APP_DBG_PRINT("[BOOST] enable vbat=%lu mV charge=%u\r\n",
                        (unsigned long)app.vbatmV,
                        app.chargeActive ? 1U : 0U);
        }
      }
      else
      {
        app.boostBelowThreshold = false;

        if (app.boostEnabled && (app.vbatmV >= BOOST_EN_OFF_MV))
        {
          if (!app.boostAboveThreshold)
          {
            app.boostAboveThreshold = true;
            app.boostThresholdTick = now;
          }
          else if ((now - app.boostThresholdTick) >= BOOST_RELEASE_CONFIRM_MS)
          {
            APP_SetBoostEnable(false);
            APP_DBG_PRINT("[BOOST] disable vbat=%lu mV charge=%u\r\n",
                          (unsigned long)app.vbatmV,
                          app.chargeActive ? 1U : 0U);
          }
        }
        else
        {
          app.boostAboveThreshold = false;
        }
      }
#else
      /* 关闭升压功能时，强制拉低 EN */
      APP_SetBoostEnable(false);
#endif
      /* 6) 电机状态机 */
      if (app.lowVbatLockLatched)
      {
        if (app.motorState != APP_MOTOR_STOP)
        {
          APP_DBG_PRINT("[MOTOR] force stop by low voltage t=%lu\r\n", (unsigned long)now);
        }
        app.motorState = APP_MOTOR_STOP;
        app.motorStopReason = APP_STOP_REASON_LOW_BATTERY;
        app.startRequestLatched = false;
        app.softStartDuty = MOTOR_SOFT_START_DUTY_START;
        app.motorEnable = false;
        app.boostMotorOnTick = 0U;
      }
      else if (app.stallFault)
      {
        if (app.motorState != APP_MOTOR_OVERCURRENT)
        {
          APP_DBG_PRINT("[MOTOR] force stop by stall/overcurrent t=%lu\r\n", (unsigned long)now);
        }
        app.motorState = APP_MOTOR_OVERCURRENT;
        app.motorStopReason = APP_STOP_REASON_STALL;
        app.startRequestLatched = false;
        app.softStartActive = false;                     /* ???????? */
        app.softStartDuty = MOTOR_SOFT_START_DUTY_START;
        app.motorEnable = false;
        app.boostMotorOnTick = 0U;
      }
      else if (app.startRequestLatched)
      {
        if (app.motorState != APP_MOTOR_RUN)
        {
          APP_DBG_PRINT("[MOTOR] enter run state t=%lu gear=%u\r\n",
                        (unsigned long)now,
                        (unsigned int)app.motorGear);
        }
        app.motorState = APP_MOTOR_RUN;
        if (app.motorGear == 0U)
        {
          app.motorTargetDuty = MOTOR_GEAR0_DUTY;
        }
        else if (app.motorGear == 1U)
        {
          app.motorTargetDuty = MOTOR_GEAR1_DUTY;
        }
        else
        {
          app.motorTargetDuty = MOTOR_GEAR2_DUTY;
        }
        /* 取消软启动：运行后直接输出目标占空比，方便排查电机抖动原因 */
        app.softStartActive = false;
        app.softStartDuty = app.motorTargetDuty;
        if (app.motorStartTick == 0U)
        {
          app.motorStartTick = now;
        }
        app.boostMotorOnTick = now;
      }
      else
      {
        if (app.motorState != APP_MOTOR_STOP)
        {
          APP_DBG_PRINT("[MOTOR] stop requested t=%lu\r\n", (unsigned long)now);
        }
        app.motorState = APP_MOTOR_STOP;
        app.motorTargetDuty = 0U;
        app.boostMotorOnTick = 0U;
      }
    }

      /* 电机运行时执行软启动，完成后自动退出软启动状态 */
      if (app.motorState == APP_MOTOR_RUN)
      {
        if ((now - app.lastSoftStartTick) >= MOTOR_SOFT_START_STEP_MS)
        {
          app.lastSoftStartTick = now;
          /* 软启动已关闭，这里不再做占空比递增 */
          app.softStartDuty = app.motorTargetDuty;
        }

        app.motorEnable = true;
        APP_SetMotorPwmDuty(app.softStartActive ? app.softStartDuty : app.motorTargetDuty); /* 输出软启动或目标占空比 */
      }
      else
      {
        app.motorEnable = false;
        APP_SetMotorPwmDuty(0U);                       /* ?????????? */
      }

      app.prevButtonState = app.buttonState;

      /* 指示灯逻辑：
         1) 充电时：PB0 -> PB1 -> PB3 循环流水
         2) 满电时：PB0+PB1 与 PB0+PB3 交替
         3) 低电时：三个挡位灯一起闪烁 5 次后熄灭
         4) 堵转时：当前挡位灯闪烁，其余挡位灯保持常亮
         5) 正常时：按挡位点亮 */
      if (app.lowVbatLockLatched)
      {
        if (app.lowVbatFlashDone)
        {
          app.lampState = 0x00U;
        }
        else if ((now - app.lastLowVbatFlashTick) >= LOW_VBAT_BLINK_MS)
        {
          app.lastLowVbatFlashTick = now;
          app.lowVbatFlashLevel = !app.lowVbatFlashLevel;
          if (app.lowVbatFlashLevel)
          {
            if (app.lowVbatFlashOnCnt < 0xFFU)
            {
              app.lowVbatFlashOnCnt++;
            }
            if (app.lowVbatFlashOnCnt >= LOW_VBAT_BLINK_TIMES)
            {
              app.lowVbatFlashDone = true;
              app.lowVbatFlashLevel = false;
            }
          }
        }
        app.lampState = app.lowVbatFlashLevel ? 0x07U : 0x00U;
      }
      else if (app.stallFault)
      {
        if (app.stallFlashDone)
        {
          app.lampState = 0x00U;
          app.stallFault = false;
          app.motorState = APP_MOTOR_STOP;
          app.motorStopReason = APP_STOP_REASON_STALL;
          app.startRequestLatched = false;
        }
        else if ((now - app.lastStallFlashTick) >= LOW_VBAT_BLINK_MS)
        {
          app.lastStallFlashTick = now;
          app.stallFlashLevel = !app.stallFlashLevel;
          if (app.stallFlashLevel)
          {
            if (app.stallFlashOnCnt < 0xFFU)
            {
              app.stallFlashOnCnt++;
            }
            if (app.stallFlashOnCnt >= LOW_VBAT_BLINK_TIMES)
            {
              app.stallFlashDone = true;
              app.stallFlashLevel = false;
            }
          }
        }

        if (app.motorGear == 0U)
        {
          app.lampState = app.stallFlashLevel ? 0x01U : 0x00U;
        }
        else if (app.motorGear == 1U)
        {
          app.lampState = app.stallFlashLevel ? 0x01U : 0x03U;
        }
        else
        {
          app.lampState = app.stallFlashLevel ? 0x03U : 0x07U;
        }
      }
      else if (app.systemState == APP_SYS_CHARGING)
      {
        if (app.fullCharged)
        {
          /* ??????010 -> 101 */
          if ((app.chargeLampStep & 0x01U) == 0U)
          {
            app.lampState = 0x02U;
          }
          else
          {
            app.lampState = 0x05U;
          }
        }
        else
        {
          /* ??????001 -> 010 -> 100 */
          switch (app.chargeLampStep % 3U)
          {
            case 0U:
              app.lampState = 0x01U;
              break;
            case 1U:
              app.lampState = 0x02U;
              break;
            default:
              app.lampState = 0x04U;
              break;
          }
        }

        if ((now - app.lastChargeLampTick) >= 300U)
        {
          app.lastChargeLampTick = now;
          app.chargeLampStep++;
        }
      }
      else if (app.motorState == APP_MOTOR_RUN)
      {
        if (app.motorGear == 0U)
        {
          app.lampState = 0x01U;
        }
        else if (app.motorGear == 1U)
        {
          app.lampState = 0x03U;
        }
        else
        {
          app.lampState = 0x07U;
        }
      }
      else
      {
        app.lampState = 0U;
      }

      HAL_GPIO_WritePin(LAMP1_PORT, LAMP1_PIN, (app.lampState & 0x01U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
      HAL_GPIO_WritePin(LAMP2_PORT, LAMP2_PIN, (app.lampState & 0x02U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
      HAL_GPIO_WritePin(LAMP3_PORT, LAMP3_PIN, (app.lampState & 0x04U) ? GPIO_PIN_SET : GPIO_PIN_RESET);

      /* PWM 输出由 BSP 驱动到 PA0 */
      if (app.motorState == APP_MOTOR_RUN)
      {
        uint8_t duty = app.softStartActive ? app.softStartDuty : app.motorTargetDuty;
        BSP_TIM1_PWM_SetDuty(duty);
      }
      else
      {
        BSP_TIM1_PWM_SetDuty(0U);
      }
#if APP_UART_DEBUG_ENABLE
      if ((now - app.lastPrintTick) >= 3000U)
      {
        app.lastPrintTick = now;
        APP_DBG_PRINT("[CURRENT] current=%lumA adc=%u valid=%u vbat=%lumV\r\n",
                      (unsigned long)app.currentmAFiltered,
                      (unsigned int)app.iSenseAdcRaw,
                      app.iSenseAdcValid ? 1U : 0U,
                      (unsigned long)app.vbatmV);

        if (prevMotorGear != app.motorGear)
        {
          APP_DBG_PRINT("[GEAR] shift %u -> %u, targetDuty=%u, pwmOut=%u\r\n",
                        (unsigned int)prevMotorGear,
                        (unsigned int)app.motorGear,
                        (unsigned int)app.motorTargetDuty,
                        (unsigned int)(app.softStartActive ? app.softStartDuty : app.motorTargetDuty));
        }

        if (prevButtonState != app.buttonState)
        {
          APP_DBG_PRINT("[BTN] state %d -> %d, pressed=%u charge=%u longHandled=%u\r\n",
                        (int)prevButtonState,
                        (int)app.buttonState,
                        app.buttonPressed ? 1U : 0U,
                        app.chargeActive ? 1U : 0U,
                        app.buttonLongPressHandled ? 1U : 0U);
        }

        if (prevMotorState != app.motorState)
        {
          APP_DBG_PRINT("[MOTOR] state %d -> %d, gear=%u, duty=%u, pwmOut=%u, enable=%u\r\n",
                        (int)prevMotorState,
                        (int)app.motorState,
                        (unsigned int)app.motorGear,
                        (unsigned int)app.motorTargetDuty,
                        (unsigned int)(app.softStartActive ? app.softStartDuty : app.motorTargetDuty),
                        app.motorEnable ? 1U : 0U);
        }

        if (prevStartRequestLatched != app.startRequestLatched)
        {
          APP_DBG_PRINT("[START] latched %d -> %d\r\n",
                        prevStartRequestLatched ? 1 : 0,
                        app.startRequestLatched ? 1 : 0);
        }

        if (prevStallFault != app.stallFault)
        {
          APP_DBG_PRINT("[STALL] fault %u -> %u, gear=%u, current=%lumA, limit=%lu\r\n",
                        prevStallFault ? 1U : 0U,
                        app.stallFault ? 1U : 0U,
                        (unsigned int)app.motorGear,
                        (unsigned long)app.currentmAFiltered,
                        (unsigned long)((app.motorGear == 0U) ? CURRENT_LIMIT_MA_GEAR0 :
                                        (app.motorGear == 1U) ? CURRENT_LIMIT_MA_GEAR1 :
                                        CURRENT_LIMIT_MA_GEAR2));
        }

        prevButtonState = app.buttonState;
        prevMotorState = app.motorState;
        prevMotorGear = app.motorGear;
        prevStartRequestLatched = app.startRequestLatched;
        prevStallFault = app.stallFault;
      }
#endif
  }
}

/* GPIO 初始化 ----------------------------------------------------------------*/
static void APP_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;                     /* GPIO 初始化结构体 */

  __HAL_RCC_GPIOA_CLK_ENABLE();                         /* 使能 GPIOA 时钟 */
  __HAL_RCC_GPIOB_CLK_ENABLE();                         /* 使能 GPIOB 时钟 */
  __HAL_RCC_GPIOC_CLK_ENABLE();                         /* 使能 GPIOC 时钟 */

  /* 电机 PWM：PA0 / TIM1_CH1 */
  GPIO_InitStruct.Pin = MOTOR_EN_PIN;                   /* PA0 */
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;               /* 复用推挽 */
  GPIO_InitStruct.Pull = GPIO_NOPULL;                   /* 无上下拉 */
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;         /* 高速 */
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM1;            /* AF2 = TIM1_CH1 */
  HAL_GPIO_Init(MOTOR_EN_PORT, &GPIO_InitStruct);       /* 初始化 */

  /* 升压使能：PA5 */
  GPIO_InitStruct.Pin = BOOST_EN_PIN;                   /* PA5 */
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;           /* 推挽输出 */
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;                 /* 下拉 */
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;         /* 高速 */
  HAL_GPIO_Init(BOOST_EN_PORT, &GPIO_InitStruct);       /* 初始化 */
  HAL_GPIO_WritePin(BOOST_EN_PORT, BOOST_EN_PIN, GPIO_PIN_RESET); /* 默认关闭 */

  /* 指示灯：PB0/PB1/PB3 */
  GPIO_InitStruct.Pin = LAMP1_PIN;                      /* PB0 */
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;           /* 推挽输出 */
  GPIO_InitStruct.Pull = GPIO_NOPULL;                   /* 无上下拉 */
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;         /* 高速 */
  HAL_GPIO_Init(LAMP1_PORT, &GPIO_InitStruct);          /* 初始化 */
  HAL_GPIO_WritePin(LAMP1_PORT, LAMP1_PIN, GPIO_PIN_RESET); /* 默认熄灭 */

  GPIO_InitStruct.Pin = LAMP2_PIN;                      /* PB1 */
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;           /* 推挽输出 */
  GPIO_InitStruct.Pull = GPIO_NOPULL;                   /* 无上下拉 */
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;         /* 高速 */
  HAL_GPIO_Init(LAMP2_PORT, &GPIO_InitStruct);          /* 初始化 */
  HAL_GPIO_WritePin(LAMP2_PORT, LAMP2_PIN, GPIO_PIN_RESET); /* 默认熄灭 */

  GPIO_InitStruct.Pin = LAMP3_PIN;                      /* PB3 */
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;           /* 推挽输出 */
  GPIO_InitStruct.Pull = GPIO_NOPULL;                   /* 无上下拉 */
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;         /* 高速 */
  HAL_GPIO_Init(LAMP3_PORT, &GPIO_InitStruct);          /* 初始化 */
  HAL_GPIO_WritePin(LAMP3_PORT, LAMP3_PIN, GPIO_PIN_RESET); /* 默认熄灭 */

  /* 按键输入：PB7，低电平有效 */
  GPIO_InitStruct.Pin = SW_IN1_PIN;                     /* PB7 */
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;               /* 输入模式 */
  GPIO_InitStruct.Pull = GPIO_PULLUP;                   /* 上拉 */
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;         /* 高速 */
  HAL_GPIO_Init(SW_IN1_PORT, &GPIO_InitStruct);         /* 初始化 */

  /* 充电检测：PC1 */
  GPIO_InitStruct.Pin = CHARGE_DET_PIN;                 /* PC1 */
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;               /* 输入模式 */
  GPIO_InitStruct.Pull = GPIO_NOPULL;                   /* 无上下拉 */
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;         /* 高速 */
  HAL_GPIO_Init(CHARGE_DET_PORT, &GPIO_InitStruct);     /* 初始化 */

  /* TP4056 CHRG：PB2，保留输入 */
  GPIO_InitStruct.Pin = TP4056_CHRG_PIN;                /* PB2 */
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;               /* 输入模式 */
  GPIO_InitStruct.Pull = GPIO_NOPULL;                   /* 无上下拉 */
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;         /* 高速 */
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);               /* 初始化 */

  /* 关闭外部中断，当前工程不使用 */
  HAL_NVIC_DisableIRQ(EXTI4_15_IRQn);
  HAL_NVIC_DisableIRQ(EXTI0_1_IRQn);
}

/* ADC 初始化 -----------------------------------------------------------------*/
static void APP_ADC1_Init(void)
{
  hadc1.Instance = ADC1;                                /* ADC1 实例 */
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2; /* ADC 时钟分频 */
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;           /* 12 位分辨率 */
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;           /* 右对齐 */
  hadc1.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;  /* 扫描方向前进 */
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;        /* 单次转换完成 */
  hadc1.Init.LowPowerAutoWait = DISABLE;                /* 关闭低功耗自动等待 */
  hadc1.Init.ContinuousConvMode = DISABLE;              /* 关闭连续转换 */
  hadc1.Init.DiscontinuousConvMode = DISABLE;           /* 关闭不连续转换 */
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;     /* 软件触发 */
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE; /* 无外部触发边沿 */
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;        /* 覆盖旧数据 */
  hadc1.Init.SamplingTimeCommon = ADC_SAMPLETIME_239CYCLES_5; /* 采样时间较长 */

  if (HAL_ADC_Init(&hadc1) != HAL_OK)                   /* 初始化 ADC */
  {
    APP_ErrorHandler();                                  /* 初始化失败 */
  }

  if (HAL_ADC_ConfigVrefBuf(&hadc1, ADC_VREFBUF_1P5V) != HAL_OK) /* 配置 1.5V 参考 */
  {
    APP_ErrorHandler();                                  /* 初始化失败 */
  }

  if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK)    /* 启动 ADC 校准 */
  {
    APP_ErrorHandler();                                  /* 初始化失败 */
  }
}

/* 读取指定 ADC 通道 ------------------------------------------------------------*/
static uint16_t APP_ADC_ReadChannel(uint32_t channel)
{
  ADC_ChannelConfTypeDef sConfig;                       /* 通道配置结构体 */

  hadc1.Instance->CHSELR = 0U;                          /* 清空通道选择 */

  sConfig.Channel = channel;                            /* 选择通道 */
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;               /* 通道 rank */

  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)/* 配置通道失败 */
  {
    return 0;                                           /* 返回 0 */
  }

  if (HAL_ADC_Start(&hadc1) != HAL_OK)                  /* 启动 ADC 失败 */
  {
    return 0;                                           /* 返回 0 */
  }

  if (HAL_ADC_PollForConversion(&hadc1, 10U) != HAL_OK) /* 等待转换完成失败 */
  {
    HAL_ADC_Stop(&hadc1);                               /* 停止 ADC */
    return 0;                                           /* 返回 0 */
  }

  (void)HAL_ADC_GetValue(&hadc1);                       /* 丢弃第一次结果 */
  HAL_ADC_Stop(&hadc1);                                 /* 停止 ADC */

  if (HAL_ADC_Start(&hadc1) != HAL_OK)                  /* 再次启动 ADC */
  {
    return 0;                                           /* 返回 0 */
  }

  if (HAL_ADC_PollForConversion(&hadc1, 10U) != HAL_OK) /* 等待转换完成失败 */
  {
    HAL_ADC_Stop(&hadc1);                               /* 停止 ADC */
    return 0;                                           /* 返回 0 */
  }

  uint16_t value = (uint16_t)HAL_ADC_GetValue(&hadc1); /* 读取 ADC 值 */
  HAL_ADC_Stop(&hadc1);                                 /* 停止 ADC */
  return value;                                         /* 返回结果 */
}

/* ADC???????? --------------------------------------------------------*/
static uint32_t APP_AdcToMilliVolt(uint16_t adc)
{
  return ((uint32_t)adc * ADC_VREF_MV) / 4095U;        /* 12????4095 */
}

/* ??ADC??????----------------------------------------------------------*/
static uint32_t APP_IsenseToMilliAmp(uint16_t adc)
{
  return ((uint32_t)adc * CURRENT_MA_NUM) / CURRENT_MA_DEN; /* ??m???.5V?? */
}

/* ??ADC????????------------------------------------------------------*/
static uint32_t APP_VbatToMilliVolt(uint16_t adc)
{
  uint32_t adcMv = APP_AdcToMilliVolt(adc);             /* ??ADC????*/
  return (adcMv * VBAT_DIV_NUM) / VBAT_DIV_DEN;         /* ????????????*/
}

/* ????????????????????------------------------------------*/
static uint16_t APP_IsenseCalibrateOffset(void)
{
  uint16_t attempt;
  uint16_t lastValid = 0U;

  HAL_GPIO_WritePin(MOTOR_EN_PORT, MOTOR_EN_PIN, GPIO_PIN_RESET); /* ??????????*/
  HAL_Delay(50U);                                       /* ???????ADC?? */

  for (attempt = 0U; attempt < ISENSE_OFFSET_RETRY_MAX; attempt++) /* ????????????*/
  {
    uint32_t sum = 0U;                                  /* ?????? */
    uint16_t validCnt = 0U;
    uint16_t i;

    for (i = 0U; i < ISENSE_OFFSET_SAMPLES; i++)        /* ????????*/
    {
      uint16_t adc = APP_ADC_ReadChannel(I_SENSE_ADC_CHANNEL);

      if ((adc >= ISENSE_ADC_VALID_MIN) && (adc <= ISENSE_ADC_VALID_MAX)) /* ??0/4095?? */
      {
        sum += adc;
        validCnt++;
      }

      HAL_Delay(ISENSE_OFFSET_SAMPLE_DELAY_MS);         /* ???? */
    }

    if (validCnt >= (ISENSE_OFFSET_SAMPLES / 2U))       /* ??????????? */
    {
      lastValid = (uint16_t)(sum / validCnt);
      break;
    }
  }

  return lastValid;                                     /* ????0??????????*/
}

/* ?????? --------------------------------------------------------------*/
static void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /* ????HSI 24MHz?AHB/APB????*/
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_24MHz;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    APP_ErrorHandler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSISYS;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    APP_ErrorHandler();
  }
}

/* ??PA0??PWM????------------------------------------------------------*/
static void APP_SetMotorPwmDuty(uint8_t duty)
{
  BSP_TIM1_PWM_SetDuty(duty);
  app.motorTargetDuty = duty;
}

static void APP_SetBoostEnable(bool enable)
{
#if APP_BOOST_ENABLE
  app.boostEnabled = enable;
  HAL_GPIO_WritePin(BOOST_EN_PORT, BOOST_EN_PIN, enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
#else
  app.boostEnabled = false;
  HAL_GPIO_WritePin(BOOST_EN_PORT, BOOST_EN_PIN, GPIO_PIN_RESET);
#endif
}

/* ???????? -----------------------------------------------------------*/
static void APP_ProgVerifyBlink(void)
{
  uint8_t i;                                            /* ???? */

  for (i = 0; i < PROG_VERIFY_TOGGLE_TIMES; i++)        /* ?????? */
  {
    HAL_GPIO_TogglePin(LAMP1_PORT, LAMP1_PIN);          /* ??1?? */
    HAL_Delay(PROG_VERIFY_TOGGLE_MS);                   /* ?? */
  }

  HAL_GPIO_WritePin(LAMP1_PORT, LAMP1_PIN, GPIO_PIN_RESET); /* ??????*/
}

/* ??STOP??????---------------------------------------------------------*/
static void APP_EnterStopMode(void)
{
  /* ???????????STOP */
}

/* ?STOP????????-------------------------------------------------------*/
static void APP_WakeupFromStop(void)
{
}

/**
  * @brief  Error executing function
  * @param  None
  * @retval None
  */
void APP_ErrorHandler(void)
{
  while (1)                                              /* ?????????? */
  {
  }
}

#ifdef  USE_FULL_ASSERT                                  /* ???????????*/
void assert_failed(uint8_t *file, uint32_t line)         /* ?????? */
{
  (void)file;                                             /* ????????*/
  (void)line;                                             /* ????????*/
  while (1)                                               /* ????????*/
  {
  }
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT Puya *****END OF FILE******************/
