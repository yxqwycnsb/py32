#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"

#include "a4988.h"
#include "as5600.h"
#include "rtos_sync.h"
#include "usart.h"
#include <stdlib.h>

EventGroupHandle_t xMotorEventGroup = NULL;  // 电机事件组句柄

typedef struct
{
	uint16_t speed_rpm;   // 当前转速
	uint8_t enabled;      // 运行使能状态
} MotorRuntime_t;

static MotorRuntime_t g_motor1 = { 20, 0 };
static MotorRuntime_t g_motor2 = { 20, 0 };

static void motor_pair_init(void)
{
	// 初始化两路步进电机控制器
	a4988_init();
	a4988_motor2_init();
	a4988_set_microstep(MICROSTEP_1_16);
	a4988_enable(true);
	// 电机2先保持关闭，等流程开始后再动作
	a4988_motor2_enable(false);
	g_motor1.enabled = 1;
	g_motor2.enabled = 0;
}

static void motor1_run(uint8_t forward, uint16_t rpm)
{
	// 启动电机1并设置方向
	g_motor1.speed_rpm = rpm;
	g_motor1.enabled = 1;
	a4988_set_direction(forward ? true : false);
	a4988_run_rpm(rpm);
}

static void motor1_stop(void)
{
	// 停止电机1
	a4988_stop_motor();
	g_motor1.enabled = 0;
}

static void motor2_run(uint8_t forward, uint16_t rpm)
{
	// 启动电机2并设置方向
	g_motor2.speed_rpm = rpm;
	g_motor2.enabled = 1;
	a4988_motor2_set_direction(forward ? true : false);
	a4988_motor2_run_rpm(rpm);
}

static void motor2_stop(void)
{
	// 停止电机2
	a4988_motor2_stop();
	g_motor2.enabled = 0;
}

static void _stop_all(void)
{
	// 紧急停止两路电机
	motor1_stop();
	motor2_stop();
}

#define BTN_PORT GPIOB
#define BTN_PIN  GPIO_Pin_1
#define LED_PORT GPIOC
#define LED_PIN  GPIO_Pin_13

static void led_init(void)
{
	// PC13 LED 初始化，常见开发板为低电平点亮
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	GPIO_InitTypeDef gpio;
	gpio.GPIO_Pin = LED_PIN;
	gpio.GPIO_Mode = GPIO_Mode_Out_PP;
	gpio.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(LED_PORT, &gpio);
	GPIO_SetBits(LED_PORT, LED_PIN);
}

static void led_set(bool on)
{
	// PC13 通常为低电平点亮
	if (on) {
		GPIO_ResetBits(LED_PORT, LED_PIN);
	} else {
		GPIO_SetBits(LED_PORT, LED_PIN);
	}
}

static void led_blink(uint32_t now_ms, uint32_t period_ms)
{
	// 按周期翻转 LED，用于状态闪烁显示
	if (period_ms == 0U) {
		led_set(false);
		return;
	}
	led_set(((now_ms / period_ms) & 1U) != 0U);
}

static void led_breath(uint32_t now_ms)
{
	// 用软件方式模拟呼吸灯：低频周期 + 占空比变化
	uint32_t phase = now_ms % 2000U;
	if (phase < 1000U) {
		led_set(phase < (phase / 4U + 1U));
	} else {
		led_set((2000U - phase) < ((2000U - phase) / 4U + 1U));
	}
}

static void button_init(void)
{
	// 按键初始化：PB1 上拉输入，按下为低电平
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitTypeDef gpio;
	gpio.GPIO_Pin = BTN_PIN;
	gpio.GPIO_Mode = GPIO_Mode_IPU;
	gpio.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(BTN_PORT, &gpio);
}

static bool button_pressed(void)
{
	// 按下返回 true
	return GPIO_ReadInputDataBit(BTN_PORT, BTN_PIN) == Bit_RESET;
}

typedef enum
{
	ST_IDLE = 0,
	ST_WAIT_ZERO,
	ST_MONITOR_MOVE,
	ST_RETURN_HOME,
	ST_M2_REPEAT_OUT,
	ST_M2_REPEAT_BACK,
	ST_M1_FINAL_OUT,
	ST_M1_FINAL_BACK,
	ST_M2_RESET_OUT,
	ST_M2_RESET_BACK,
	ST_FINISHED,
	ST_ERROR,
} AppState_t;

static const char *state_to_str(AppState_t state)
{
	switch (state)
	{
	case ST_IDLE: return "IDLE";
	case ST_WAIT_ZERO: return "WAIT_ZERO";
	case ST_MONITOR_MOVE: return "MONITOR";
	case ST_RETURN_HOME: return "RETURN_HOME";
	case ST_M2_REPEAT_OUT: return "M2_OUT";
	case ST_M2_REPEAT_BACK: return "M2_BACK";
	case ST_M1_FINAL_OUT: return "M1_FOUT";
	case ST_M1_FINAL_BACK: return "M1_FBACK";
	case ST_M2_RESET_OUT: return "M2_RST_OUT";
	case ST_M2_RESET_BACK: return "M2_RST_BACK";
	case ST_FINISHED: return "FINISHED";
	case ST_ERROR: return "ERROR";
	default: return "UNKNOWN";
	}
}

static int32_t _angle_diff_raw(uint16_t current, uint16_t previous)
{
	// 处理 0~4095 的回绕，得到跨圈后的真实差值
	int32_t diff = (int32_t)current - (int32_t)previous;
	if (diff > 2048) diff -= 4096;
	if (diff < -2048) diff += 4096;
	return diff;
}

static uint32_t _raw_to_steps(uint32_t raw_delta)
{
	// 将 AS5600 原始角度差换算成步数
	return (uint32_t)(((uint64_t)raw_delta * (uint64_t)a4988_get_steps_per_rev()) / 4096ULL);
}

static bool _within_step_tolerance(int32_t current, int32_t target, uint32_t tolerance_steps)
{
	// 允许少量回差/抖动，避免“必须完全相等”导致状态机卡死
	uint32_t delta_steps = _raw_to_steps((uint32_t)labs((long)(current - target)));
	return delta_steps <= tolerance_steps;
}

static bool i2c1_probe_addr(uint8_t addr)
{
	uint32_t timeout = 0xFFFF;

	while (I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY)) {
		if (timeout-- == 0) return false;
	}

	I2C_GenerateSTART(I2C1, ENABLE);
	timeout = 0xFFFF;
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)) {
		if (timeout-- == 0) {
			I2C_GenerateSTOP(I2C1, ENABLE);
			return false;
		}
	}

	I2C_Send7bitAddress(I2C1, (uint8_t)(addr << 1), I2C_Direction_Transmitter);
	timeout = 0xFFFF;
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {
		if (I2C_GetFlagStatus(I2C1, I2C_FLAG_AF)) {
			I2C_ClearFlag(I2C1, I2C_FLAG_AF);
			I2C_GenerateSTOP(I2C1, ENABLE);
			return false;
		}
		if (timeout-- == 0) {
			I2C_GenerateSTOP(I2C1, ENABLE);
			return false;
		}
	}

	I2C_GenerateSTOP(I2C1, ENABLE);
	return true;
}

static void i2c1_scan_and_report(void)
{
	usart_send_string("[SCAN] Start I2C1 address scan (0x03~0x77)\r\n");
	for (uint8_t addr = 3; addr < 0x78; ++addr)
	{
		if (i2c1_probe_addr(addr))
		{
			usart_send_string("[SCAN] Found device at 0x");
			usart_send_u32(addr);
			usart_send_string("\r\n");
		}
	}
	usart_send_string("[SCAN] Scan done. Expected AS5600 addr = 0x36\r\n");
}

static void PhotoTask(void *arg)
{
	(void)arg;

	// 业务参数：转速、圈数、超时与稳定判定时间
	const uint16_t motor1_rpm = 20;
	const uint16_t motor2_rpm = 20;
	const uint32_t motor1_turn_steps = 3U * a4988_get_steps_per_rev();
	const uint32_t repeat_times = 5U;
	const uint32_t encoder_timeout_ms = 30000;
	const uint32_t move_timeout_ms = 30000;
	const uint32_t encoder_stable_ms = 5000;
	const uint32_t position_tolerance_steps = 2U;

	// 状态机与运行时变量
	static AppState_t app_state = ST_IDLE;
	static uint8_t inited = 0;
	static bool last_btn_state = true;
	static uint32_t t_start = 0;
	static uint32_t t_stable = 0;

	// AS5600 连续角度相关变量
	static uint16_t last_raw = 0;
	static uint16_t start_raw = 0;
	static uint16_t end_raw = 0;
	static int32_t continuous_raw = 0;
	static int32_t start_continuous_raw = 0;
	static int32_t end_continuous_raw = 0;
	static uint32_t measured_steps = 0;
	static uint32_t motor2_target_steps = 0;
	static uint32_t motor2_repeat_count = 0;
	static uint8_t motor1_started = 0;
	static uint8_t motor2_started = 0;
	static uint32_t last_monitor_steps = 0;

	if (!inited)
	{
		motor_pair_init();
		button_init();
		led_init();
		usart_init(115200);

		// AS5600 配置：默认使用 I2C1（PB6/PB7）
		AS5600_Config_t encoder_cfg = {0};
		encoder_cfg.i2c_id = AS5600_I2C1;
		encoder_cfg.scl_port = GPIOB;
		encoder_cfg.sda_port = GPIOB;
		encoder_cfg.scl_pin = GPIO_Pin_6;
		encoder_cfg.sda_pin = GPIO_Pin_7;
		encoder_cfg.dir_enable = true;
		encoder_cfg.dir_port = GPIOB;
		encoder_cfg.dir_pin = GPIO_Pin_0;
		encoder_cfg.dir_level = 0;  // 需要时可改为 1 以反转方向定义
		encoder_cfg.sample_ms = AS5600_DEFAULT_SAMPLE_MS;
		encoder_cfg.degree_offset = AS5600_DEFAULT_DEGREE_OFFSET;
		encoder_cfg.i2c_address = AS5600_DEFAULT_ADDRESS;

		if (as5600_init(&encoder_cfg) != AS5600_OK)
		{
			usart_send_string("[BOOT] ERROR: AS5600 init failed! Check I2C wiring (PB6/PB7)\r\n");
			while (1)
			{
				// 用LED闪烁提示编码器故障：快速闪3次后停1秒循环
				for (int blk = 0; blk < 3; blk++)
				{
					led_set(true);
					vTaskDelay(pdMS_TO_TICKS(200));
					led_set(false);
					vTaskDelay(pdMS_TO_TICKS(200));
				}
				vTaskDelay(pdMS_TO_TICKS(1000));
			}
		}
		usart_send_string("[BOOT] AS5600 init OK\r\n");
		i2c1_scan_and_report();

		// 上电先读取一次编码器，建立连续角度基准
		usart_send_string("[BOOT] Waiting for first encoder reading...\r\n");
		{
			uint32_t retry_start = xTaskGetTickCount();
			uint32_t retry_count = 0;
			uint32_t diag_printed = 0;
			while (!as5600_update())
			{
				vTaskDelay(pdMS_TO_TICKS(10));
				retry_count++;

					// 每 1 秒输出一次诊断日志
					if ((xTaskGetTickCount() - retry_start) >= pdMS_TO_TICKS(2000))
					{
						retry_start = xTaskGetTickCount();
						diag_printed++;

						// 读取 PB6/PB7 实际电平（判决引脚是否被意外拉低）
						uint8_t pb6_level = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_6);
						uint8_t pb7_level = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_7);

						// 读取 I2C1 状态寄存器 SR1 和 SR2
						uint16_t sr1 = I2C1->SR1;
						uint16_t sr2 = I2C1->SR2;

					usart_send_string("[BOOT] retry=");
					usart_send_u32_dec(retry_count);
					usart_send_string(" PB6=");
					usart_send_u32_dec(pb6_level);
					usart_send_string(" PB7=");
					usart_send_u32_dec(pb7_level);
					usart_send_string(" RCC_APB1=0x");
					usart_send_u32(RCC->APB1ENR);
					usart_send_string(" AFIO_MAPR=0x");
					usart_send_u32(AFIO->MAPR);
					usart_send_string(" GPIOB_CRL=0x");
					usart_send_u32(GPIOB->CRL);
					usart_send_string(" I2C_CR1=0x");
					usart_send_u32(*(volatile uint16_t *)&I2C1->CR1);
					usart_send_string(" I2C_SR1=0x");
					usart_send_u32(sr1);
					usart_send_string(" I2C_SR2=0x");
					usart_send_u32(sr2);

					// 检查 PE 位（CR1 bit 0）
					{
						uint16_t cr1 = *(volatile uint16_t *)&I2C1->CR1;
						if (cr1 & 0x0001) usart_send_string(" PE=1");
						else usart_send_string(" PE=0");
						if (cr1 & 0x0100) usart_send_string(" START=1");
						if (cr1 & 0x0200) usart_send_string(" STOP=1");
						if (cr1 & 0x0400) usart_send_string(" ACK=1");
					}

					// 解析关键标志位
					if (sr1 & 0x0400) usart_send_string(" AF=1(NACK)");
					else usart_send_string(" AF=0");
					if (sr1 & 0x0100) usart_send_string(" BERR=1");
					if (sr1 & 0x0200) usart_send_string(" ARLO=1");
					if (sr2 & 0x0002) usart_send_string(" BUSY=1");
					else usart_send_string(" BUSY=0");
					if (sr2 & 0x0001) usart_send_string(" MSL=1");
					if (sr1 == 0 && sr2 == 0) usart_send_string(" IDLE");

					// 根据 PE 位分类故障
					if ((*(volatile uint16_t *)&I2C1->CR1 & 0x0001) == 0) {
						usart_send_string(" | FATAL: I2C PE=0! Peripheral disabled, cannot communicate");
					} else if (sr1 & 0x0400) {
						usart_send_string(" | AF=1: AS5600 not answering! Check I2C addr/wiring/power/magnet");
					} else if (sr2 & 0x0002) {
						usart_send_string(" | BUSY stuck: RESET AS5600 module power");
					} else if (pb6_level == 0 || pb7_level == 0) {
						usart_send_string(" | WARN: SCL/SDA stuck LOW! Check for short to GND");
					} else {
						usart_send_string(" | I2C bus idle but AS5600 unreachable. Check module power & connections");
					}

					usart_send_string("\r\n");
					}
			}
		}
		usart_send_string("[BOOT] First encoder reading OK, entering main loop\r\n");

		last_raw = (uint16_t)as5600_read_angle_raw();
		continuous_raw = 0;
		start_raw = last_raw;
		end_raw = last_raw;
		start_continuous_raw = 0;
		end_continuous_raw = 0;
		last_monitor_steps = 0;
		last_btn_state = button_pressed();
		inited = 1;
	}

	for (;;)
	{
		uint32_t now = xTaskGetTickCount();
		bool encoder_ok = as5600_update();
		uint16_t encoder_raw = (uint16_t)as5600_read_angle_raw();

		// 更新连续角度：解决 AS5600 0~360° 回绕问题
		if (encoder_ok)
		{
			int32_t diff = _angle_diff_raw(encoder_raw, last_raw);
			continuous_raw += diff;
			last_raw = encoder_raw;
		}

		// 只要角度还在变化，就刷新稳定计时
		uint32_t current_steps = _raw_to_steps((uint32_t)labs((long)continuous_raw));
		if (labs((long)(current_steps - last_monitor_steps)) > position_tolerance_steps)
		{
			last_monitor_steps = current_steps;
			t_stable = now;
		}

		bool btn_now = button_pressed();
		bool btn_fall = (last_btn_state == true && btn_now == false);
		last_btn_state = btn_now;
		bool btn_raw = (GPIO_ReadInputDataBit(BTN_PORT, BTN_PIN) == Bit_SET);

		if (app_state == ST_IDLE)
		{
			led_set(false);
			// 上电后第一次按键：把当前位置设为软件零点
			if (btn_fall && encoder_ok)
			{
				as5600_reset_reference();
				start_raw = encoder_raw;
				start_continuous_raw = continuous_raw;
				end_continuous_raw = continuous_raw;
				measured_steps = 0;
				motor2_target_steps = 0;
				motor2_repeat_count = 0;
				motor1_started = 0;
				motor2_started = 0;
				last_monitor_steps = 0;
				t_start = now;
				t_stable = now;
				usart_send_string("[STATE] ZERO_SET\r\n");
				led_set(true);
				app_state = ST_WAIT_ZERO;
			}
		}
		else if (app_state == ST_WAIT_ZERO)
		{
			led_blink(now, 500);
			// 零点设置完成后，再次按键开始后续流程
			if (btn_fall)
			{
				start_raw = encoder_raw;
				start_continuous_raw = continuous_raw;
				end_continuous_raw = continuous_raw;
				last_monitor_steps = 0;
				t_start = now;
				t_stable = now;
				usart_send_string("[STATE] START_MONITOR\r\n");
				led_set(false);
				app_state = ST_MONITOR_MOVE;
			}
		}
		else if (app_state == ST_MONITOR_MOVE)
		{
			led_blink(now, 100);
			// 角度在 5 秒内不变，或监测超时 30 秒，就认为“推动结束”
			if ((now - t_start) > pdMS_TO_TICKS(encoder_timeout_ms))
			{
				_stop_all();
				led_set(false);
				usart_send_string("[STATE] ERROR_TIMEOUT\r\n");
				app_state = ST_ERROR;
			}
			else if ((now - t_stable) >= pdMS_TO_TICKS(encoder_stable_ms))
			{
				end_raw = encoder_raw;
				end_continuous_raw = continuous_raw;
				measured_steps = _raw_to_steps((uint32_t)labs((long)(end_continuous_raw - start_continuous_raw)));
				if (measured_steps == 0U) measured_steps = 1U;
				motor2_target_steps = measured_steps;
				t_start = now;
				led_set(false);
				usart_send_string("[STATE] END_CAPTURE\r\n");
				app_state = ST_RETURN_HOME;
			}
		}
		else if (app_state == ST_RETURN_HOME)
		{
			led_set(true);
			// 先把当前位置退回到起点，再开始 5 次往返
			if ((now - t_start) > pdMS_TO_TICKS(move_timeout_ms))
			{
				_stop_all();
				led_set(false);
				usart_send_string("[STATE] ERROR_TIMEOUT\r\n");
				app_state = ST_ERROR;
			}
			else
			{
				if (!motor2_started)
				{
					motor2_run(false, motor2_rpm);
					motor2_started = 1;
				}
				if (_within_step_tolerance(continuous_raw, start_continuous_raw, position_tolerance_steps))
				{
					motor2_stop();
					motor2_started = 0;
					motor2_repeat_count = 0;
					t_start = now;
					usart_send_string("[STATE] HOME_DONE\r\n");
					app_state = ST_M2_REPEAT_OUT;
				}
			}
		}
		else if (app_state == ST_M2_REPEAT_OUT)
		{
			// 电机2正向走到测得的步数
			led_set(true);
			if ((now - t_start) > pdMS_TO_TICKS(move_timeout_ms))
			{
				_stop_all();
				led_set(false);
				usart_send_string("[STATE] ERROR_TIMEOUT\r\n");
				app_state = ST_ERROR;
			}
			else
			{
				if (!motor2_started)
				{
					motor2_run(true, motor2_rpm);
					motor2_started = 1;
				}
				if (a4988_get_pulse_count() >= motor2_target_steps)
				{
					motor2_stop();
					motor2_started = 0;
					t_start = now;
					usart_send_string("[STATE] M2_OUT_DONE\r\n");
					app_state = ST_M2_REPEAT_BACK;
				}
			}
		}
		else if (app_state == ST_M2_REPEAT_BACK)
		{
			// 电机2反向回到起点，算完成 1 次往返
			led_set(true);
			if ((now - t_start) > pdMS_TO_TICKS(move_timeout_ms))
			{
				_stop_all();
				led_set(false);
				usart_send_string("[STATE] ERROR_TIMEOUT\r\n");
				app_state = ST_ERROR;
			}
			else
			{
				if (!motor2_started)
				{
					motor2_run(false, motor2_rpm);
					motor2_started = 1;
				}
				if (_within_step_tolerance(continuous_raw, start_continuous_raw, position_tolerance_steps))
				{
					motor2_stop();
					motor2_started = 0;
					motor2_repeat_count++;
					if (motor2_repeat_count < repeat_times)
					{
						usart_send_string("[STATE] M2_LOOP_NEXT\r\n");
						app_state = ST_M2_REPEAT_OUT;
					}
					else
					{
						usart_send_string("[STATE] M2_REPEAT_DONE\r\n");
						app_state = ST_M1_FINAL_OUT;
					}
					t_start = now;
				}
			}
		}
		else if (app_state == ST_M1_FINAL_OUT)
		{
			// 最后让电机1再顺时针走 3 圈
			led_set(true);
			if ((now - t_start) > pdMS_TO_TICKS(move_timeout_ms))
			{
				_stop_all();
				led_set(false);
				usart_send_string("[STATE] ERROR_TIMEOUT\r\n");
				app_state = ST_ERROR;
			}
			else
			{
				if (!motor1_started)
				{
					motor1_run(true, motor1_rpm);
					motor1_started = 1;
				}
				if (_raw_to_steps((uint32_t)labs((long)(continuous_raw - start_continuous_raw))) >= motor1_turn_steps)
				{
					motor1_stop();
					motor1_started = 0;
					t_start = now;
					usart_send_string("[STATE] M1_FINAL_OUT_DONE\r\n");
					app_state = ST_M1_FINAL_BACK;
				}
			}
		}
		else if (app_state == ST_M1_FINAL_BACK)
		{
			// 电机1回到起点后，再开始电机2最终复位
			led_set(true);
			if ((now - t_start) > pdMS_TO_TICKS(move_timeout_ms))
			{
				_stop_all();
				led_set(false);
				usart_send_string("[STATE] ERROR_TIMEOUT\r\n");
				app_state = ST_ERROR;
			}
			else
			{
				if (!motor1_started)
				{
					motor1_run(false, motor1_rpm);
					motor1_started = 1;
				}
				if (_within_step_tolerance(continuous_raw, start_continuous_raw, position_tolerance_steps))
				{
					motor1_stop();
					motor1_started = 0;
					t_start = now;
					usart_send_string("[STATE] M1_FINAL_BACK_DONE\r\n");
					app_state = ST_M2_RESET_OUT;
				}
			}
		}
		else if (app_state == ST_M2_RESET_OUT)
		{
			// 电机2最后再正向走回目标步数
			led_set(true);
			if ((now - t_start) > pdMS_TO_TICKS(move_timeout_ms))
			{
				_stop_all();
				led_set(false);
				usart_send_string("[STATE] ERROR_TIMEOUT\r\n");
				app_state = ST_ERROR;
			}
			else
			{
				if (!motor2_started)
				{
					motor2_run(true, motor2_rpm);
					motor2_started = 1;
				}
				if (a4988_get_pulse_count() >= motor2_target_steps)
				{
					motor2_stop();
					motor2_started = 0;
					t_start = now;
					usart_send_string("[STATE] M2_RESET_OUT_DONE\r\n");
					app_state = ST_M2_RESET_BACK;
				}
			}
		}
		else if (app_state == ST_M2_RESET_BACK)
		{
			// 电机2反向回到初始位置，整个流程结束
			led_set(true);
			if ((now - t_start) > pdMS_TO_TICKS(move_timeout_ms))
			{
				_stop_all();
				led_set(false);
				usart_send_string("[STATE] ERROR_TIMEOUT\r\n");
				app_state = ST_ERROR;
			}
			else
			{
				if (!motor2_started)
				{
					motor2_run(false, motor2_rpm);
					motor2_started = 1;
				}
				if (_within_step_tolerance(continuous_raw, start_continuous_raw, position_tolerance_steps))
				{
					motor2_stop();
					motor2_started = 0;
					usart_send_string("[STATE] M2_RESET_DONE\r\n");
					app_state = ST_FINISHED;
				}
			}
		}
		else if (app_state == ST_FINISHED)
		{
			// 所有动作完成，回到空闲状态
			_stop_all();
			led_set(false);
			usart_send_string("[STATE] FINISHED\r\n");
			app_state = ST_IDLE;
		}
		else
		{
			_stop_all();
			led_set(false);
		}

		// 串口调试输出：仅在状态/角度/按键变化时打印，避免刷屏
		{
			static AppState_t last_state = ST_ERROR;
			static bool last_btn_raw = true;
			static bool last_btn_now = true;
			static bool last_btn_fall = false;
			static bool last_encoder_ok = false;
			static uint16_t last_encoder_raw = 0xFFFF;
			static int32_t last_continuous_raw = 0x7FFFFFFF;
			static uint16_t last_start_raw = 0xFFFF;
			static uint16_t last_end_raw = 0xFFFF;
			static uint32_t last_measured_steps = 0xFFFFFFFFUL;
			static uint32_t last_motor2_target_steps = 0xFFFFFFFFUL;
			static uint32_t last_motor2_repeat_count = 0xFFFFFFFFUL;
			static uint8_t last_m1 = 0xFF;
			static uint8_t last_m2 = 0xFF;
			bool need_print = false;

			if (app_state != last_state) need_print = true;
			if (btn_raw != last_btn_raw) need_print = true;
			if (btn_now != last_btn_now) need_print = true;
			if (btn_fall != last_btn_fall) need_print = true;
			if (encoder_ok != last_encoder_ok) need_print = true;
			if (encoder_raw != last_encoder_raw) need_print = true;
			if (continuous_raw != last_continuous_raw) need_print = true;
			if (start_raw != last_start_raw) need_print = true;
			if (end_raw != last_end_raw) need_print = true;
			if (measured_steps != last_measured_steps) need_print = true;
			if (motor2_target_steps != last_motor2_target_steps) need_print = true;
			if (motor2_repeat_count != last_motor2_repeat_count) need_print = true;
			if (g_motor1.enabled != last_m1) need_print = true;
			if (g_motor2.enabled != last_m2) need_print = true;

			if (need_print)
			{
				last_state = app_state;
				last_btn_raw = btn_raw;
				last_btn_now = btn_now;
				last_btn_fall = btn_fall;
				last_encoder_ok = encoder_ok;
				last_encoder_raw = encoder_raw;
				last_continuous_raw = continuous_raw;
				last_start_raw = start_raw;
				last_end_raw = end_raw;
				last_measured_steps = measured_steps;
				last_motor2_target_steps = motor2_target_steps;
				last_motor2_repeat_count = motor2_repeat_count;
				last_m1 = g_motor1.enabled;
				last_m2 = g_motor2.enabled;

				usart_send_string("STATE=");
				usart_send_string((char *)state_to_str(app_state));
				usart_send_string(" BTN_RAW=");
				usart_send_u32_dec(btn_raw ? 1 : 0);
				usart_send_string(" BTN_PRESS=");
				usart_send_u32_dec(btn_now ? 1 : 0);
				usart_send_string(" BTN_FALL=");
				usart_send_u32_dec(btn_fall ? 1 : 0);
				usart_send_string(" ENC_OK=");
				usart_send_u32_dec(encoder_ok ? 1 : 0);
				usart_send_string(" RAW=");
				usart_send_u32_dec(encoder_raw);
				usart_send_string(" CONT=");
				usart_send_i32_dec(continuous_raw);
				usart_send_string(" START=");
				usart_send_u32_dec(start_raw);
				usart_send_string(" END=");
				usart_send_u32_dec(end_raw);
				usart_send_string(" STEP=");
				usart_send_u32_dec(measured_steps);
				usart_send_string(" M2_STEP=");
				usart_send_u32_dec(motor2_target_steps);
				usart_send_string(" REP=");
				usart_send_u32_dec(motor2_repeat_count);
				usart_send_string(" M1=");
				usart_send_u32_dec(g_motor1.enabled);
				usart_send_string(" M2=");
				usart_send_u32_dec(g_motor2.enabled);
				usart_send_string("\r\n");
			}
		}

		vTaskDelay(pdMS_TO_TICKS(10));
	}
}

int main(void)
{
	xMotorEventGroup = xEventGroupCreate();
	xTaskCreate(PhotoTask, "photo", 256, NULL, 2, NULL);
	vTaskStartScheduler();
	while (1)
	{
	}
}
