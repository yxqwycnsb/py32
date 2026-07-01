#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "event_groups.h"

#include "a4988.h"
#include "rtos_sync.h"
#include "usart.h"
#include "x_v2_motor2.h"
#include <stdlib.h>

// 电机同步事件组，后续如果要扩展任务同步可以继续使用
EventGroupHandle_t xMotorEventGroup = NULL;

// 运行时状态，主要用于串口调试输出
typedef struct
{
	uint16_t speed_rpm;
	uint8_t enabled;
} MotorRuntime_t;

static MotorRuntime_t g_motor1 = { 20, 0 };
static MotorRuntime_t g_motor2 = { 20, 0 };

// 主状态机：控制整套加工流程
typedef enum
{
	ST_IDLE = 0,
	ST_WAIT_M2_STABLE,
	ST_M1_CW,
	ST_M1_CCW,
	ST_M2_RETURN_ZERO,
	ST_REPEAT_CHECK,
	ST_FINISHED,
	ST_ERROR,
} AppState_t;

// 状态转字符串，方便串口打印
static const char *state_to_str(AppState_t state)
{
	switch (state)
	{
	case ST_IDLE: return "IDLE";
	case ST_WAIT_M2_STABLE: return "WAIT_M2";
	case ST_M1_CW: return "M1_CW";
	case ST_M1_CCW: return "M1_CCW";
	case ST_M2_RETURN_ZERO: return "M2_HOME";
	case ST_REPEAT_CHECK: return "REPEAT";
	case ST_FINISHED: return "FINISHED";
	case ST_ERROR: return "ERROR";
	default: return "UNKNOWN";
	}
}

// 初始化电机1和电机2
static void motor_pair_init(void)
{
	a4988_init();
	a4988_set_microstep(MICROSTEP_1_16);
	a4988_enable(true);
	x_v2_motor2_init();
	g_motor1.enabled = 1;
	g_motor2.enabled = 0;
}

// 启动电机1
static void motor1_run(uint8_t forward, uint16_t rpm)
{
	g_motor1.speed_rpm = rpm;
	g_motor1.enabled = 1;
	a4988_set_direction(forward ? true : false);
	a4988_run_rpm(rpm);
}

// 停止电机1
static void motor1_stop(void)
{
	a4988_stop_motor();
	g_motor1.enabled = 0;
}

#define BTN_PORT GPIOB
#define BTN_PIN  GPIO_Pin_1

// 按键初始化，PB1 上拉输入，低电平表示按下
static void button_init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitTypeDef gpio;
	gpio.GPIO_Pin = BTN_PIN;
	gpio.GPIO_Mode = GPIO_Mode_IPU;
	gpio.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(BTN_PORT, &gpio);
}

// 读取按键状态
static bool button_pressed(void)
{
	return GPIO_ReadInputDataBit(BTN_PORT, BTN_PIN) == Bit_RESET;
}

// 主控制任务
static void PhotoTask(void *arg)
{
	(void)arg;

	// 参数可根据实际调试进行调整
	const uint16_t motor1_rpm = 20;
	const uint32_t repeat_times = 5U;
	const uint32_t stable_window_ms = 800U;
	const uint32_t encoder_timeout_ms = 15000U;
	const int32_t encoder_deadband_tenths = 10;
	const uint32_t move_timeout_ms = 30000U;

	static AppState_t app_state = ST_IDLE;
	static uint32_t t_start = 0;
	static uint32_t stable_start = 0;
	static int32_t last_pos_tenths = 0;
	static int32_t home_pos_tenths = 0;
	static uint32_t cycle_count = 0;
	static uint8_t inited = 0;
	static bool last_btn_state = true;
	static uint8_t motor1_started = 0;
	static uint8_t m2_return_started = 0;

	if (!inited)
	{
		motor_pair_init();
		button_init();
		last_btn_state = button_pressed();
		inited = 1;
		usart_send_string("[BOOT] system init ok\r\n");
	}

	for (;;)
	{
		uint32_t now = xTaskGetTickCount();
		bool btn_now = button_pressed();
		bool btn_fall = (last_btn_state == true && btn_now == false);
		last_btn_state = btn_now;

		if (app_state == ST_IDLE)
		{
			motor1_stop();
			x_v2_motor2_enable(false);
			if (btn_fall)
			{
				cycle_count = 0;
				m2_return_started = 0;
				home_pos_tenths = x_v2_motor2_get_angle_tenths();
				last_pos_tenths = home_pos_tenths;
				stable_start = now;
				t_start = now;
				app_state = ST_WAIT_M2_STABLE;
				usart_send_string("[EVENT] key pressed, enter WAIT_M2\r\n");
			}
		}
		else if (app_state == ST_WAIT_M2_STABLE)
		{
			int32_t cur = x_v2_motor2_get_angle_tenths();
			if (labs((long)(cur - last_pos_tenths)) <= encoder_deadband_tenths)
			{
				if ((now - stable_start) >= pdMS_TO_TICKS(stable_window_ms))
				{
					home_pos_tenths = cur;
					x_v2_motor2_enable(true);
					x_v2_motor2_reset_position();
					motor1_started = 0;
					m2_return_started = 0;
					t_start = now;
					app_state = ST_M1_CW;
					usart_send_string("[STATE] M2 stable, start M1 CW\r\n");
				}
			}
			else
			{
				last_pos_tenths = cur;
				stable_start = now;
				usart_send_string("[DEBUG] M2 moved, restart stable timer\r\n");
			}
			if ((now - t_start) > pdMS_TO_TICKS(encoder_timeout_ms))
			{
				app_state = ST_ERROR;
				usart_send_string("[ERROR] wait M2 stable timeout\r\n");
			}
		}
		else if (app_state == ST_M1_CW)
		{
			if ((now - t_start) > pdMS_TO_TICKS(move_timeout_ms))
			{
				app_state = ST_ERROR;
				usart_send_string("[ERROR] M1 CW timeout\r\n");
			}
			else
			{
				if (!motor1_started)
				{
					motor1_run(true, motor1_rpm);
					motor1_started = 1;
					usart_send_string("[RUN] M1 clockwise start\r\n");
				}
				if (labs((long)a4988_get_pulse_count()) >= (long)a4988_get_steps_per_rev())
				{
					motor1_stop();
					motor1_started = 0;
					t_start = now;
					app_state = ST_M1_CCW;
					usart_send_string("[STATE] M1 CW done, enter CCW\r\n");
				}
			}
		}
		else if (app_state == ST_M1_CCW)
		{
			if ((now - t_start) > pdMS_TO_TICKS(move_timeout_ms))
			{
				app_state = ST_ERROR;
				usart_send_string("[ERROR] M1 CCW timeout\r\n");
			}
			else
			{
				if (!motor1_started)
				{
					motor1_run(false, motor1_rpm);
					motor1_started = 1;
					usart_send_string("[RUN] M1 counterclockwise start\r\n");
				}
				if (a4988_get_pulse_count() == 0)
				{
					motor1_stop();
					motor1_started = 0;
					t_start = now;
					app_state = ST_M2_RETURN_ZERO;
					usart_send_string("[STATE] M1 CCW done, enter M2 home\r\n");
				}
			}
		}
		else if (app_state == ST_M2_RETURN_ZERO)
		{
			if ((now - t_start) > pdMS_TO_TICKS(move_timeout_ms))
			{
				app_state = ST_ERROR;
				usart_send_string("[ERROR] M2 return timeout\r\n");
			}
			else
			{
				if (!m2_return_started)
				{
					x_v2_motor2_enable(true);
					x_v2_motor2_return_to_zero();
					m2_return_started = 1;
					usart_send_string("[RUN] M2 return to zero start\r\n");
				}
				if (labs((long)x_v2_motor2_get_angle_tenths()) <= encoder_deadband_tenths)
				{
					x_v2_motor2_stop();
					app_state = ST_REPEAT_CHECK;
					usart_send_string("[STATE] M2 home done\r\n");
				}
			}
		}
		else if (app_state == ST_REPEAT_CHECK)
		{
			cycle_count++;
			usart_send_string("[COUNT] cycle="); usart_send_u32_dec(cycle_count); usart_send_string("\r\n");
			if (cycle_count < repeat_times)
			{
				app_state = ST_WAIT_M2_STABLE;
				usart_send_string("[STATE] next cycle, wait M2 stable\r\n");
			}
			else
			{
				app_state = ST_FINISHED;
				usart_send_string("[STATE] all cycles finished\r\n");
			}
		}
		else if (app_state == ST_FINISHED)
		{
			motor1_stop();
			x_v2_motor2_enable(false);
			usart_send_string("[DONE] task finished, back to idle\r\n");
			app_state = ST_IDLE;
		}
		else
		{
			motor1_stop();
			x_v2_motor2_stop();
			usart_send_string("[ERROR] enter safe stop\r\n");
		}

		// 定期输出状态，方便串口实时看程序卡在哪一步
		static uint32_t last_print = 0;
		if ((now - last_print) >= pdMS_TO_TICKS(200))
		{
			last_print = now;
			usart_send_string("STATE="); usart_send_string((char *)state_to_str(app_state));
			usart_send_string(" M1="); usart_send_u32_dec(g_motor1.enabled);
			usart_send_string(" M2="); usart_send_u32_dec(g_motor2.enabled);
			usart_send_string(" M2POS="); usart_send_i32_dec(x_v2_motor2_get_angle_tenths());
			usart_send_string(" IBUS="); usart_send_i32_dec(x_v2_motor2_get_bus_current_ma());
			usart_send_string(" IPHA="); usart_send_i32_dec(x_v2_motor2_get_phase_current_ma());
			usart_send_string(" HOME="); usart_send_i32_dec(home_pos_tenths);
			usart_send_string(" CYCLE="); usart_send_u32_dec(cycle_count);
			usart_send_string("\r\n");
		}

		vTaskDelay(pdMS_TO_TICKS(10));
	}
}

int main(void)
{
	xMotorEventGroup = xEventGroupCreate();
	usart_init(115200);
	usart_send_string("[BOOT] main start\r\n");
	usart_send_string("[BOOT] creating task\r\n");
	xTaskCreate(PhotoTask, "photo", 256, NULL, 2, NULL);
	vTaskStartScheduler();
	while (1) {}
}
