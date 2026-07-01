/**
 * photoelectric.c — 光电传感器驱动（已弃用）
 *
 * 当前项目已改用 AS5600 磁编码器进行位置检测。
 * 此文件作为编译桩保留，以防止 Keil 工程因缺少此文件而链接失败。
 * 如需彻底移除，请在 Keil IDE 中从工程里删除 photoelectric.c 的引用。
 */

#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "task.h"

/* 占位实现 — 未被当前 main.c 调用 */

void photoelectric_init(void)
{
	/* 已弃用 */
}

uint8_t photoelectric_update(void)
{
	/* 已弃用 */
	return 0;
}

uint8_t photoelectric_read_raw(void)
{
	/* 已弃用 */
	return 0;
}
