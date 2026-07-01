#ifndef __AS5600_H
#define __AS5600_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32f10x.h"

#ifdef __cplusplus
extern "C" {
#endif

#define AS5600_DEFAULT_I2C        AS5600_I2C1
#define AS5600_DEFAULT_ADDRESS    0x36
#define AS5600_DEFAULT_SAMPLE_MS  10
#define AS5600_DEFAULT_DEGREE_OFFSET 0

typedef enum {
    AS5600_I2C1 = 1,
    AS5600_I2C2 = 2
} AS5600_I2CId_t;

typedef enum {
    AS5600_OK = 0, // 成功
    AS5600_ERR_INVALID_PARAM = -1, // 无效参数
    AS5600_ERR_NOT_INIT = -2, // 未初始化
    AS5600_ERR_I2C = -3, // I2C 错误
    AS5600_ERR_ALREADY_INIT = -4 // 已经初始化
} AS5600_Error_t;

typedef struct {
    AS5600_I2CId_t i2c_id; // I2C 编号
    GPIO_TypeDef *scl_port;
    GPIO_TypeDef *sda_port; // SDA 端口
    uint16_t scl_pin; // SCL 引脚
    uint16_t sda_pin; // SDA 引脚
    GPIO_TypeDef *dir_port; // DIR 端口（可选）
    uint16_t dir_pin; // DIR 引脚（可选）
    bool dir_enable; // 是否启用 DIR 引脚
    uint8_t dir_level; // DIR 输出电平（0/1）
    uint16_t sample_ms; // 采样周期
    int16_t degree_offset;
    uint8_t i2c_address;
} AS5600_Config_t;

AS5600_Error_t as5600_init(const AS5600_Config_t *config);//初始化AS5600
bool as5600_update(void);//更新AS5600
bool as5600_read_raw(uint16_t *raw_value);//读取原始角度
int32_t as5600_read_angle(void);//读取角度
int32_t as5600_read_angle_raw(void);//读取原始角度
void as5600_reset_reference(void);//重置参考角度
bool as5600_is_ready(void);//是否准备好

#ifdef __cplusplus
}
#endif

#endif /* __AS5600_H */
