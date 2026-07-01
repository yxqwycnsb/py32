#include "as5600.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_rcc.h"
#include <stddef.h>

#define AS5600_REG_RAW_ANGLE_H   0x0C
#define AS5600_REG_ANGLE_H       0x0E
#define AS5600_RAW_MAX           4096U

static AS5600_Config_t s_cfg;
static bool s_initialized = false;
static bool s_ready = false;
static uint16_t s_raw_angle = 0;
static int32_t s_reference_offset = 0;
static I2C_TypeDef *s_i2c = I2C1;

static I2C_TypeDef *_resolve_i2c(AS5600_I2CId_t id)
{
    return (id == AS5600_I2C2) ? I2C2 : I2C1;
}

static void _enable_i2c_clock(AS5600_I2CId_t id)
{
    if (id == AS5600_I2C2) {
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
    } else {
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
    }
}

static void _enable_gpio_clock(GPIO_TypeDef *port)
{
    if (port == GPIOA) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    else if (port == GPIOB) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    else if (port == GPIOC) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    else if (port == GPIOD) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
    else if (port == GPIOE) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
}

static void _gpio_init(void)
{
    if (s_cfg.scl_port) _enable_gpio_clock(s_cfg.scl_port);
    if (s_cfg.sda_port && s_cfg.sda_port != s_cfg.scl_port) _enable_gpio_clock(s_cfg.sda_port);

    GPIO_InitTypeDef gpio;
    gpio.GPIO_Pin = s_cfg.scl_pin;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_Init(s_cfg.scl_port, &gpio);

    gpio.GPIO_Pin = s_cfg.sda_pin;
    GPIO_Init(s_cfg.sda_port, &gpio);

    if (s_cfg.dir_enable && s_cfg.dir_port != 0) {
        GPIO_InitTypeDef dir_gpio;
        dir_gpio.GPIO_Pin = s_cfg.dir_pin;
        dir_gpio.GPIO_Speed = GPIO_Speed_50MHz;
        dir_gpio.GPIO_Mode = GPIO_Mode_Out_PP;
        GPIO_Init(s_cfg.dir_port, &dir_gpio);
        GPIO_WriteBit(s_cfg.dir_port, s_cfg.dir_pin, s_cfg.dir_level ? Bit_SET : Bit_RESET);
    }
}

static void _i2c_init(void)
{
    I2C_InitTypeDef i2c;
    I2C_DeInit(s_i2c);
    i2c.I2C_ClockSpeed = 100000;
    i2c.I2C_Mode = I2C_Mode_I2C;
    i2c.I2C_DutyCycle = I2C_DutyCycle_2;
    i2c.I2C_OwnAddress1 = 0x00;
    i2c.I2C_Ack = I2C_Ack_Enable;
    i2c.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_Init(s_i2c, &i2c);
    I2C_Cmd(s_i2c, ENABLE);
}

static bool _wait_event(uint32_t event, uint32_t timeout)
{
    while (!I2C_CheckEvent(s_i2c, event)) {
        if (timeout-- == 0) return false;
    }
    return true;
}

static bool _wait_flag(uint16_t flag, FlagStatus status, uint32_t timeout)
{
    while (I2C_GetFlagStatus(s_i2c, flag) == status) {
        if (timeout-- == 0) return false;
    }
    return true;
}

static bool _i2c_read_bytes(uint8_t reg, uint8_t *buf, uint8_t len)
{
    if (buf == 0 || len == 0) return false;

    while (I2C_GetFlagStatus(s_i2c, I2C_FLAG_BUSY) == SET) {
    }

    I2C_GenerateSTART(s_i2c, ENABLE);
    if (!_wait_event(I2C_EVENT_MASTER_MODE_SELECT, 0xFFFF)) {
        I2C_GenerateSTOP(s_i2c, ENABLE);
        return false;
    }

    I2C_Send7bitAddress(s_i2c, (uint8_t)(s_cfg.i2c_address << 1), I2C_Direction_Transmitter);
    if (!_wait_event(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED, 0xFFFF)) {
        I2C_GenerateSTOP(s_i2c, ENABLE);
        return false;
    }

    I2C_SendData(s_i2c, reg);
    if (!_wait_event(I2C_EVENT_MASTER_BYTE_TRANSMITTED, 0xFFFF)) {
        I2C_GenerateSTOP(s_i2c, ENABLE);
        return false;
    }

    I2C_GenerateSTART(s_i2c, ENABLE);
    if (!_wait_event(I2C_EVENT_MASTER_MODE_SELECT, 0xFFFF)) {
        I2C_GenerateSTOP(s_i2c, ENABLE);
        return false;
    }

    I2C_Send7bitAddress(s_i2c, (uint8_t)(s_cfg.i2c_address << 1), I2C_Direction_Receiver);
    if (!_wait_event(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED, 0xFFFF)) {
        I2C_GenerateSTOP(s_i2c, ENABLE);
        return false;
    }

    if (len == 1) {
        I2C_AcknowledgeConfig(s_i2c, DISABLE);
        I2C_GenerateSTOP(s_i2c, ENABLE);
        if (!_wait_event(I2C_EVENT_MASTER_BYTE_RECEIVED, 0xFFFF)) {
            I2C_AcknowledgeConfig(s_i2c, ENABLE);
            return false;
        }
        buf[0] = I2C_ReceiveData(s_i2c);
        I2C_AcknowledgeConfig(s_i2c, ENABLE);
        return true;
    }

    while (len > 0) {
        if (len == 1) {
            I2C_AcknowledgeConfig(s_i2c, DISABLE);
            I2C_GenerateSTOP(s_i2c, ENABLE);
        }
        if (!_wait_event(I2C_EVENT_MASTER_BYTE_RECEIVED, 0xFFFF)) {
            I2C_AcknowledgeConfig(s_i2c, ENABLE);
            return false;
        }
        *buf++ = I2C_ReceiveData(s_i2c);
        len--;
    }
    I2C_AcknowledgeConfig(s_i2c, ENABLE);
    return true;
}

AS5600_Error_t as5600_init(const AS5600_Config_t *config)
{
    if (config == 0 || config->scl_port == 0 || config->sda_port == 0) return AS5600_ERR_INVALID_PARAM;
    if (s_initialized) return AS5600_ERR_ALREADY_INIT;

    s_cfg = *config;
    if (s_cfg.sample_ms == 0) s_cfg.sample_ms = AS5600_DEFAULT_SAMPLE_MS;
    if (s_cfg.i2c_address == 0) s_cfg.i2c_address = AS5600_DEFAULT_ADDRESS;
    if (s_cfg.i2c_id != AS5600_I2C2) s_cfg.i2c_id = AS5600_I2C1;

    s_i2c = _resolve_i2c(s_cfg.i2c_id);
    _enable_i2c_clock(s_cfg.i2c_id);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    _gpio_init();
    _i2c_init();

    s_initialized = true;
    s_ready = false;
    s_raw_angle = 0;
    s_reference_offset = 0;
    return AS5600_OK;
}

bool as5600_update(void)
{
    if (!s_initialized) return false;

    uint8_t data[2] = {0};
    if (!_i2c_read_bytes(AS5600_REG_RAW_ANGLE_H, data, 2)) return false;

    s_raw_angle = (uint16_t)((((uint16_t)data[0] & 0x0F) << 8) | data[1]);
    s_ready = true;
    return true;
}

bool as5600_read_raw(uint16_t *raw_value)
{
    if (!s_initialized || !s_ready || raw_value == 0) return false;
    *raw_value = s_raw_angle;
    return true;
}

int32_t as5600_read_angle_raw(void)
{
    if (!s_initialized || !s_ready) return 0;
    return (int32_t)s_raw_angle;
}

int32_t as5600_read_angle(void)
{
    if (!s_initialized || !s_ready) return 0;
    int32_t angle = ((int32_t)s_raw_angle * 360) / (int32_t)AS5600_RAW_MAX;
    angle -= s_cfg.degree_offset;
    angle -= s_reference_offset;
    while (angle < 0) angle += 360;
    while (angle >= 360) angle -= 360;
    return angle;
}

void as5600_reset_reference(void)
{
    if (!s_initialized || !s_ready) return;
    s_reference_offset = ((int32_t)s_raw_angle * 360) / (int32_t)AS5600_RAW_MAX;
}

bool as5600_is_ready(void)
{
    return s_initialized && s_ready;
}
