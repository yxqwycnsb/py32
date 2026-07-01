---
name: mcu-peripheral-driver
description: MCU外设驱动开发规范与模式。涵盖I2C、SPI、UART、PWM、ADC、定时器、GPIO。适用于基于HAL或寄存器操作的ARM Cortex-M（STM32、PY32、GD32等）及各类MCU的驱动开发。
---

# MCU 外设驱动开发规范

## 基本原则

1. **分层隔离**：应用代码不直接操作寄存器，通过HAL或薄抽象层访问
2. **先配置后初始化**：填充配置结构体→调用init函数，不允许在init内部硬编码参数
3. **错误返回**：每个I/O操作返回状态码（`int`或枚举），避免void函数
4. **所有阻塞操作加超时**：忙等待循环必须带超时机制
5. **驱动实例静态化**：驱动状态放在static结构体中，通过访问函数暴露

## 驱动结构规范

```
driver/
├── bsp_xxx.h        # 公共API + 配置结构体
├── bsp_xxx.c        # 实现
└── bsp_xxx_priv.h   # 私有寄存器定义（可选）
```

### 命名约定

| 项目 | 约定 | 示例 |
|------|------|------|
| 模块前缀 | `bsp_<外设名>` | `bsp_i2c` |
| 初始化函数 | `bsp_xxx_init` | `bsp_i2c_init` |
| 配置结构体 | `bsp_xxx_cfg_t` | `bsp_i2c_cfg_t` |
| 状态类型 | `bsp_xxx_status_t` | `bsp_i2c_status_t` |
| 句柄结构体 | `bsp_xxx_t` | `bsp_i2c_t` |

## 各外设驱动模式

### GPIO

```c
typedef struct {
    void    *port;          // GPIO_TypeDef*
    uint16_t pin;
    uint8_t  mode;          // 0=输入, 1=推挽输出, 2=开漏输出, 3=复用推挽
    uint8_t  pull;          // 0=无, 1=上拉, 2=下拉
    uint8_t  speed;         // 0=低, 1=中, 2=高
} bsp_gpio_cfg_t;

int bsp_gpio_init(bsp_gpio_cfg_t *cfg);
void bsp_gpio_set(void *port, uint16_t pin, uint8_t level);
uint8_t bsp_gpio_get(void *port, uint16_t pin);
void bsp_gpio_toggle(void *port, uint16_t pin);
```

### UART（中断驱动）

```c
typedef struct {
    void     *instance;       // USART_TypeDef*
    uint32_t baud;
    uint8_t  data_bits;
    uint8_t  parity;
    uint8_t  stop_bits;
    void     *rx_buf;         // 环形缓冲区
    uint32_t rx_buf_size;
    void (*tx_cplt_cb)(void);
    void (*rx_cb)(uint8_t byte);
} bsp_uart_cfg_t;

int bsp_uart_init(bsp_uart_cfg_t *cfg);
int bsp_uart_send(bsp_uart_cfg_t *cfg, const uint8_t *data, uint32_t len, uint32_t timeout_ms);
int bsp_uart_recv_byte(bsp_uart_cfg_t *cfg, uint8_t *byte, uint32_t timeout_ms);
```

**推荐模式**：RX使用环形缓冲区，TX使用双缓冲或DMA。

### I2C（主机模式）

```c
typedef struct {
    void     *instance;
    uint32_t speed_hz;
    uint8_t  own_addr;
    uint32_t timeout_ms;
} bsp_i2c_cfg_t;

int bsp_i2c_init(bsp_i2c_cfg_t *cfg);
int bsp_i2c_write_reg(bsp_i2c_cfg_t *cfg, uint8_t dev_addr, uint8_t reg, const uint8_t *data, uint32_t len);
int bsp_i2c_read_reg(bsp_i2c_cfg_t *cfg, uint8_t dev_addr, uint8_t reg, uint8_t *data, uint32_t len);
```

**推荐模式**：写-重启-读实现寄存器访问。必须处理NACK和总线忙。

### SPI

```c
typedef struct {
    void     *instance;
    uint32_t baud;
    uint8_t  mode;            // 0-3 (CPOL/CPHA)
    uint8_t  data_width;
    uint8_t  first_bit;       // 0=MSB, 1=LSB
} bsp_spi_cfg_t;

int bsp_spi_init(bsp_spi_cfg_t *cfg);
int bsp_spi_transfer(bsp_spi_cfg_t *cfg, const uint8_t *tx, uint8_t *rx, uint32_t len);
```

**推荐模式**：全双工`transfer`同时发送和接收；超过64字节用DMA。

### PWM

```c
typedef struct {
    void     *timer;          // TIM_TypeDef*
    uint8_t  channel;
    uint32_t freq_hz;
    uint16_t duty_permille;   // 0-1000
} bsp_pwm_cfg_t;

int bsp_pwm_init(bsp_pwm_cfg_t *cfg);
int bsp_pwm_set_duty(bsp_pwm_cfg_t *cfg, uint16_t duty_permille);
```

**推荐模式**：使用千分比(0-1000)而非百分比，避免浮点运算。启用预装载防止更新抖动。

### ADC（单次转换）

```c
typedef struct {
    void     *instance;
    uint8_t  channel;
    uint32_t sampling_time;
} bsp_adc_cfg_t;

int bsp_adc_init(bsp_adc_cfg_t *cfg);
int bsp_adc_read(bsp_adc_cfg_t *cfg, uint16_t *value);
```

**推荐模式**：切换通道后加短延时等待稳定；多次采样取平均抗噪声。

### 定时器

```c
typedef struct {
    void     *timer;
    uint32_t period_us;
    void (*callback)(void);
    uint8_t  oneshot;         // 0=周期, 1=单次
} bsp_timer_cfg_t;

int bsp_timer_init(bsp_timer_cfg_t *cfg);
int bsp_timer_start(bsp_timer_cfg_t *cfg);
int bsp_timer_stop(bsp_timer_cfg_t *cfg);
```

## 常见反模式

| 反模式 | 修正方案 |
|--------|---------|
| 中断里调用 `HAL_Delay()` | 改用时间戳轮询或RTOS信号量 |
| 忙等无超时 | 始终加 `timeout_ms` 参数 |
| 寄存器的魔法数值无注释 | 加注释标注参考手册页码 |
| 混合HAL和直接寄存器操作 | 每个模块选一种方式 |
| SPI片选在驱动外控制 | 将CS抽象进 `bsp_spi_cfg_t` |
| ISR做大量处理 | ISR只置标志/发信号量，任务里处理 |

## 验收检查清单

- [ ] 所有阻塞操作带超时
- [ ] 忙等待循环在硬件异常时不会死锁
- [ ] 中断服务函数最小化（置标志+清中断+退出）
- [ ] 初始化函数可重入或有保护
- [ ] 配置结构体的可选字段有默认值
- [ ] 错误码用枚举而非魔法数字
- [ ] 反初始化恢复GPIO默认状态
- [ ] ISR中无阻塞调用
