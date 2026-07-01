---
name: mcu-peripheral-driver
description: MCU peripheral driver development patterns for embedded C. Covers I2C, SPI, UART, PWM, ADC, Timer, GPIO. Use when writing bare-metal or HAL-based peripheral drivers for ARM Cortex-M (STM32, PY32, GD32, etc.) or other MCUs.
---

# MCU Peripheral Driver Development

## Principles

1. **Layered separation**: Application logic never touches registers directly — goes through HAL or a thin abstraction layer
2. **Configuration first, init second**: Fill a config struct, call init. Never hardcode parameters inside init
3. **Error return**: Every operation returns a status code (`int` or enum); avoid void functions for I/O
4. **Timeout on all blocking operations**: Busy loops must always have a timeout
5. **Static driver instance**: Keep driver state in a static struct, not global; expose via accessor functions

## Driver Structure Template

```
driver/
├── bsp_xxx.h        # Public API + config struct
├── bsp_xxx.c        # Implementation
└── bsp_xxx_priv.h   # Private register definitions (optional)
```

### Naming Convention

| Item | Convention | Example |
|------|-----------|---------|
| Module prefix | `bsp_<periph>` | `bsp_i2c`, `bsp_uart` |
| Init function | `bsp_xxx_init` | `bsp_i2c_init` |
| Config struct | `bsp_xxx_cfg_t` | `bsp_i2c_cfg_t` |
| Status type | `bsp_xxx_status_t` | `bsp_i2c_status_t` |
| Handle struct | `bsp_xxx_t` | `bsp_i2c_t` |

## Peripherals

### GPIO

```c
typedef struct {
    void    *port;          // GPIO_TypeDef*
    uint16_t pin;
    uint8_t  mode;          // 0=input, 1=output_pp, 2=output_od, 3=af_pp
    uint8_t  pull;          // 0=nopull, 1=pullup, 2=pulldown
    uint8_t  speed;         // 0=low, 1=medium, 2=high
} bsp_gpio_cfg_t;

int bsp_gpio_init(bsp_gpio_cfg_t *cfg);
void bsp_gpio_set(void *port, uint16_t pin, uint8_t level);
uint8_t bsp_gpio_get(void *port, uint16_t pin);
void bsp_gpio_toggle(void *port, uint16_t pin);
```

### UART (Interrupt-driven)

```c
typedef struct {
    void     *instance;       // USART_TypeDef*
    uint32_t baud;
    uint8_t  data_bits;       // 7 or 8
    uint8_t  parity;          // 0=none, 1=even, 2=odd
    uint8_t  stop_bits;       // 0=0.5, 1=1, 2=1.5, 3=2
    void     *rx_buf;         // ring buffer
    uint32_t rx_buf_size;
    void (*tx_cplt_cb)(void);
    void (*rx_cb)(uint8_t byte);
} bsp_uart_cfg_t;

int bsp_uart_init(bsp_uart_cfg_t *cfg);
int bsp_uart_send(bsp_uart_cfg_t *cfg, const uint8_t *data, uint32_t len, uint32_t timeout_ms);
int bsp_uart_recv_byte(bsp_uart_cfg_t *cfg, uint8_t *byte, uint32_t timeout_ms);
```

**Pattern:** Use a ring buffer for RX, double-buffer or DMA for TX.

### I2C (Master)

```c
typedef struct {
    void     *instance;
    uint32_t speed_hz;        // 100000 or 400000
    uint8_t  own_addr;
    uint32_t timeout_ms;
} bsp_i2c_cfg_t;

int bsp_i2c_init(bsp_i2c_cfg_t *cfg);
int bsp_i2c_write_reg(bsp_i2c_cfg_t *cfg, uint8_t dev_addr, uint8_t reg, const uint8_t *data, uint32_t len);
int bsp_i2c_read_reg(bsp_i2c_cfg_t *cfg, uint8_t dev_addr, uint8_t reg, uint8_t *data, uint32_t len);
```

**Pattern:** Write-then-restart-read for register access. Always handle NACK and bus busy.

### SPI

```c
typedef struct {
    void     *instance;
    uint32_t baud;
    uint8_t  mode;            // 0-3 (CPOL/CPHA)
    uint8_t  data_width;      // 8 or 16
    uint8_t  first_bit;       // 0=MSB, 1=LSB
} bsp_spi_cfg_t;

int bsp_spi_init(bsp_spi_cfg_t *cfg);
int bsp_spi_transfer(bsp_spi_cfg_t *cfg, const uint8_t *tx, uint8_t *rx, uint32_t len);
```

**Pattern:** Full-duplex `transfer` always sends and receives simultaneously. Use DMA for blocks > 64 bytes.

### PWM

```c
typedef struct {
    void     *timer;          // TIM_TypeDef*
    uint8_t  channel;         // 1-4
    uint32_t freq_hz;
    uint16_t duty_permille;   // 0-1000
} bsp_pwm_cfg_t;

int bsp_pwm_init(bsp_pwm_cfg_t *cfg);
int bsp_pwm_set_duty(bsp_pwm_cfg_t *cfg, uint16_t duty_permille);
```

**Pattern:** Use permille (0-1000) not percent, to avoid floating point. Preload ARR+CCR for glitch-free update.

### ADC (Single conversion)

```c
typedef struct {
    void     *instance;
    uint8_t  channel;
    uint32_t sampling_time;   // cycles
} bsp_adc_cfg_t;

int bsp_adc_init(bsp_adc_cfg_t *cfg);
int bsp_adc_read(bsp_adc_cfg_t *cfg, uint16_t *value);
```

**Pattern:** Insert small delay after channel switch for settling. Average N samples for noise reduction.

### Timer (Basic delay / periodic)

```c
typedef struct {
    void     *timer;
    uint32_t period_us;
    void (*callback)(void);
    uint8_t  oneshot;         // 0=periodic, 1=oneshot
} bsp_timer_cfg_t;

int bsp_timer_init(bsp_timer_cfg_t *cfg);
int bsp_timer_start(bsp_timer_cfg_t *cfg);
int bsp_timer_stop(bsp_timer_cfg_t *cfg);
```

## Common Anti-patterns

| Anti-pattern | Fix |
|-------------|-----|
| `HAL_Delay()` in interrupt context | Use timestamps + polling, or RTOS semaphore |
| Busy-loop without timeout | Always add `timeout_ms` parameter |
| Magical register values in `#define` | Add comment referencing RM page |
| Mixing HAL + direct register access | Pick one layer per module |
| SPI CS pin toggled outside driver | Abstract CS into `bsp_spi_cfg_t` |
| ISR doing heavy processing | ISR should only set flag/semaphore, do work in task |

## Verification Checklist

- [ ] All blocking operations have a timeout
- [ ] Busy-wait loops can't lock up on hardware fault
- [ ] ISRs are minimal (flag + clear + exit)
- [ ] Init is idempotent or has protection
- [ ] Config struct has defaults for optional fields
- [ ] Error codes are enumerated, not magic numbers
- [ ] De-init restores GPIO to default state
- [ ] No `HAL_Delay()` in ISR or between start/stop
