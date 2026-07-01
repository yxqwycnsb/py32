---
name: low-power-optimization
description: Low-power design patterns for battery-powered MCU devices. Covers sleep modes, clock gating, peripheral shutdown, ADC sampling strategy, and measurement techniques.
---

# Low-Power Optimization

## Sleep Mode Hierarchy (ARM Cortex-M)

| Mode | Current | Wake Latency | Wake Source | When to Use |
|------|---------|-------------|-------------|-------------|
| Run | Full | N/A | N/A | Active processing |
| Sleep (WFI) | ~50-70% of run | ~5 µs | Any interrupt | Short idle 100 µs-10 ms |
| Stop/SleepDeep | ~5-20 µA | ~10-100 µs | RTC, EXTI, LPUART | Waiting for periodic wake or external event |
| Standby | ~0.1-2 µA | ~1-10 ms | Reset, RTC, Wakeup pin | Long idle, periodic sensor reporting |
| Shutdown/Off | ~0 nA | ~10-100 ms | Reset, Power-on | User off, shipping mode |

## Key Strategies

### 1. Clock Gating First

Before optimizing sleep modes, turn off unused peripheral clocks:

```c
// Turn off peripherals not in use
RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_USART2, DISABLE);
RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_SPI1, DISABLE);

// After entering stop mode, gate the HSI/PLL too
RCC->APB1ENR = 0;  // or use HAL_RCCEx_PeriphCLKConfig
```

**Check `RCC->xxENR` after init — many HAL drivers leave clocks on.**

### 2. Stop Mode + RTC Wake

```c
// Enter STOP mode, wake by RTC every N seconds
void enter_stop_with_rtc(uint32_t seconds) {
    // RTCCLK/16 = 2048 Hz when LSE=32768 Hz
    // counter = seconds * 2048, max 16-bit (65535 ≈ 32s)
    uint32_t counter = seconds * 2048U;
    if (counter > 0xFFFF) counter = 0xFFFF;
    HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, counter, RTC_WAKEUPCLOCK_RTCCLK_DIV16);

    // Enter Stop mode
    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);

    // After wake: re-init system clocks
    SystemClock_Config();
}
```

### 3. STOP2 on STM32L-series

```c
// Deepest stop with SRAM retention
HAL_PWREx_EnterSTOP2Mode(PWR_STOPENTRY_WFI);
```

### 4. Peripheral Shutdown Sequence

```
Application idle
  → Stop ADC conversions (if continuous)
    → Disable peripheral clocks
      → Set GPIO to analog mode (lowest leakage)
        → Enter STOP/STANDBY mode
          → Wakeup event → re-init peripherals → resume
```

### 5. GPIO Leakage Optimization

```c
// Before sleep: set all unused pins to analog mode (lowest leakage)
for (int i = 0; i < 16; i++) {
    GPIOA->MODER |= (0x03 << (i * 2));  // Analog mode for all PA pins
}
```

### 6. Sensor Power Gating

```c
// Power sensor via MOSFET, only on when needed
void sensor_power_ctrl(uint8_t on) {
    HAL_GPIO_WritePin(SENSOR_POWER_PORT, SENSOR_POWER_PIN,
                      on ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_Delay(5);  // wait for sensor power-up
}
```

## ADC Sampling Strategy for Low Power

| Strategy | Power | Accuracy | When |
|----------|-------|----------|------|
| Continuous conversion | Highest | Best | Must monitor in real-time |
| Timer-triggered scan | Medium | Good | Periodic sampling |
| One-shot + stop | Lowest | Good | Infrequent, event-driven |
| Inject conversion | Low | Good | Interrupt normal conversion sequence |

**One-shot + stop pattern:**

```c
void adc_sample_and_sleep(uint32_t channel) {
    HAL_ADC_Start(&hadc);
    HAL_ADC_PollForConversion(&hadc, 10);
    uint16_t val = HAL_ADC_GetValue(&hadc);
    HAL_ADC_Stop(&hadc);

    // Now enter stop
    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
}
```

## Measurement Technique

| Tool | Measures | Notes |
|------|----------|-------|
| Multimeter (mA range) | Average over seconds | Good for steady-state, misses short spikes |
| Oscilloscope + shunt resistor | Real-time current waveform | Use 10Ω shunt, measure voltage drop |
| Power profiler (nRF PPK, Joulescope) | µA-accuracy, time-correlated | Best tool for battery life debugging |
| e-meter (uCurrent Gold) | µA-range precision | Add between DUT and PSU |

**Common measurement mistake:** The MCU spends 99% of time in STOP at 5 µA, but wakes for 2 ms every 100 ms at 10 mA = ~250 µA average, not 5 µA.

## Common Anti-patterns

| Anti-pattern | Fix |
|-------------|-----|
| Entering STOP with floating pins | Set unused pins to analog or pull-down |
| Waking every ms for a sensor that updates every 100 ms | Use RTC alarm at 100 ms |
| Debugger left connected during power measurement | Disconnect SWD — debugger pulls µA |
| LDO regulator when DCDC is available | Switch to DCDC mode |
| Polling GPIO in while(1) with no idle | Use EXTI interrupt instead |
| All peripherals clocked at max speed | Reduce HCLK/PCLK when full speed not needed |

## Power Budget Calculation Template

```
Budget item          Current    Duty Cycle    Average
─────────────────────────────────────────────────────
Run (active)         10 mA      1%             0.10 mA
Communication (TX)   30 mA      0.1%           0.03 mA
Sleep (STOP)          5 µA      98.9%          0.005 mA
─────────────────────────────────────────────────────
Total average                                 ~0.135 mA

Battery 200 mAh / 0.135 mA ≈ 1481 hours ≈ 61 days
```

## Verification Checklist

- [ ] All unused GPIOs set to analog mode or known state before sleep
- [ ] Peripheral clocks disabled when not in use (`RCC->xxENR` checked)
- [ ] Wake-up source selected and verified (EXTI/RTC)
- [ ] Average current measured over full cycle (not peak, not idle-only)
- [ ] Voltage regulator configured for low-power mode
- [ ] Debugger/SWD disconnected during current measurement
- [ ] Flash/SRAM wait states configured for lower clock speed
- [ ] Unused analog peripherals (ADC/DAC/COMP) disabled in stop mode
