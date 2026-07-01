---
name: low-power-optimization
description: 电池供电MCU设备的低功耗设计模式。涵盖休眠模式、时钟门控、外设关断、ADC采样策略和功耗测量技巧。
---

# 低功耗优化

## 休眠模式层次（ARM Cortex-M）

| 模式 | 电流 | 唤醒延迟 | 唤醒源 | 适用场景 |
|------|------|---------|--------|---------|
| Run | 满电流 | N/A | N/A | 正常处理 |
| Sleep (WFI) | ~50-70% | ~5 us | 任意中断 | 短空闲 100 us-10 ms |
| Stop/深度睡眠 | ~5-20 uA | ~10-100 us | RTC, EXTI, LPUART | 等待周期唤醒或外部事件 |
| Standby | ~0.1-2 uA | ~1-10 ms | 复位, RTC, 唤醒脚 | 长空闲、周期性上报 |
| Shutdown/关机 | ~0 nA | ~10-100 ms | 复位, 上电 | 用户关机、运输模式 |

## 关键策略

### 1. 时钟门控优先

```c
RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_USART2, DISABLE);
RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_SPI1, DISABLE);
```

**检查 RCC->xxENR——很多HAL驱动初始化后不关时钟。**

### 2. STOP模式 + RTC唤醒

```c
void enter_stop_with_rtc(uint32_t seconds) {
    // RTCCLK/16 = 2048 Hz（LSE=32768Hz时）
    // counter = seconds * 2048，最大16位(65535 ≈ 32s)
    uint32_t counter = seconds * 2048U;
    if (counter > 0xFFFF) counter = 0xFFFF;
    HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, counter, RTC_WAKEUPCLOCK_RTCCLK_DIV16);
    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
    SystemClock_Config();
}
```

### 3. GPIO泄漏优化

```c
for (int i = 0; i < 16; i++) {
    GPIOA->MODER |= (0x03 << (i * 2));
}
```

### 4. 外设电源门控

```c
void sensor_power_ctrl(uint8_t on) {
    HAL_GPIO_WritePin(SENSOR_POWER_PORT, SENSOR_POWER_PIN,
                      on ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_Delay(5);
}
```

## ADC低功耗采样策略

| 策略 | 功耗 | 精度 | 适用场景 |
|------|------|------|---------|
| 连续转换 | 最高 | 最好 | 实时监测 |
| 定时器触发 | 中 | 好 | 周期性采样 |
| 单次+休眠 | 最低 | 好 | 低频事件驱动 |
| 注入转换 | 低 | 好 | 中断正常转换序列 |

## 功耗测量工具

| 工具 | 用途 | 说明 |
|------|------|------|
| 万用表(mA档) | 平均电流 | 适合稳态，测不到短时尖峰 |
| 示波器+采样电阻 | 实时电流波形 | 10欧姆采样电阻，测压降 |
| 功率分析仪 | uA级精度 | 电池寿命调试最佳工具 |

**常见误区：** MCU 99%时间在STOP 5uA，但每100ms唤醒2ms运行在10mA，平均约250uA。

## 常见反模式

| 反模式 | 修正方案 |
|--------|---------|
| 浮空引脚进入STOP | 未用引脚设为模拟模式或下拉 |
| 每1ms唤醒但传感器100ms才更新 | 用RTC闹钟100ms唤醒 |
| 调试器未断开测功耗 | 断开SWD——调试器本身耗电 |
| 有DCDC却用LDO | 切换到DCDC模式 |
| while(1)轮询GPIO | 改用EXTI中断 |
| 所有外设全速运行 | 降频使用 |

## 功耗预算计算模板

```
项目                电流       占空比     平均
Run (运行)          10 mA      1%         0.10 mA
通信 (发射)         30 mA      0.1%       0.03 mA
睡眠 (STOP)          5 uA      98.9%      0.005 mA
合计平均                                    0.135 mA
电池 200mAh / 0.135mA 约 61 天
```

## 验收检查清单

- [ ] 所有未用GPIO在休眠前设模拟模式或已知状态
- [ ] 不用的外设时钟已关闭
- [ ] 唤醒源已选定并验证（EXTI/RTC）
- [ ] 测量了完整工作周期的平均电流
- [ ] 电压调节器已配为低功耗模式
- [ ] 测量电流时已断开调试器/SWD
- [ ] 未用模拟外设(ADC/DAC/COMP)在stop模式已关闭
