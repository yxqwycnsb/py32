---
name: hardware-debugging
description: 嵌入式硬件调试技术。涵盖逻辑分析仪使用、示波器测量、SWD/JTAG调试、协议解码及常见信号完整性问题。
---

# 硬件调试

## 调试工具选择

| 工具 | 最适合 | 分辨率 | 成本 |
|------|--------|--------|------|
| 逻辑分析仪(8-24ch) | 数字协议(I2C, SPI, UART, PWM) | ns | $10-500 |
| 示波器(2-4ch) | 模拟信号、时序、噪声 | ps | $200-5000+ |
| SWD/JTAG调试器 | MCU暂停、单步、断点、寄存器 | N/A | $20-500 |
| 协议分析仪 | USB, CAN, 以太网 | N/A | $100-5000 |

## 逻辑分析仪：协议解码

### I2C——先查这些

```
SCL为高时SDA从高变低 = START条件
START后：7位地址 + R/W位 + ACK
```

**常见I2C故障：**
- 无START：SDA或SCL卡在高电平——上拉电阻缺失或太弱
- 无ACK：地址错误、设备未上电、电平不匹配
- 未处理时钟拉伸：从机拉住SCL，主机未等待
- 总线冲突：两个主机或地址冲突

### SPI——先查这些

```
SCK空闲极性、数据在哪个边沿采样 = 模式不匹配（最常见问题）
```

| 模式 | CPOL | CPHA | 数据采样 |
|------|------|------|---------|
| 0 | 0 | 0 | 上升沿 |
| 1 | 0 | 1 | 下降沿 |
| 2 | 1 | 0 | 下降沿 |
| 3 | 1 | 1 | 上升沿 |

### UART——先查这些

```
波特率误差 < 2%
空闲线 = 高电平（停止位=1）
起始位 = 低电平
```

## SWD/JTAG调试

### Cortex-M寄存器恢复

```
1. 暂停：   调试器发halt请求
2. PC：     读R15（程序计数器）
3. LR：     读R14（链接寄存器——知道谁调了当前函数）
4. PSP/MSP：读取进程/主栈指针
5. 栈回溯：  dump栈内存重建调用栈
```

### Hard Fault分析

```c
void HardFault_Handler(void) {
    uint32_t cfsr = SCB->CFSR;
    // Bits [7:0]   = UsageFault
    // Bits [15:8]  = BusFault
    // Bits [23:16] = MemManage
    // BusFault时: SCB->BFAR有故障地址
    // MemManage时: SCB->MMFAR有故障地址

    uint32_t hfsr = SCB->HFSR;
    // bit 30 = forced hard fault（来自更底层故障）
    while(1);
}
```

### 看门狗调试

```c
// 调试会话中临时禁IWDG
// 方案1：停调试位（如果MCU支持）
DBGMCU->CR |= DBGMCU_CR_IWDG_STOP;

// 方案2：调试版本宏替换
#ifndef NDEBUG
    #define IWDG_REFRESH() ((void)0)
#else
    #define IWDG_REFRESH() IWDG->KR = 0xAAAA
#endif
```

## 常用调试流程

```
1. 有电吗？
   → 测MCU引脚VDD，检查3.3V/1.8V，检查短路

2. 有时钟吗？
   → 用示波器测晶振引脚或MCO输出，检查HSI/HSE/PLL

3. 能启动吗？
   → 检查RESET引脚应为高，检查BOOT0/BOOT1电压
   → 复位后用示波器测SWO/SWDIO有无活动

4. 能通信吗？
   → 示波器测UART TX引脚（应有启动信息）
   → 逻辑分析仪测I2C SDA/SCL

5. 能控制吗？
   → 调试器强制GPIO电平，测引脚电压
   → 示波器测PWM引脚
```

## 验收检查清单

- [ ] 电源电压在MCU引脚处测量（非LDO输出端）
- [ ] 时钟频率已实测（MCO或示波器）
- [ ] 复位引脚稳定在高电平（无欠压抖动）
- [ ] 开发期间SWD/JTAG引脚未用作GPIO
- [ ] UART波特率误差<2%（示波器实测）
- [ ] 所有I2C设备总线扫描时有ACK应答
- [ ] SPI主从模式匹配
- [ ] 调试时可禁看门狗（DBGMCU或构建选项）
