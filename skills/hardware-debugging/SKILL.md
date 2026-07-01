---
name: hardware-debugging
description: Embedded hardware debugging techniques. Covers logic analyzer usage, oscilloscope measurements, SWD/JTAG debug, protocol decoding, and common signal integrity issues.
---

# Hardware Debugging

## Debug Tool Selection

| Tool | Best For | Resolution | Cost |
|------|----------|-----------|------|
| Logic analyzer (8-24 ch) | Digital protocols (I2C, SPI, UART, PWM) | ns | $10-500 |
| Oscilloscope (2-4 ch) | Analog signals, timing, noise | ps | $200-5000+ |
| SWD/JTAG debugger | MCU halt, step, breakpoint, register dump | N/A | $20-500 |
| Protocol analyzer | USB, CAN, Ethernet | N/A | $100-5000 |

## Logic Analyzer: Protocol Decoding

### I2C — Check these first

```
SDA high-to-low while SCL high = START condition
After START: 7-bit address + R/W bit + ACK
```

**Common I2C faults:**
- No START: SDA or SCL stuck high — pull-up resistors missing or too weak
- No ACK: Wrong address, device not powered, wrong voltage level
- Clock stretching not handled: Slave holds SCL low, master doesn't wait
- Bus contention: Two masters or address conflict

### SPI — Check these first

```
SCK idle polarity, data sampled on which edge = mode mismatch (most common bug)
```

| Mode | CPOL | CPHA | Data sampled on |
|------|------|------|-----------------|
| 0 | 0 | 0 | Rising edge |
| 1 | 0 | 1 | Falling edge |
| 2 | 1 | 0 | Falling edge |
| 3 | 1 | 1 | Rising edge |

### UART — Check these first

```
Baud rate mismatch < 2% tolerance
Idle line = high (stop bit = 1)
Start bit = low
```

## Oscilloscope: Signal Integrity

### Probing Technique

- Use 10x probe (less capacitive loading)
- Ground spring, not ground lead (reduces loop inductance)
- Probe at the target pin, not at the break-out header
- Check both SDA and SCL (or TX and RX) simultaneously

### Symptoms vs Cause

| Scope Shows | Likely Cause |
|-------------|-------------|
| Ringing on edges (>10% overshoot) | Long PCB trace, no series termination resistor |
| Slow rise time | Too weak pull-up, too much bus capacitance |
| Glitch on clock line | Crosstalk from adjacent switching signal |
| Non-monotonic edge | Impedance mismatch, reflections |
| DC offset on AC-coupled line | Leakage from adjacent pin |
| 50/60 Hz hum on analog signal | Probe ground loop, poor shielding |

## SWD/JTAG Debugging

### Cortex-M Register Recovery

```text
1. Halt:     debugger issues halt request
2. PC:       read R15 (program counter)
3. LR:       read R14 (link register — tells you where it was called from)
4. PSP/MSP:  read process/main stack pointer
5. Stack:    dump memory at SP to reconstruct call stack
```

### Hard Fault Analysis

```c
void HardFault_Handler(void) {
    // Read SCB->CFSR (Configurable Fault Status Register)
    uint32_t cfsr = SCB->CFSR;
    // Bits [7:0]   = UsageFault
    // Bits [15:8]  = BusFault
    // Bits [23:16] = MemManage
    // If BusFault: SCB->BFAR has the fault address
    // If MemManage: SCB->MMFAR has the fault address

    // SCB->HFSR bit 30 = forced hard fault (deferred from lower-level fault)
    uint32_t hfsr = SCB->HFSR;

    __BKPT(0);  // breakpoint trap for debugger
    while(1);
}
```

### Watchdog Debugging

```c
// Temporarily disable IWDG during debug session
// Option 1: Hold debug bit (if MCU supports it)
DBGMCU->CR |= DBGMCU_CR_IWDG_STOP;

// Option 2: Extend timeout massively for debugging
// Option 3: #define away IWDG refresh in debug build
#ifndef NDEBUG
    #define IWDG_REFRESH() ((void)0)
#else
    #define IWDG_REFRESH() IWDG->KR = 0xAAAA
#endif
```

## Common Debug Workflow

```
1. DOES IT POWER?
   → Measure VDD at MCU pin, check 3.3V/1.8V, check for short circuits

2. DOES IT CLOCK?
   → Probe OSC pins or MCO output, check HSI/HSE/PLL with oscilloscope

3. DOES IT BOOT?
   → Check RESET pin (should be high), check BOOT0/BOOT1 voltage
   → Probe SWO/SWDIO for activity after reset

4. DOES IT COMMUNICATE?
   → Check UART TX pin with scope (should show boot message)
   → Check I2C SDA/SCL with logic analyzer

5. DOES IT CONTROL?
   → Force GPIO with debugger, measure voltage at pin
   → Check PWM pin with scope
```

## Verification Checklist

- [ ] Power rail measured at MCU pin (not just at LDO output)
- [ ] Clock frequency measured (MCO or scope probe)
- [ ] Reset pin stable at VDD (no glitches from brown-out)
- [ ] SWD/JTAG pins not used as GPIO during development
- [ ] UART baud rate within 2% of target (scope measurement)
- [ ] All I2C devices ACK their address on bus scan
- [ ] SPI mode matches between master and slave
- [ ] Watchdog can be disabled during debug (DBGMCU or build flag)
