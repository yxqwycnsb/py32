---
name: bootloader-ota
description: MCU Bootloader设计和OTA固件升级模式。涵盖Flash分区、双Bank切换、恢复机制、固件签名和升级协议设计。
---

# Bootloader 与 OTA 升级

## Flash 内存布局

```
0x0800 0000  +──────────────────────+
              │ Bootloader           │  32 KB
0x0800 8000  ├──────────────────────┤
              │ 固件 Slot A (运行中)  │  192 KB
0x0803 8000  ├──────────────────────┤
              │ 固件 Slot B (备份)    │  192 KB
0x0806 8000  ├──────────────────────┤
              │ 配置/标志位           │  4 KB
0x0806 9000  ├──────────────────────┤
              │ 工作日志/诊断         │  剩余
0x0808 0000  └──────────────────────┘
```

### 标志位结构

```c
typedef struct __attribute__((packed)) {
    uint32_t magic;            // 0xA5A5A5A5
    uint32_t version;          // 固件版本号
    uint32_t boot_slot;        // 0=Slot A, 1=Slot B
    uint32_t update_pending;   // 0=否, 1=是
    uint32_t crc32;            // 结构体CRC
} boot_flags_t;
```

## 启动流程

```c
void bootloader_main(void) {
    boot_flags_t flags = read_flags();

    if (flags.magic != 0xA5A5A5A5) {
        flags = (boot_flags_t){ .magic = 0xA5A5A5A5, .boot_slot = 0 };
        write_flags(&flags);
    }

    if (flags.update_pending) {
        uint32_t new_slot = (flags.boot_slot == 0) ? SLOT_B : SLOT_A;
        if (validate_firmware(new_slot)) {
            // 验证通过，切换槽位
            flags.boot_slot ^= 1;
            flags.update_pending = 0;
            write_flags(&flags);
        } else {
            // 验证失败，回退
            flags.update_pending = 0;
            write_flags(&flags);
        }
    }

    jump_to_app(flags.boot_slot);
}
```

## OTA 升级协议

### 最小协议设计

```
[主机]                            [设备]
  |                                |
  |--- PREPARE (总大小, CRC) ----->|
  |<-- READY / REJECT ------------|
  |--- DATA (序号, 偏移, 块) ---->|
  |<-- ACK -----------------------|
  |--- ... 重复 ...               |
  |--- FINISH (验证CRC) --------->|
  |<-- OK / FAIL -----------------|
  |--- COMMIT ------------------->|
  |<-- RESET ---------------------|
```

## 固件验证

```c
bool validate_firmware(uint32_t slot_addr) {
    // 1. 检查栈指针是否在SRAM范围内
    uint32_t msp = *(volatile uint32_t *)slot_addr;
    if (msp < SRAM_BASE || msp >= (SRAM_BASE + SRAM_SIZE)) return false;

    // 2. 检查复位向量是否在Flash范围内
    uint32_t reset_vec = *(volatile uint32_t *)(slot_addr + 4);
    if (reset_vec < FLASH_BASE || reset_vec >= (FLASH_BASE + FLASH_SIZE)) return false;

    // 3. CRC校验
    uint32_t fw_size = read_firmware_size(slot_addr);
    uint32_t stored_crc = read_firmware_crc(slot_addr);
    if (crc32((uint8_t *)slot_addr, fw_size) != stored_crc) return false;

    // 4. 签名验证（生产环境）
    if (!verify_signature(slot_addr)) return false;

    return true;
}
```

## 安全与恢复机制

| 机制 | 目的 |
|------|------|
| 双Bank切换 | 升级失败可回滚 |
| 逐块CRC+全固件CRC | 检测传输和存储中的损坏 |
| 标志位魔数 | 检测未初始化或损坏的配置 |
| 外部看门狗 | 从bootloader卡死中恢复 |
| 回退定时器 | 新固件N秒无响应则回退 |
| 掉电检测 | 防止写入过程中断电导致数据损坏 |

## 验收检查清单

- [ ] Bootloader小于预留的Flash空间
- [ ] 跳转前已关闭所有中断
- [ ] VTOR在跳转前正确设置
- [ ] 跳转前检查栈指针合法性
- [ ] 固件启动前验证CRC
- [ ] 升级过程原子化（掉电可回滚）
- [ ] Bootloader可独立升级（双Bank模式）
- [ ] 新固件启动失败有回退机制
- [ ] 生产固件有签名验证
- [ ] Flash按页擦除，非逐字节擦写
