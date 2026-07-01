---
name: bootloader-ota
description: Bootloader design and OTA firmware update patterns for MCUs. Covers flash partitioning, dual-bank swap, recovery mechanisms, firmware signing, and update protocol design.
---

# Bootloader & OTA Update

## Flash Memory Layout

```
0x0800 0000  +──────────────────────+
              │ Bootloader           │  32 KB (sector 0)
0x0800 8000  ├──────────────────────┤
              │ Firmware Slot A      │  192 KB (active)
0x0803 8000  ├──────────────────────┤
              │ Firmware Slot B      │  192 KB (backup)
0x0806 8000  ├──────────────────────┤
              │ Settings / Flags     │  4 KB
0x0806 9000  ├──────────────────────┤
              │ Work log / Diag      │   remaining
0x0808 0000  └──────────────────────┘
```

### Flags Sector (4 KB, one erased page)

```c
typedef struct __attribute__((packed)) {
    uint32_t magic;            // 0xA5A5A5A5
    uint32_t version;          // firmware version number
    uint32_t boot_slot;        // 0=Slot A, 1=Slot B
    uint32_t update_pending;   // 0=no, 1=yes
    uint32_t crc32;            // checksum of this struct
} boot_flags_t;
```

## Boot Flow

```c
void bootloader_main(void) {
    boot_flags_t flags = read_flags();

    if (flags.magic != 0xA5A5A5A5) {
        // First boot after programming — bootstrap slot A
        flags = (boot_flags_t){ .magic = 0xA5A5A5A5, .boot_slot = 0 };
        write_flags(&flags);
    }

    if (flags.update_pending) {
        uint32_t new_slot = flags.boot_slot ^ 1;
        bool ok = validate_firmware(new_slot);
        if (ok) {
            flags.boot_slot = new_slot;     // swap active slot
            flags.update_pending = 0;
            write_flags(&flags);
        } else {
            // fallback — boot existing slot
            flags.update_pending = 0;
            write_flags(&flags);
        }
    }

    jump_to_app(flags.boot_slot);
}
```

### Jump to Application

```c
typedef void (*app_entry_t)(void);

void jump_to_app(uint32_t slot) {
    uint32_t app_base = (slot == 0) ? APP_A_BASE : APP_B_BASE;

    // Disable all interrupts
    __disable_irq();
    for (int i = 0; i < 8; i++) NVIC->ICER[i] = 0xFFFFFFFF;

    // Reset SysTick to prevent interrupt after jump
    SysTick->CTRL = 0;

    // Set vector table
    SCB->VTOR = app_base;

    // Set MSP to the first word in the app vector table
    __set_MSP(*(volatile uint32_t *)app_base);

    // Jump to reset handler (second word)
    app_entry_t entry = (app_entry_t)(*(volatile uint32_t *)(app_base + 4));
    entry();
}
```

## OTA Update Protocol

### Minimal Protocol Design

```
[HOST]                          [DEVICE]
  |                                |
  |--- PREPARE (total_size, crc) ->|
  |<-- READY / REJECT ------------|
  |--- DATA (seq, offset, chunk)->|
  |<-- ACK (seq) -----------------|
  |--- ... repeat ...             |
  |--- FINISH (crc_verify) ------>|
  |<-- OK / FAIL -----------------|
  |--- COMMIT ------------------->|
  |<-- RESET ---------------------|
```

### Chunk Transfer

```c
#define OTA_CHUNK_SIZE   256

typedef struct __attribute__((packed)) {
    uint32_t seq;           // sequence number
    uint32_t offset;        // byte offset in firmware
    uint8_t  data[OTA_CHUNK_SIZE];
    uint32_t crc32;         // CRC of this chunk
} ota_data_pkt_t;

int ota_process_chunk(ota_data_pkt_t *pkt, uint32_t slot_addr) {
    if (!validate_chunk_crc(pkt)) return OTA_ERR_CRC;
    if (pkt->offset + OTA_CHUNK_SIZE > MAX_FW_SIZE) return OTA_ERR_SIZE;

    // Erase page if first write to this page
    uint32_t page_addr = slot_addr + pkt->offset;
    if (!is_page_already_erased(page_addr)) {
        flash_erase_page(page_addr);
    }

    flash_write(page_addr, pkt->data, OTA_CHUNK_SIZE);
    return OTA_OK;
}
```

## Firmware Validation

```c
bool validate_firmware(uint32_t slot_addr) {
    // 1. Check vector table: first word must be valid stack address
    uint32_t msp = *(volatile uint32_t *)slot_addr;
    if (msp < SRAM_BASE || msp >= (SRAM_BASE + SRAM_SIZE)) return false;

    // 2. Check reset vector: must be in flash range
    uint32_t reset_vec = *(volatile uint32_t *)(slot_addr + 4);
    if (reset_vec < FLASH_BASE || reset_vec >= (FLASH_BASE + FLASH_SIZE)) return false;

    // 3. CRC check over entire firmware
    uint32_t fw_size = read_firmware_size(slot_addr);
    uint32_t stored_crc = read_firmware_crc(slot_addr);
    uint32_t calc_crc = crc32((uint8_t *)slot_addr, fw_size);
    if (calc_crc != stored_crc) return false;

    // 4. Optional: signature verification
    if (!verify_signature(slot_addr)) return false;

    return true;
}
```

## Safety & Recovery

| Mechanism | Purpose |
|-----------|---------|
| Dual-bank swap | Rollback on failed update |
| CRC per chunk + full CRC | Detect corruption during transfer or storage |
| Magic number in flags | Detect uninitialized or corrupt settings |
| External watchdog (if available) | Recover from bootloader hang |
| Fallback timer | If new firmware doesn't respond within N seconds, revert |
| Brown-out detection | Prevent writing partial data during power loss |

## Verification Checklist

- [ ] Bootloader is smaller than reserved flash area
- [ ] All interrupts disabled before jumping to app
- [ ] Vector table offset (VTOR) set correctly before jump
- [ ] Stack pointer validity checked before jumping
- [ ] Firmware CRC validated before boot
- [ ] Update process atomic (power loss during update can revert)
- [ ] Bootloader can be updated independently (if dual-bank)
- [ ] Fallback mechanism exists if new firmware fails to boot
- [ ] Signature verification for production firmware
- [ ] Flash erasure handled in pages, not byte-by-byte
