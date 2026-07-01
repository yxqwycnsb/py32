#include "worklog.h"
#include "stm32f10x.h"
#include "stm32f10x_flash.h"
#include "FreeRTOS.h"
#include "task.h"

// STM32F103C8: 64KB Flash, page size 1KB, base 0x08000000
// 使用最后 4KB（4页）作为日志区
#define WORKLOG_BASE_ADDR   ((uint32_t)0x0800F000)
#define WORKLOG_PAGE_SIZE   1024U
#define WORKLOG_PAGES       4U
#define WORKLOG_TOTAL_SIZE  (WORKLOG_PAGE_SIZE * WORKLOG_PAGES)
#define WORKLOG_MAX_RECORDS (WORKLOG_TOTAL_SIZE / sizeof(WorkLogRecord_t))

static uint32_t s_write_index = 0;

static uint32_t _addr_from_index(uint32_t index)
{
    return WORKLOG_BASE_ADDR + index * sizeof(WorkLogRecord_t);
}

static uint32_t _is_empty_record(const WorkLogRecord_t *rec)
{
    const uint32_t *p = (const uint32_t *)rec;
    for (uint32_t i = 0; i < sizeof(WorkLogRecord_t) / 4; i++) {
        if (p[i] != 0xFFFFFFFF) return 0;
    }
    return 1;
}

void worklog_init(void)
{
    s_write_index = 0;
    const WorkLogRecord_t *flash = (const WorkLogRecord_t *)WORKLOG_BASE_ADDR;
    for (uint32_t i = 0; i < WORKLOG_MAX_RECORDS; i++) {
        if (_is_empty_record(&flash[i])) {
            s_write_index = i;
            break;
        }
    }
    if (s_write_index >= WORKLOG_MAX_RECORDS) {
        s_write_index = WORKLOG_MAX_RECORDS; // 表示已满
    }
}

void worklog_clear(void)
{
    taskENTER_CRITICAL();
    FLASH_Unlock();
    for (uint32_t i = 0; i < WORKLOG_PAGES; i++) {
        FLASH_ErasePage(WORKLOG_BASE_ADDR + i * WORKLOG_PAGE_SIZE);
    }
    FLASH_Lock();
    taskEXIT_CRITICAL();
    s_write_index = 0;
}

void worklog_append(WorkLogEvent_t event, int32_t pulse, bool raw_level, bool debounced_level)
{
    if (s_write_index >= WORKLOG_MAX_RECORDS) return;

    WorkLogRecord_t rec;
    rec.tick = xTaskGetTickCount();
    rec.event = (uint32_t)event;
    rec.pulse = pulse;
    rec.raw_level = raw_level ? 1 : 0;
    rec.debounced_level = debounced_level ? 1 : 0;

    uint32_t addr = _addr_from_index(s_write_index);

    taskENTER_CRITICAL();
    FLASH_Unlock();
    FLASH_ProgramWord(addr + 0, rec.tick);
    FLASH_ProgramWord(addr + 4, rec.event);
    FLASH_ProgramWord(addr + 8, (uint32_t)rec.pulse);
    FLASH_ProgramHalfWord(addr + 12, rec.raw_level);
    FLASH_ProgramHalfWord(addr + 14, rec.debounced_level);
    FLASH_Lock();
    taskEXIT_CRITICAL();

    s_write_index++;
}
