#include "worklog_dump.h"
#include "worklog.h"
#include "usart.h"
#include "stm32f10x.h"

#define WORKLOG_BASE_ADDR   ((uint32_t)0x0800F000)
#define WORKLOG_PAGE_SIZE   1024U
#define WORKLOG_PAGES       4U
#define WORKLOG_TOTAL_SIZE  (WORKLOG_PAGE_SIZE * WORKLOG_PAGES)
#define WORKLOG_MAX_RECORDS (WORKLOG_TOTAL_SIZE / sizeof(WorkLogRecord_t))

static uint32_t _is_empty_record(const WorkLogRecord_t *rec)
{
    const uint32_t *p = (const uint32_t *)rec;
    for (uint32_t i = 0; i < sizeof(WorkLogRecord_t) / 4; i++) {
        if (p[i] != 0xFFFFFFFF) return 0;
    }
    return 1;
}

static void _print_record(const WorkLogRecord_t *rec)
{
    usart_send_string("T=");
    usart_send_u32(rec->tick);
    usart_send_string(" E=");
    usart_send_u32(rec->event);
    usart_send_string(" P=");
    usart_send_i32(rec->pulse);
    usart_send_string(" R=");
    usart_send_u32(rec->raw_level);
    usart_send_string(" D=");
    usart_send_u32(rec->debounced_level);
    usart_send_string("\r\n");
}

void worklog_dump(void)
{
    const WorkLogRecord_t *flash = (const WorkLogRecord_t *)WORKLOG_BASE_ADDR;
    usart_send_string("WORKLOG DUMP BEGIN\r\n");
    for (uint32_t i = 0; i < WORKLOG_MAX_RECORDS; i++) {
        if (_is_empty_record(&flash[i])) {
            break;
        }
        _print_record(&flash[i]);
    }
    usart_send_string("WORKLOG DUMP END\r\n");
}
