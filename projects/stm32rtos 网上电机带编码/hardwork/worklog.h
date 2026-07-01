#ifndef __WORKLOG_H
#define __WORKLOG_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// 日志事件类型
typedef enum {
    WORKLOG_EVT_INIT = 1,
    WORKLOG_EVT_BTN_START,
    WORKLOG_EVT_FIND_TIMEOUT,
    WORKLOG_EVT_SENSOR_TRIGGER,
    WORKLOG_EVT_RETURN_TIMEOUT,
    WORKLOG_EVT_RETURN_DONE,
    WORKLOG_EVT_ERROR
} WorkLogEvent_t;

// 日志记录（16字节）
typedef struct {
    uint32_t tick;
    uint32_t event;
    int32_t  pulse;
    uint16_t raw_level;
    uint16_t debounced_level;
} WorkLogRecord_t;

void worklog_init(void);
void worklog_append(WorkLogEvent_t event, int32_t pulse, bool raw_level, bool debounced_level);
void worklog_clear(void);

#ifdef __cplusplus
}
#endif

#endif /* __WORKLOG_H */
