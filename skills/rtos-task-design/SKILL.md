---
name: rtos-task-design
description: RTOS task design patterns for embedded systems (FreeRTOS, RT-Thread, uC/OS). Covers task splitting, synchronization, queue design, priority assignment, and common anti-patterns.
---

# RTOS Task Design Patterns

## Task Splitting Principles

1. **One trigger per task**: Timer tick, queue message, or event flag — not all three
2. **Rate-monotonic priority**: Higher rate → higher priority
3. **ISR does minimum**: Set flag/semaphore, exit. Work in task
4. **No blocking communication inside ISR**: Use `...FromISR()` variants only

## Common Task Archetypes

| Task Type | Trigger | Priority | Stack | Typical Period |
|-----------|---------|----------|-------|----------------|
| Control loop | Timer | Highest | 256-512 | 1-10 ms |
| Sensor read | Timer | High | 256-512 | 10-100 ms |
| Communication | Queue/Msg | High | 512-1024 | Event-driven |
| UI/Display | Timer | Medium | 1024-2048 | 50-200 ms |
| Logging/Monitor | Timer | Low | 256-512 | 500-5000 ms |
| Idle/Background | Idle hook | Lowest | 128-256 | Continuous |

## Communication Patterns

### Queue: One producer, one consumer

```c
// Producer (ISR or task)
int32_t data = read_sensor();
BaseType_t woke = pdFALSE;
xQueueSendFromISR(sensor_queue, &data, &woke);
portYIELD_FROM_ISR(woke);

// Consumer (task)
int32_t data;
if (xQueueReceive(sensor_queue, &data, pdMS_TO_TICKS(100)) == pdPASS) {
    process(data);
}
```

### Event Group: Multiple conditions

```c
// Bits
#define BIT_BUTTON   (1 << 0)
#define BIT_TIMEOUT  (1 << 1)
#define BIT_SENSOR   (1 << 2)

// Waiter
EventBits_t bits = xEventGroupWaitBits(evt_group,
    BIT_BUTTON | BIT_TIMEOUT | BIT_SENSOR,
    pdTRUE, pdFALSE, pdMS_TO_TICKS(1000));

if (bits & BIT_BUTTON) { handle_button(); }
```

### Stream Buffer: Variable-length data

```c
// ISR
BaseType_t woke = pdFALSE;
uint8_t byte;
while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE)) {
    byte = USART_ReceiveData(USART1);
    xStreamBufferSendFromISR(rx_stream, &byte, 1, &woke);
}
portYIELD_FROM_ISR(woke);

// Task
uint8_t buf[64];
size_t n = xStreamBufferReceive(rx_stream, buf, sizeof(buf), portMAX_DELAY);
parse_frame(buf, n);
```

## Synchronization Patterns

### Binary Semaphore: ISR → Task notification

```c
// ISR
BaseType_t woke = pdFALSE;
xSemaphoreGiveFromISR(adc_done_sem, &woke);
portYIELD_FROM_ISR(woke);

// Task waits
xSemaphoreTake(adc_done_sem, portMAX_DELAY);
uint16_t val = read_adc_result();
```

### Mutex: Shared resource protection

```c
xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(100));
bsp_i2c_write_reg(...);  // only one task accesses I2C at a time
xSemaphoreGive(i2c_mutex);
```

**Note:** Use mutex (with priority inheritance), not binary semaphore, for resource protection.

### Task Notification: Lightweight single-waiter

```c
// Task A
uint32_t val;
xTaskNotifyWait(0, ULONG_MAX, &val, portMAX_DELAY);

// Task B (or ISR)
xTaskNotify(task_a_handle, sensor_value, eSetValueWithOverwrite);
```

## Priority Assignment Rules

1. ISR > Task (always)
2. Short duration > Long duration (at same rate)
3. Event-driven > Time-triggered (at same rate)
4. Control > Monitor > Logging
5. Minimum 2 priority levels gap between communication and control tasks (to prevent priority inversion)

## Typical FreeRTOSConfig.h Settings

```c
#define configUSE_PREEMPTION          1
#define configUSE_IDLE_HOOK           0
#define configUSE_TICK_HOOK           0
#define configTICK_RATE_HZ          1000    // 1 ms tick
#define configMAX_PRIORITIES           5     // keep it small
#define configMINIMAL_STACK_SIZE     128     // words, not bytes
#define configTOTAL_HEAP_SIZE       (4096 * 4)  // 16 KB
```

## Common Anti-patterns

| Anti-pattern | Fix |
|-------------|-----|
| `vTaskDelay()` in control loop | Use timer or notification, not delay |
| 50+ priority levels | 5-7 levels is enough; use sub-priority via task design |
| Shared data without mutex | Use queues or mutex-protected access |
| ISR sending to multiple tasks | Use event group or queue, not direct ISR→task calls |
| Task stacks oversized | Measure with `uxTaskGetStackHighWaterMark()` |
| Creating/destroying tasks at runtime | Pre-create all tasks at boot, use suspend/resume |
| Blocking in idle hook | Never block in idle hook — it halts task scheduling |

## Verification Checklist

- [ ] Every task has a documented purpose and trigger
- [ ] Stack sizes verified via `uxTaskGetStackHighWaterMark()`
- [ ] All ISR→task communication uses `...FromISR()` APIs
- [ ] No `vTaskDelay()` in control or sensor tasks (use timer/notification)
- [ ] Mutex used for resource protection (not binary semaphore)
- [ ] Priority assignment documented and justified
- [ ] Heap size sufficient for worst-case allocation
- [ ] Idle hook does not block
