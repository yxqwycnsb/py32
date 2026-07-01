---
name: rtos-task-design
description: RTOS任务设计模式，适用于FreeRTOS、RT-Thread、uC/OS等嵌入式实时操作系统。涵盖任务拆分、同步原语、队列设计、优先级分配及常见反模式。
---

# RTOS 任务设计模式

## 任务拆分原则

1. **每个任务一个触发源**：定时器、队列消息、事件标志——三选一
2. **单调速率优先级**：速率越高→优先级越高
3. **ISR最小化**：只置标志/发信号量，工作放任务里
4. **ISR中不用阻塞式通信**：只用 `...FromISR()` 系列

## 常用任务类型

| 任务类型 | 触发方式 | 优先级 | 栈大小 | 典型周期 |
|---------|---------|--------|--------|---------|
| 控制环路 | 定时器 | 最高 | 256-512 | 1-10 ms |
| 传感器读取 | 定时器 | 高 | 256-512 | 10-100 ms |
| 通信处理 | 队列/消息 | 高 | 512-1024 | 事件驱动 |
| UI/显示 | 定时器 | 中 | 1024-2048 | 50-200 ms |
| 日志/监控 | 定时器 | 低 | 256-512 | 500-5000 ms |
| 空闲/后台 | Idle hook | 最低 | 128-256 | 持续 |

## 通信模式

### 队列：一生产者一消费者

```c
// 生产者（ISR或任务）
int32_t data = read_sensor();
BaseType_t woke = pdFALSE;
xQueueSendFromISR(sensor_queue, &data, &woke);
portYIELD_FROM_ISR(woke);

// 消费者（任务）
int32_t data;
if (xQueueReceive(sensor_queue, &data, pdMS_TO_TICKS(100)) == pdPASS) {
    process(data);
}
```

### 事件组：多条件等待

```c
#define BIT_BUTTON   (1 << 0)
#define BIT_TIMEOUT  (1 << 1)
#define BIT_SENSOR   (1 << 2)

EventBits_t bits = xEventGroupWaitBits(evt_group,
    BIT_BUTTON | BIT_TIMEOUT | BIT_SENSOR,
    pdTRUE, pdFALSE, pdMS_TO_TICKS(1000));
if (bits & BIT_BUTTON) { handle_button(); }
```

### 流缓冲区：变长数据

```c
// ISR
BaseType_t woke = pdFALSE;
uint8_t byte = USART_ReceiveData(USART1);
xStreamBufferSendFromISR(rx_stream, &byte, 1, &woke);
portYIELD_FROM_ISR(woke);

// 任务
uint8_t buf[64];
size_t n = xStreamBufferReceive(rx_stream, buf, sizeof(buf), portMAX_DELAY);
parse_frame(buf, n);
```

## 同步模式

### 二值信号量：ISR → 任务通知

```c
// ISR
BaseType_t woke = pdFALSE;
xSemaphoreGiveFromISR(adc_done_sem, &woke);
portYIELD_FROM_ISR(woke);

// 任务等待
xSemaphoreTake(adc_done_sem, portMAX_DELAY);
proceed_with_adc_data();
```

### 互斥量：共享资源保护（带优先级继承）

```c
xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(100));
bsp_i2c_write_reg(...);  // 同一时刻只有一个任务访问I2C
xSemaphoreGive(i2c_mutex);
```

### 任务通知：轻量级单等待者

```c
// 任务A等待
uint32_t val;
xTaskNotifyWait(0, ULONG_MAX, &val, portMAX_DELAY);

// 任务B（或ISR）通知
xTaskNotify(task_a_handle, sensor_value, eSetValueWithOverwrite);
```

## 优先级分配规则

1. ISR > 任务（永远）
2. 短周期 > 长周期（同速率下）
3. 事件驱动 > 时间触发（同速率下）
4. 控制 > 监控 > 日志
5. 通信任务和控制在任务之间至少留2级优先级差，防止优先级反转

## 常见反模式

| 反模式 | 修正方案 |
|--------|---------|
| 控制循环中用 `vTaskDelay()` | 改用定时器或任务通知 |
| 50+ 个优先级 | 5-7级足够，用任务设计实现子优先级 |
| 共享数据不加锁 | 用队列或互斥量保护 |
| ISR向多个任务发通知 | 用事件组或队列 |
| 任务栈分配过大 | 用 `uxTaskGetStackHighWaterMark()` 测量 |
| 运行时创建/删除任务 | 启动时预创建所有任务，用挂起/恢复 |
| Idle hook中阻塞 | Idle hook绝对不能阻塞——会卡死调度 |

## 验收检查清单

- [ ] 每个任务有明确的用途和触发方式
- [ ] 栈大小已用 `uxTaskGetStackHighWaterMark()` 验证
- [ ] ISR→任务通信全部使用 `...FromISR()` API
- [ ] 控制和传感器任务中无 `vTaskDelay()`（用定时器/通知替代）
- [ ] 资源保护使用互斥量（非二值信号量）
- [ ] 优先级分配有文档记录和依据
- [ ] 堆大小足够最坏情况分配
- [ ] Idle hook 不阻塞
