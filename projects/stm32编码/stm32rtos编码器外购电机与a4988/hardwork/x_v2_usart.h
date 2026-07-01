#ifndef __X_V2_USART_H
#define __X_V2_USART_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32f10x.h"

// 本工程中没有单独的 fifo.h，接收缓存大小直接在这里定义，避免头文件缺失导致编译失败
#ifndef FIFO_SIZE
#define FIFO_SIZE 64
#endif

#ifdef __cplusplus
extern "C" {
#endif

// 接收完成标志
extern __IO bool rxFrameFlag;
// 接收缓存
extern __IO uint8_t rxCmd[FIFO_SIZE];
// 当前接收字节数
extern __IO uint8_t rxCount;

// USART2 初始化
void x_v2_usart2_init(uint32_t baudrate);
// 发送整段命令
void usart_SendCmd(__IO uint8_t *cmd, uint8_t len);
// 发送单个字节
void usart_SendByte(uint16_t data);

#ifdef __cplusplus
}
#endif

#endif /* __X_V2_USART_H */
