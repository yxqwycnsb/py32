#ifndef __USART_SIMPLE_H
#define __USART_SIMPLE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void usart_init(uint32_t baudrate);
void usart_send_char(char ch);
void usart_send_string(const char *s);
void usart_send_u32(uint32_t value);
void usart_send_i32(int32_t value);
void usart_send_u32_dec(uint32_t value);
void usart_send_i32_dec(int32_t value);

#ifdef __cplusplus
}
#endif

#endif /* __USART_SIMPLE_H */
