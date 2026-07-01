#include "usart.h"
#include "stm32f10x.h"

// USART1: PA9(TX) / PA10(RX)
void usart_init(uint32_t baudrate)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1, ENABLE);

    GPIO_InitTypeDef gpio;
    // TX PA9
    gpio.GPIO_Pin = GPIO_Pin_9;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &gpio);

    // RX PA10
    gpio.GPIO_Pin = GPIO_Pin_10;
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &gpio);

    USART_InitTypeDef usart;
    USART_StructInit(&usart);
    usart.USART_BaudRate = baudrate;
    usart.USART_WordLength = USART_WordLength_8b;
    usart.USART_StopBits = USART_StopBits_1;
    usart.USART_Parity = USART_Parity_No;
    usart.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART1, &usart);

    USART_Cmd(USART1, ENABLE);
}

void usart_send_char(char ch)
{
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET) { }
    USART_SendData(USART1, (uint16_t)ch);
}

void usart_send_string(const char *s)
{
    if (!s) return;
    while (*s) {
        usart_send_char(*s++);
    }
}

static void _send_u32_hex(uint32_t value)
{
    for (int i = 28; i >= 0; i -= 4) {
        uint8_t nibble = (value >> i) & 0xF;
        char c = (nibble < 10) ? ('0' + nibble) : ('A' + (nibble - 10));
        usart_send_char(c);
    }
}

void usart_send_u32(uint32_t value)
{
    _send_u32_hex(value);
}

void usart_send_i32(int32_t value)
{
    if (value < 0) {
        usart_send_char('-');
        value = -value;
    }
    _send_u32_hex((uint32_t)value);
}

void usart_send_u32_dec(uint32_t value)
{
    char buf[10];
    int i = 0;
    if (value == 0) {
        usart_send_char('0');
        return;
    }
    while (value > 0 && i < (int)sizeof(buf)) {
        buf[i++] = (char)('0' + (value % 10));
        value /= 10;
    }
    while (i-- > 0) {
        usart_send_char(buf[i]);
    }
}

void usart_send_i32_dec(int32_t value)
{
    if (value < 0) {
        usart_send_char('-');
        value = -value;
    }
    usart_send_u32_dec((uint32_t)value);
}
