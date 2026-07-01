#include "x_v2_usart.h"

// 接收一帧完整数据的标志位
__IO bool rxFrameFlag = false;
// 串口接收缓存
__IO uint8_t rxCmd[FIFO_SIZE] = {0};
// 当前已接收字节数
__IO uint8_t rxCount = 0;

// USART2 初始化，作为 X_V2 电机通信串口使用
void x_v2_usart2_init(uint32_t baudrate)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitTypeDef gpio;
    gpio.GPIO_Pin = GPIO_Pin_2;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &gpio);

    gpio.GPIO_Pin = GPIO_Pin_3;
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
    USART_Init(USART2, &usart);

    // 开启接收中断
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

    NVIC_InitTypeDef nvic;
    nvic.NVIC_IRQChannel = USART2_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 6;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    USART_Cmd(USART2, ENABLE);
}

// 发送一个字节
void usart_SendByte(uint16_t data)
{
    volatile uint32_t timeout = 1000000;
    while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET)
    {
        if (--timeout == 0) return;
    }
    USART_SendData(USART2, data);
}

// 发送一段命令数据
void usart_SendCmd(__IO uint8_t *cmd, uint8_t len)
{
    for (uint8_t i = 0; i < len; i++)
    {
        usart_SendByte(cmd[i]);
    }
}

// USART2 接收中断处理函数
void USART2_IRQHandler(void)
{
    if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
    {
        uint8_t data = (uint8_t)USART_ReceiveData(USART2);
        if (rxCount < FIFO_SIZE)
        {
            rxCmd[rxCount++] = data;
        }
        // 简单按 8 字节作为一帧处理
        if (rxCount >= 8)
        {
            rxFrameFlag = true;
        }
        USART_ClearITPendingBit(USART2, USART_IT_RXNE);
    }
}
