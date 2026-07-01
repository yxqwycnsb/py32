#ifndef __BSP_UART_H
#define __BSP_UART_H

#include "py32f0xx_hal.h"
#include "stdio.h"


#define		DEBUG_UART_Rx_GPIO_CLK_ENABLE			__HAL_RCC_GPIOB_CLK_ENABLE
#define		DEBUG_UART_Rx_GPIO_PORT						GPIOB
#define		DEBUG_UART_Rx_GPIO_PIN						GPIO_PIN_5

#define		DEBUG_UART_Tx_GPIO_CLK_ENABLE			__HAL_RCC_GPIOB_CLK_ENABLE
#define		DEBUG_UART_Tx_GPIO_PORT						GPIOB
#define		DEBUG_UART_Tx_GPIO_PIN						GPIO_PIN_4

extern	UART_HandleTypeDef	Uart1_Handle;

void DEBUG_USART_Config(uint32_t BaudRate);
void Usart_SendString(uint8_t *str);


#endif /* __BSP_UART_H */

