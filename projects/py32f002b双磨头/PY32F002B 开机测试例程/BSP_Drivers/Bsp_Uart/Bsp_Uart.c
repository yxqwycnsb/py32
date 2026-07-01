#include "Bsp_Uart/Bsp_Uart.h"



UART_HandleTypeDef	Uart1_Handle;



/**
 * @brief       ����X��ʼ������
 * @param       baudrate: ������, �����Լ���Ҫ���ò�����ֵ
 * @note        ע��: ����������ȷ��ʱ��Դ, ���򴮿ڲ����ʾͻ������쳣.
 * @retval      ��
 */
void DEBUG_USART_Config(uint32_t BaudRate)
{
	
	Uart1_Handle.Instance = USART1;
	Uart1_Handle.Init.BaudRate = BaudRate;
	Uart1_Handle.Init.WordLength = UART_WORDLENGTH_8B;
	Uart1_Handle.Init.StopBits = UART_STOPBITS_1;
	Uart1_Handle.Init.Parity = UART_PARITY_NONE;
	Uart1_Handle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	Uart1_Handle.Init.Mode = UART_MODE_TX_RX;
	/*ʹ�ܴ��� */
	HAL_UART_Init(&Uart1_Handle);
	/*ʹ�ܴ��ڽ��ն� */
	__HAL_UART_ENABLE_IT(&Uart1_Handle,UART_IT_RXNE); 
}

/**
 * @brief       UART�ײ��ʼ������
 * @param       huart: UART�������ָ��
 * @note        �˺����ᱻHAL_UART_Init()����
 *              ���ʱ��ʹ�ܣ��������ã��ж�����z
 * @retval      ��
 */	 
void HAL_UART_MspInit(UART_HandleTypeDef *UartHandle)
{
	
	if(UartHandle->Instance == USART1)  
	{
		GPIO_InitTypeDef	Uart_GPIO_InitConfig;
	
		/*USART1ʱ��ʹ��*/
		DEBUG_UART_Rx_GPIO_CLK_ENABLE();
    DEBUG_UART_Tx_GPIO_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();
		
		
		Uart_GPIO_InitConfig.Pin       = DEBUG_UART_Tx_GPIO_PIN;
    Uart_GPIO_InitConfig.Mode      = GPIO_MODE_AF_PP;
    Uart_GPIO_InitConfig.Pull      = GPIO_PULLUP;
    Uart_GPIO_InitConfig.Speed     = GPIO_SPEED_FREQ_HIGH;
    Uart_GPIO_InitConfig.Alternate = GPIO_AF1_USART1;
    HAL_GPIO_Init(DEBUG_UART_Tx_GPIO_PORT, &Uart_GPIO_InitConfig);

    Uart_GPIO_InitConfig.Pin 			 = DEBUG_UART_Rx_GPIO_PIN;
    Uart_GPIO_InitConfig.Mode      = GPIO_MODE_AF_PP;
    Uart_GPIO_InitConfig.Pull      = GPIO_PULLUP;
    Uart_GPIO_InitConfig.Speed     = GPIO_SPEED_FREQ_HIGH;
    Uart_GPIO_InitConfig.Alternate = GPIO_AF1_USART1;
    HAL_GPIO_Init(DEBUG_UART_Rx_GPIO_PORT, &Uart_GPIO_InitConfig);
		
		
		HAL_NVIC_SetPriority(USART1_IRQn, 0, 1);              /* ��ռ���ȼ�0�������ȼ�1 */
		HAL_NVIC_EnableIRQ(USART1_IRQn);                      /* ʹ��USART1�ж�ͨ�� */	
	}
}


/* USART1_IRQHandler is implemented in User/main.c */

void Usart_SendString(uint8_t *str)
{
	unsigned int k=0;
  do 
  {
    HAL_UART_Transmit(&Uart1_Handle,(uint8_t *)(str + k) ,1,1000);
    k++;
  } while(*(str + k)!='\0');
  
}


///* �������´���, ֧��printf���� */
#if (defined (__CC_ARM)) || (defined (__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050))
/**
  * @brief  writes a character to the usart
  * @param  ch
  *         *f
  * @retval the character
  */
int fputc(int ch, FILE *f)
{
  /* Send a byte to USART */
  HAL_UART_Transmit(&Uart1_Handle, (uint8_t *)&ch, 1, 1000);

  return (ch);
}

/**
  * @brief  get a character from the usart
  * @param  *f
  * @retval a character
  */
int fgetc(FILE *f)
{
  int ch;
  HAL_UART_Receive(&Uart1_Handle, (uint8_t *)&ch, 1, 1000);
  return (ch);
}

#elif defined(__ICCARM__)
/**
  * @brief  writes a character to the usart
  * @param  ch
  *         *f
  * @retval the character
  */
int putchar(int ch)
{
  /* Send a byte to USART */
  HAL_UART_Transmit(&DebugUartHandle, (uint8_t *)&ch, 1, 1000);

  return (ch);
}
#elif  defined(__GNUC__)
/**
  * @brief  writes a character to the usart
  * @param  ch
  * @retval the character
  */
int __io_putchar(int ch)
{
  /* Send a byte to USART */
  HAL_UART_Transmit(&DebugUartHandle, (uint8_t *)&ch, 1, 1000);

  return ch;
}

int _write(int file, char *ptr, int len)
{
  int DataIdx;
  for (DataIdx=0;DataIdx<len;DataIdx++)
  {
    __io_putchar(*ptr++);
  }
  return len;
}
#endif



