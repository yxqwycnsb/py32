#include "./Bsp_Led/Bsp_Led.h"


void Bsp_Led_Init(void)
{
	GPIO_InitTypeDef	GPIO_Led_InitConfig;
	
	/* GPIOA时钟使能 */
	LED2_GPIO_CLK_ENABLE();  
	LED3_GPIO_CLK_ENABLE();  
	LED4_GPIO_CLK_ENABLE();  	
	
	GPIO_Led_InitConfig.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_Led_InitConfig.Pull = GPIO_PULLUP;
	GPIO_Led_InitConfig.Speed = GPIO_SPEED_FREQ_HIGH;
	/* LED2 GPIO初始化 */
	GPIO_Led_InitConfig.Pin = LED2_GPIO_PIN;
	HAL_GPIO_Init(LED2_GPIO_PORT, &GPIO_Led_InitConfig);
	/* LED3 GPIO初始化 */
	GPIO_Led_InitConfig.Pin = LED3_GPIO_PIN;
	HAL_GPIO_Init(LED3_GPIO_PORT, &GPIO_Led_InitConfig);
	/* LED4 GPIO初始化 */
	GPIO_Led_InitConfig.Pin = LED4_GPIO_PIN;
	HAL_GPIO_Init(LED4_GPIO_PORT, &GPIO_Led_InitConfig);
	
	/* LED 关闭 */
	LED2(LED_OFF);
	LED3(LED_OFF);
	LED4(LED_OFF);
	
}



