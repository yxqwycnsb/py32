/**
  ******************************************************************************
  * @file    bsp_ws2812b.h
  * @brief   WS2812B driver via SPI1 PA7 (adapted from ws2812_spi.c)
  *
  *          PY32        WS2812(B)
  *          PA7   ------> DIN
  *
  *          Encoding: 1 SPI byte = 1 WS2812 data bit @ 3MHz (DIV8)
  *            bit-0:  0x80  (10000000)
  *            bit-1:  0xFC  (11111100)
  *          Reset: 60 bytes of 0x00
  ******************************************************************************
  */

#ifndef __BSP_WS2812B_H
#define __BSP_WS2812B_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* User-configurable -------------------------------------------------------- */
#define WS2812B_LED_MAX     8u

/* Timing constants (matching ws2812_spi.c) --------------------------------- */
#define WS2812_RESET_PULSE  60u
#define WS2812_BUFFER_SIZE  (WS2812B_LED_MAX * 24u + WS2812_RESET_PULSE)

/* Exported globals --------------------------------------------------------- */
extern volatile uint32_t g_systick_ms;
extern uint8_t ws2812_buffer[];

/* API ---------------------------------------------------------------------- */

void BSP_WS2812B_Init(void);
void BSP_WS2812B_Send(const uint8_t *p_rgb, uint8_t led_cnt);
void BSP_WS2812B_DelayMs(uint32_t ms);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_WS2812B_H */
