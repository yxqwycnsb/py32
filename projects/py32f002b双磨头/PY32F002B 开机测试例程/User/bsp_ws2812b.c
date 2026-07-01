/**
  ******************************************************************************
  * @file    bsp_ws2812b.c
  * @brief   WS2812B driver via SPI1 PA7 (HAL library)
  *          Adapted from ws2812_spi.c (LL library)
  *
  *          Encoding: 1 SPI byte = 1 WS2812 data bit @ 3MHz (DIV8)
  *            bit-0:  0x80  (10000000)
  *            bit-1:  0xFC  (11111100)
  *
  *          PY32        WS2812(B)
  *          PA7   ------> DIN
  ******************************************************************************
  */

#include "bsp_ws2812b.h"
#include "py32f0xx_hal.h"

/* -------------------------------------------------------------------------- */
/* SPI Handle (matching LL config: PA7, 3MHz, CPOL=0 CPHA=0, full duplex)   */
/* -------------------------------------------------------------------------- */
static SPI_HandleTypeDef SpiHandle;

/* -------------------------------------------------------------------------- */
/* Global WS2812 SPI buffer (matches ws2812_spi.c pattern) */
/* -------------------------------------------------------------------------- */
uint8_t ws2812_buffer[WS2812_BUFFER_SIZE];

/* -------------------------------------------------------------------------- */
/* SysTick millisecond counter (incremented in SysTick_Handler / main.c) */
/* -------------------------------------------------------------------------- */
volatile uint32_t g_systick_ms = 0u;

/* -------------------------------------------------------------------------- */
/* FILL_BUFFER macro: 1 byte per WS2812 bit (identical to ws2812_spi.c) */
/* -------------------------------------------------------------------------- */
#define WS2812_FILL_BUFFER(COLOR)                            \
    for (uint8_t mask = 0x80u; mask; mask >>= 1u)           \
    {                                                        \
        if ((COLOR) & mask) { *ptr++ = 0xFCu; }             \
        else                { *ptr++ = 0x80u; }             \
    }

/* ======================================================================== */
/*  Init: SPI1 PA7  (matches LLC APP_SPIConfig exactly)                     */
/* ======================================================================== */
void BSP_WS2812B_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    /* Clock enable */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_SPI1_CLK_ENABLE();

    /* PA7: AF0 (SPI1_MOSI), push-pull, high speed, no pull */
    gpio.Pin       = GPIO_PIN_7;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF0_SPI1;
    HAL_GPIO_Init(GPIOA, &gpio);

    /* SPI1: Master, Full-Duplex, 8bit, CPOL=0 CPHA=0, MSB, DIV8 → 3MHz */
    SpiHandle.Instance               = SPI1;
    SpiHandle.Init.Mode              = SPI_MODE_MASTER;
    SpiHandle.Init.Direction         = SPI_DIRECTION_2LINES;         /* full duplex (match LL) */
    SpiHandle.Init.DataSize          = SPI_DATASIZE_8BIT;
    SpiHandle.Init.CLKPolarity       = SPI_POLARITY_LOW;
    SpiHandle.Init.CLKPhase          = SPI_PHASE_1EDGE;
    SpiHandle.Init.NSS               = SPI_NSS_SOFT;
    SpiHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;     /* 24/8 = 3 MHz */
    SpiHandle.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    HAL_SPI_Init(&SpiHandle);

    /* Enable SPI */
    __HAL_SPI_ENABLE(&SpiHandle);

    /* Clear buffer (matching ws2812_init memset) */
    for (uint16_t i = 0u; i < WS2812_BUFFER_SIZE; i++)
    {
        ws2812_buffer[i] = 0x00u;
    }
}

/* ======================================================================== */
/*  Send RGB data + reset pulse via SPI (matches ws2812_send_spi)           */
/* ======================================================================== */
void BSP_WS2812B_Send(const uint8_t *p_rgb, uint8_t led_cnt)
{
    uint8_t *ptr;
    uint8_t  i;

    if (led_cnt > WS2812B_LED_MAX)
        led_cnt = WS2812B_LED_MAX;

    /* Clear entire buffer (all LEDs + reset area) */
    ptr = ws2812_buffer;
    for (i = 0; i < WS2812_BUFFER_SIZE; i++)
    {
        *ptr++ = 0x00u;
    }

    /* Fill buffer with GRB pixels (1 SPI byte per WS2812 data bit) */
    ptr = ws2812_buffer;
    for (i = 0u; i < led_cnt; i++)
    {
        uint8_t r = p_rgb[i * 3u + 0u];
        uint8_t g = p_rgb[i * 3u + 1u];
        uint8_t b = p_rgb[i * 3u + 2u];

        WS2812_FILL_BUFFER(g);      /* WS2812 expects GRB order */
        WS2812_FILL_BUFFER(r);
        WS2812_FILL_BUFFER(b);
    }
    /* ptr now points to unused LEDs + reset area, already cleared to 0x00 */

    /* Send entire buffer (matching ws2812_send_spi byte-by-byte loop) */
    HAL_SPI_Transmit(&SpiHandle, ws2812_buffer, WS2812_BUFFER_SIZE, 100u);
}

/* ======================================================================== */
/*  Millisecond delay (blocking, uses g_systick_ms from SysTick)            */
/* ======================================================================== */
void BSP_WS2812B_DelayMs(uint32_t ms)
{
    uint32_t start = g_systick_ms;
    while ((g_systick_ms - start) < ms)
    {
        /* wait */
    }
}
