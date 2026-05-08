/* ad7606_config.h — AD7606 + W5500 hardware configuration
 *
 * ═══════════════════════════════════════════════════════════════
 * TARGET MCU : STM32F401RCT6  (64-pin)
 * SYSCLK     : 64 MHz  (HSE=25 MHz, PLLM=25, PLLN=384, PLLP=6)
 * APB1       : 32 MHz  (HCLK/2)  — SPI2, SPI3, TIM2
 * APB2       : 64 MHz  (HCLK/1)  — SPI1 (ETH)
 * ═══════════════════════════════════════════════════════════════
 *
 * ── Sample rate (48 kHz) ───────────────────────────────────────
 *  TIM2_CLK = 64 MHz  (APB1=32 MHz, APB1x2 rule → 64 MHz)
 *  ARR = 64,000,000 / 48,000 − 1 = 1332  → 48 kHz exactly
 *
 * ── SPI rates ────────────────────────────────────────────────────
 *  SPI2/SPI3 (APB1=32 MHz) prescaler /2 → 16 MHz  (AD7606 max 23 MHz ✓)
 *  SPI1      (APB2=64 MHz) prescaler /2 → 32 MHz  (W5500  max 80 MHz ✓)
 *
 * ── Pin assignment ───────────────────────────────────────────────
 *  PA4   ADC_CS       Active LOW, SW NSS
 *  PA5   SPI1_SCK     ETH clock (AF5)         ← PA5 reserved for ETH
 *  PA6   SPI1_MISO    ETH data  (AF5)         ← PA6 reserved for ETH
 *  PA7   SPI1_MOSI    ETH data  (AF5)
 *  PA8   ETH_CS       W5500 chip-select (active LOW)
 *  PB0   CONVST       Pulse LOW ≥25 ns; idle HIGH
 *  PB1   BUSY         Input; falling edge EXTI1; HIGH during conversion
 *  PB2   RESET        Active HIGH; pulse ≥100 ns; 5 ms settle after
 *  PB5   ETH_RST      W5500 reset (active LOW)
 *  PB10  HOLD         Active LOW (freezes X and Y banks simultaneously)
 *  PB15  CH_SEL       LOW=X bank (default), HIGH=Y bank
 *  PB13  SPI2_SCK     AF5
 *  PB14  SPI2_MISO    AF5
 *  PB3   SPI3_SCK     AF6
 *  PB4   SPI3_MISO    AF6
 *  PC13  LED          Active LOW
 *
 * NOTE: HOLD and CH_SEL moved from PA5/PA6 to PB10/PB15 to free
 *       PA5 (SPI1_SCK) and PA6 (SPI1_MISO) for the ETH module.
 */

#ifndef AD7606_CONFIG_H
#define AD7606_CONFIG_H

/* ── Sample rate ─────────────────────────────────────────────── */
#define AD7606_SAMPLE_RATE_HZ     48000u
#define AD7606_TIM2_CLK_HZ      64000000u  /* APB1(32MHz) x2 = 64 MHz */
#define AD7606_TIM_ARR    ((AD7606_TIM2_CLK_HZ / AD7606_SAMPLE_RATE_HZ) - 1u)  /* 1332 @ 48 kHz */

/* ── Voltage reference ───────────────────────────────────────── */
#define AD7606_VREF_MV            5000
#define AD7606_FULLSCALE_CODE    32767

/* ── Channel / batch config ──────────────────────────────────── */
#define AD7606_NUM_CHANNELS           16u
#define AD7606_CHANNELS_PER_SPI_BURST  4u
#define AD7606_ROUNDS                  2u
#define AD7606_BATCH_FRAMES            24u

/* ── Frame / batch sizes ─────────────────────────────────────── */
#define AD7606_SYNC_BYTES            4u
#define AD7606_FRAME_DATA_BYTES      (AD7606_NUM_CHANNELS * 2u)                     /* 32 */
#define AD7606_FRAME_SIZE            (AD7606_SYNC_BYTES + AD7606_FRAME_DATA_BYTES)  /* 36 */
#define AD7606_BATCH_SIZE            (AD7606_BATCH_FRAMES * AD7606_FRAME_SIZE)      /* 864 */

/* ── ADC control pins ────────────────────────────────────────── */
#define ADC_CS_PORT               GPIOA
#define ADC_CS_PIN                GPIO_PIN_4

/* HOLD and CH_SEL moved to PB10/PB15 — PA5/PA6 are used by SPI1 (ETH) */
#define ADC_HOLD_PORT             GPIOB
#define ADC_HOLD_PIN              GPIO_PIN_10

#define ADC_CHSEL_PORT            GPIOB
#define ADC_CHSEL_PIN             GPIO_PIN_15

#define ADC_CONVST_PORT           GPIOB
#define ADC_CONVST_PIN            GPIO_PIN_0

#define ADC_BUSY_PORT             GPIOB
#define ADC_BUSY_PIN              GPIO_PIN_1

#define ADC_RESET_PORT            GPIOB
#define ADC_RESET_PIN             GPIO_PIN_2

#define ADC_LED_PORT              GPIOC
#define ADC_LED_PIN               GPIO_PIN_13

/* ── ADC pin control macros ──────────────────────────────────── */
#define AD7606_HOLD_ASSERT()   HAL_GPIO_WritePin(ADC_HOLD_PORT,  ADC_HOLD_PIN,  GPIO_PIN_RESET)
#define AD7606_HOLD_RELEASE()  HAL_GPIO_WritePin(ADC_HOLD_PORT,  ADC_HOLD_PIN,  GPIO_PIN_SET)
#define AD7606_CHSEL_X()       HAL_GPIO_WritePin(ADC_CHSEL_PORT, ADC_CHSEL_PIN, GPIO_PIN_RESET)
#define AD7606_CHSEL_Y()       HAL_GPIO_WritePin(ADC_CHSEL_PORT, ADC_CHSEL_PIN, GPIO_PIN_SET)

/* ── CONVST pulse: 4 NOPs @ 64 MHz = 62.5 ns ≥ 25 ns min ───── */
#define ADC_CONVST_NOP_COUNT      4u

/* ── SPI prescalers ──────────────────────────────────────────── */
/* SPI2/SPI3: APB1=32 MHz, /2 → 16 MHz (AD7606 max 23 MHz ✓) */
#define ADC_SPI_PRESCALER         SPI_BAUDRATEPRESCALER_2

/* SPI1 (ETH): APB2=64 MHz, /2 → 32 MHz (W5500 max 80 MHz ✓) */
#define ETH_SPI_PRESCALER         SPI_BAUDRATEPRESCALER_2

/* ── ADC SPI pins ────────────────────────────────────────────── */
#define ADC_SPI2_SCK_PORT         GPIOB
#define ADC_SPI2_SCK_PIN          GPIO_PIN_13   /* AF5 */
#define ADC_SPI2_MISO_PORT        GPIOB
#define ADC_SPI2_MISO_PIN         GPIO_PIN_14   /* AF5 */

#define ADC_SPI3_SCK_PORT         GPIOB
#define ADC_SPI3_SCK_PIN          GPIO_PIN_3    /* AF6 */
#define ADC_SPI3_MISO_PORT        GPIOB
#define ADC_SPI3_MISO_PIN         GPIO_PIN_4    /* AF6 */

/* ── ADC DMA streams (STM32F401 RM0368 Table 20) ─────────────── */
#define ADC_DMA_SPI2_STREAM       DMA1_Stream3
#define ADC_DMA_SPI2_CHANNEL      DMA_CHANNEL_0
#define ADC_DMA_SPI2_IRQn         DMA1_Stream3_IRQn

#define ADC_DMA_SPI3_STREAM       DMA1_Stream0
#define ADC_DMA_SPI3_CHANNEL      DMA_CHANNEL_0
#define ADC_DMA_SPI3_IRQn         DMA1_Stream0_IRQn

/* ── ETH DMA stream (SPI1 TX, DMA2) ─────────────────────────── */
#define ETH_DMA_STREAM            DMA2_Stream3
#define ETH_DMA_CHANNEL           DMA_CHANNEL_3
#define ETH_DMA_IRQn              DMA2_Stream3_IRQn

/* ── NVIC priorities ─────────────────────────────────────────── */
/* ADC DMA(0) > ADC EXTI1(1) > ADC TIM2(2) > ETH DMA(5)
 * ETH DMA lower than ADC — ETH TX fires after batch is ready,
 * never races with the 10 µs ADC acquisition window. */
#define ADC_DMA_NVIC_PRIORITY     1u
#define ADC_EXTI1_NVIC_PRIORITY   2u
#define ADC_TIM2_NVIC_PRIORITY    3u
#define ETH_DMA_NVIC_PRIORITY     6u

#endif /* AD7606_CONFIG_H */
