/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file         stm32f4xx_hal_msp.c
  * @brief        MSP Initialization / De-Initialization
  *
  * FIX 2 applied in this file:
  *   • DMA1_Stream3 IRQ (SPI2 RX) — HAL_NVIC_EnableIRQ added.
  *   • DMA1_Stream0 IRQ (SPI3 RX) — HAL_NVIC_EnableIRQ added.
  *   • TIM2 update   IRQ          — HAL_NVIC_SetPriority + EnableIRQ added.
  *
  * The EXTI1 NVIC enable is intentionally placed in MX_GPIO_Init (main.c)
  * so it is armed only AFTER the BUSY pin pull-down has been applied.
  * Enabling it here (before GPIO init) would risk a spurious interrupt
  * if BUSY floats HIGH during power-up.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"

extern DMA_HandleTypeDef hdma_spi1_tx;
extern DMA_HandleTypeDef hdma_spi2_rx;
extern DMA_HandleTypeDef hdma_spi3_rx;

/* ── HAL_MspInit ─────────────────────────────────────────────────────────── */
void HAL_MspInit(void)
{
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();
}

/* ── HAL_SPI_MspInit ─────────────────────────────────────────────────────── */
void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* ── SPI1 — W5500 Ethernet TX DMA ───────────────────────── */
    if (hspi->Instance == SPI1)
    {
        __HAL_RCC_SPI1_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();

        /* PA5=SCK, PA6=MISO, PA7=MOSI */
        GPIO_InitStruct.Pin       = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        /* SPI1 TX → DMA2 Stream3 Ch3 */
        hdma_spi1_tx.Instance                 = DMA2_Stream3;
        hdma_spi1_tx.Init.Channel             = DMA_CHANNEL_3;
        hdma_spi1_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
        hdma_spi1_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
        hdma_spi1_tx.Init.MemInc              = DMA_MINC_ENABLE;
        hdma_spi1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_spi1_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
        hdma_spi1_tx.Init.Mode                = DMA_NORMAL;
        hdma_spi1_tx.Init.Priority            = DMA_PRIORITY_VERY_HIGH;
        hdma_spi1_tx.Init.FIFOMode            = DMA_FIFOMODE_ENABLE;
        hdma_spi1_tx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
        hdma_spi1_tx.Init.MemBurst            = DMA_MBURST_INC4;
        hdma_spi1_tx.Init.PeriphBurst         = DMA_PBURST_SINGLE;
        if (HAL_DMA_Init(&hdma_spi1_tx) != HAL_OK) Error_Handler();

        __HAL_LINKDMA(hspi, hdmatx, hdma_spi1_tx);
        /* NOTE: DMA2_Stream3 IRQ NVIC is enabled in MX_DMA_Init (main.c) */
    }

    /* ── SPI2 — AD7606 DOUTA RX DMA ─────────────────────────── */
    else if (hspi->Instance == SPI2)
    {
        __HAL_RCC_SPI2_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();

        /* PB13=SCK, PB14=MISO (1-wire RX-only, no MOSI) */
        GPIO_InitStruct.Pin       = GPIO_PIN_13 | GPIO_PIN_14;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        /* SPI2 RX → DMA1 Stream3 Ch0 */
        hdma_spi2_rx.Instance                 = DMA1_Stream3;
        hdma_spi2_rx.Init.Channel             = DMA_CHANNEL_0;
        hdma_spi2_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
        hdma_spi2_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
        hdma_spi2_rx.Init.MemInc              = DMA_MINC_ENABLE;
        hdma_spi2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;      /* 8-bit SPI */
        hdma_spi2_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
        hdma_spi2_rx.Init.Mode                = DMA_NORMAL;
        hdma_spi2_rx.Init.Priority            = DMA_PRIORITY_VERY_HIGH;
        hdma_spi2_rx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;     /* direct mode for RX */
        if (HAL_DMA_Init(&hdma_spi2_rx) != HAL_OK) Error_Handler();

        __HAL_LINKDMA(hspi, hdmarx, hdma_spi2_rx);

        /*
         * FIX 2 — DMA1_Stream3 IRQ was configured but NEVER enabled.
         * Without this, HAL_SPI_Receive_DMA() launches the DMA transfer
         * but the TC (Transfer Complete) interrupt never fires, so
         * HAL_SPI_RxCpltCallback is never called → deadlock / HardFault.
         *
         * Priority 0: highest in the system, must pre-empt TIM2 (2) and
         * EXTI1 (1) so the CS deassert in AD7606_OnSPIComplete happens
         * as quickly as possible after the last byte is clocked out.
         */
        HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, ADC_DMA_NVIC_PRIORITY, 0);
        HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);   /* FIX 2: was missing */
    }

    /* ── SPI3 — AD7606 DOUTB RX DMA ─────────────────────────── */
    else if (hspi->Instance == SPI3)
    {
        __HAL_RCC_SPI3_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();

        /* PB3=SCK, PB4=MISO (1-wire RX-only, no MOSI) */
        GPIO_InitStruct.Pin       = GPIO_PIN_3 | GPIO_PIN_4;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        /* SPI3 RX → DMA1 Stream0 Ch0 */
        hdma_spi3_rx.Instance                 = DMA1_Stream0;
        hdma_spi3_rx.Init.Channel             = DMA_CHANNEL_0;
        hdma_spi3_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
        hdma_spi3_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
        hdma_spi3_rx.Init.MemInc              = DMA_MINC_ENABLE;
        hdma_spi3_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;      /* 8-bit SPI */
        hdma_spi3_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
        hdma_spi3_rx.Init.Mode                = DMA_NORMAL;
        hdma_spi3_rx.Init.Priority            = DMA_PRIORITY_VERY_HIGH;
        hdma_spi3_rx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;     /* direct mode for RX */
        if (HAL_DMA_Init(&hdma_spi3_rx) != HAL_OK) Error_Handler();

        __HAL_LINKDMA(hspi, hdmarx, hdma_spi3_rx);

        /*
         * FIX 2 — DMA1_Stream0 IRQ was configured but NEVER enabled.
         * Same consequence as SPI2: TC interrupt silent, callback never
         * fires, acquisition state machine stalls after Round1.
         */
        HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, ADC_DMA_NVIC_PRIORITY, 0);
        HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);   /* FIX 2: was missing */
    }
}

/* ── HAL_SPI_MspDeInit ───────────────────────────────────────────────────── */
void HAL_SPI_MspDeInit(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI1)
    {
        __HAL_RCC_SPI1_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);
        HAL_DMA_DeInit(hspi->hdmatx);
    }
    else if (hspi->Instance == SPI2)
    {
        __HAL_RCC_SPI2_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_13 | GPIO_PIN_14);
        HAL_DMA_DeInit(hspi->hdmarx);
        HAL_NVIC_DisableIRQ(DMA1_Stream3_IRQn);
    }
    else if (hspi->Instance == SPI3)
    {
        __HAL_RCC_SPI3_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_3 | GPIO_PIN_4);
        HAL_DMA_DeInit(hspi->hdmarx);
        HAL_NVIC_DisableIRQ(DMA1_Stream0_IRQn);
    }
}

/* ── HAL_TIM_Base_MspInit ────────────────────────────────────────────────── */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim_base)
{
    if (htim_base->Instance == TIM2)
    {
        __HAL_RCC_TIM2_CLK_ENABLE();

        /*
         * FIX 2 — TIM2 update IRQ was never enabled.
         * Without this, HAL_TIM_Base_Start_IT() arms the TIM2 UIE bit
         * but the NVIC never routes the interrupt to TIM2_IRQHandler,
         * so AD7606_TriggerConversion() is never called and acquisition
         * never starts.
         *
         * Priority 2: lower than DMA (0) and EXTI1 (1) so a DMA
         * completion can always pre-empt a TIM2 tick.
         */
        HAL_NVIC_SetPriority(TIM2_IRQn, ADC_TIM2_NVIC_PRIORITY, 0);  /* priority 2 */
        HAL_NVIC_EnableIRQ(TIM2_IRQn);   /* FIX 2: was missing */
    }
}

/* ── HAL_TIM_Base_MspDeInit ──────────────────────────────────────────────── */
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef *htim_base)
{
    if (htim_base->Instance == TIM2)
    {
        __HAL_RCC_TIM2_CLK_DISABLE();
        HAL_NVIC_DisableIRQ(TIM2_IRQn);
    }
}
