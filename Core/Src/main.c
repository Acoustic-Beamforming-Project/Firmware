/* USER CODE BEGIN Header */
/**
 * main.c — AD7606 16-ch ADC acquisition + W5500 Ethernet DMA TX
 *
 * STM32F401RCT6 @ 64 MHz (HSE=25 MHz, PLLM=25, PLLN=128, PLLP=/2)
 *
 * SPI1 (ETH)      : APB2=64 MHz, prescaler /2 → 32 MHz
 * SPI2/SPI3 (ADC) : APB1=32 MHz, prescaler /2 → 16 MHz
 * TIM2            : APB1x2=64 MHz, ARR=1332 → 48 kHz acquisition trigger
 *
 * FIX 1 — All ADC GPIO pins initialised here (CS, CONVST, RESET, HOLD,
 *          CH_SEL as outputs; BUSY as input with pull-down to prevent a
 *          floating-pin interrupt storm that starves SysTick).
 * FIX 2 — MX_SPI2_Init, MX_SPI3_Init and MX_TIM2_Init are now present
 *          and called from main() so the ADC driver can operate.
 */
/* USER CODE END Header */

#include "main.h"

/* USER CODE BEGIN Includes */
#include "wiz5500.h"
#include "ad7606.h"
#include <string.h>
/* USER CODE END Includes */

/* USER CODE BEGIN PD */
#define BATCH_FRAMES   AD7606_BATCH_FRAMES   /* 24  */
#define CHANNELS       AD7606_NUM_CHANNELS   /* 16  */
#define FRAME_SIZE     AD7606_FRAME_SIZE     /* 36  */
#define BATCH_SIZE     AD7606_BATCH_SIZE     /* 864 */
/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;

/* ADC peripheral handles — fully initialised by MX_SPI2/3_Init below */
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;
TIM_HandleTypeDef htim2;
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_spi3_rx;

/* USER CODE BEGIN PV */
static AD7606_Batch rx_batch;
static volatile uint32_t sent_ok    = 0u;
static volatile uint32_t eth_drops  = 0u;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);   /* FIX 2: was missing */
static void MX_SPI3_Init(void);   /* FIX 2: was missing */
static void MX_TIM2_Init(void);   /* FIX 2: was missing */

/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

int main(void)
{
    HAL_Init();
    /* HAL SPI timeout helpers use HAL_GetTick(). We launch ADC SPI DMA from
     * EXTI/TIM paths, so SysTick must preempt those ISRs; otherwise timeout
     * loops in HAL can stall waiting for a tick that never advances. */
    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
    SystemClock_Config();

    /* FIX 1: GPIO must come first so all output pins are in a safe
     * idle state (CS high, CONVST high, BUSY pull-down applied) before
     * any peripheral is started.  The BUSY pull-down prevents the
     * floating-pin EXTI storm that starved SysTick. */
    MX_GPIO_Init();

    /* FIX 2: DMA clocks and NVIC must be set up before the SPI inits
     * that link DMA handles, and before the ADC driver is called. */
    MX_DMA_Init();

    MX_SPI1_Init();
    MX_SPI2_Init();   /* FIX 2 */
    MX_SPI3_Init();   /* FIX 2 */
    MX_TIM2_Init();   /* FIX 2 */

    /* USER CODE BEGIN 2 */
    /* ── Ethernet ────────────────────────────────────────────── */
    WIZ5500_Config eth_cfg = {
        .mac       = {0x00, 0x08, 0xDC, 0x11, 0x22, 0x33},
        .ip        = {192, 168, 1, 20},
        .subnet    = {255, 255, 255, 0},
        .gateway   = {192, 168, 1, 1},
        .dest_ip   = {192, 168, 1, 255},
        .dest_port = 5002u,
        .src_port  = 5001u,
    };
    WIZ5500_Init(&eth_cfg);

    if (WIZCHIP_READ(VERSIONR) != 0x04u)
        Error_Handler();

    if (WIZ5500_SetupSocket0() != WIZ5500_OK)
        Error_Handler();

    /* ── ADC ─────────────────────────────────────────────────── */
    AD7606_Init(&hspi2, &hspi3, &htim2);
    AD7606_Start();
    /* USER CODE END 2 */

    while (1)
    {
        if (AD7606_IsBatchReady())
        {
            AD7606_GetBatch(&rx_batch);

            WIZ5500_Status s;
            do {
                s = WIZ5500_SendBatch(rx_batch.packet, BATCH_SIZE);
            } while (s == WIZ5500_ERR_BUSY);

            if (s == WIZ5500_OK)
                sent_ok++;
            else
                eth_drops++;
        }

        /* LED heartbeat — toggle every 4096 frames */
        if ((AD7606_GetFrameCount() & 0xFFFu) == 0u)
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    }
}

/* ── System clock ────────────────────────────────────────────────────────── */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM       = 25;
    RCC_OscInitStruct.PLL.PLLN       = 128;
    RCC_OscInitStruct.PLL.PLLP       = RCC_PLLP_DIV2;  /* SYSCLK = 64 MHz */
    RCC_OscInitStruct.PLL.PLLQ       = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                      | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;   /* HCLK  = 64 MHz */
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;     /* APB1  = 32 MHz */
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;     /* APB2  = 64 MHz */
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
        Error_Handler();
}

/* ── SPI1 — W5500 Ethernet @ 32 MHz ─────────────────────────────────────── */
static void MX_SPI1_Init(void)
{
    hspi1.Instance               = SPI1;
    hspi1.Init.Mode              = SPI_MODE_MASTER;
    hspi1.Init.Direction         = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize          = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity       = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase          = SPI_PHASE_1EDGE;
    hspi1.Init.NSS               = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;  /* APB2/2 = 32 MHz */
    hspi1.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode            = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial     = 10;
    if (HAL_SPI_Init(&hspi1) != HAL_OK) Error_Handler();
}

/* ── SPI2 — AD7606 DOUTA (X bank, CH1-CH8) @ 16 MHz ────────────────────── */
static void MX_SPI2_Init(void)
{
    hspi2.Instance               = SPI2;
    hspi2.Init.Mode              = SPI_MODE_MASTER;
    hspi2.Init.Direction         = SPI_DIRECTION_2LINES_RXONLY;
    hspi2.Init.DataSize          = SPI_DATASIZE_8BIT;    /* Changed to 8-bit for direct DMA */
    hspi2.Init.CLKPolarity       = SPI_POLARITY_LOW;
    hspi2.Init.CLKPhase          = SPI_PHASE_1EDGE;
    hspi2.Init.NSS               = SPI_NSS_SOFT;
    hspi2.Init.BaudRatePrescaler = ADC_SPI_PRESCALER;
    hspi2.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    hspi2.Init.TIMode            = SPI_TIMODE_DISABLE;
    hspi2.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    hspi2.Init.CRCPolynomial     = 10;
    if (HAL_SPI_Init(&hspi2) != HAL_OK) Error_Handler();
}

/* ── SPI3 — AD7606 DOUTB (Y bank, CH9-CH16) @ 16 MHz ───────────────────── */
static void MX_SPI3_Init(void)
{
    hspi3.Instance               = SPI3;
    hspi3.Init.Mode              = SPI_MODE_MASTER;
    hspi3.Init.Direction         = SPI_DIRECTION_2LINES_RXONLY;
    hspi3.Init.DataSize          = SPI_DATASIZE_8BIT;    /* Changed to 8-bit for direct DMA */
    hspi3.Init.CLKPolarity       = SPI_POLARITY_LOW;
    hspi3.Init.CLKPhase          = SPI_PHASE_1EDGE;
    hspi3.Init.NSS               = SPI_NSS_SOFT;
    hspi3.Init.BaudRatePrescaler = ADC_SPI_PRESCALER;
    hspi3.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    hspi3.Init.TIMode            = SPI_TIMODE_DISABLE;
    hspi3.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    hspi3.Init.CRCPolynomial     = 10;
    if (HAL_SPI_Init(&hspi3) != HAL_OK) Error_Handler();
}

/* ── TIM2 — 48 kHz ADC acquisition trigger ──────────────────────────────── */
/* FIX 2: This function was entirely absent from the original main.c.
 * TIM2_CLK = APB1(32MHz) x2 = 64 MHz (APB1 prescaler != 1 → timer x2 rule).
 * ARR = 64,000,000 / 48,000 - 1 = 1332 → exactly 48 kHz.
 * HAL_TIM_Base_MspInit (hal_msp.c) enables TIM2 clock and NVIC. */
static void MX_TIM2_Init(void)
{
    TIM_ClockConfigTypeDef  sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig      = {0};

    htim2.Instance               = TIM2;
    htim2.Init.Prescaler         = 0;                  /* No prescaler — full 64 MHz */
    htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim2.Init.Period            = AD7606_TIM_ARR;     /* 1332 → 48 kHz */
    htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK) Error_Handler();

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
        Error_Handler();

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
        Error_Handler();
}

/* ── DMA controller clocks + NVIC ───────────────────────────────────────── */
/* FIX 2: DMA1 clock added for SPI2/SPI3 DMA streams.
 * Only the ETH DMA IRQ was previously enabled; the ADC DMA IRQs were absent. */
static void MX_DMA_Init(void)
{
    /* DMA1: SPI2_RX (Stream3) and SPI3_RX (Stream0) */
    __HAL_RCC_DMA1_CLK_ENABLE();

    /* DMA2: SPI1_TX (Stream3) — Ethernet */
    __HAL_RCC_DMA2_CLK_ENABLE();

    /* FIX 2: ADC DMA IRQs were never enabled — HardFault on first DMA TC */
    HAL_NVIC_SetPriority(ADC_DMA_SPI2_IRQn, ADC_DMA_NVIC_PRIORITY, 0);   /* priority 0 */
    HAL_NVIC_EnableIRQ(ADC_DMA_SPI2_IRQn);   /* DMA1_Stream3 — SPI2 RX */

    HAL_NVIC_SetPriority(ADC_DMA_SPI3_IRQn, ADC_DMA_NVIC_PRIORITY, 0);   /* priority 0 */
    HAL_NVIC_EnableIRQ(ADC_DMA_SPI3_IRQn);   /* DMA1_Stream0 — SPI3 RX */

    /* Ethernet DMA — lower priority than ADC */
    HAL_NVIC_SetPriority(ETH_DMA_IRQn, ETH_DMA_NVIC_PRIORITY, 0);        /* priority 5 */
    HAL_NVIC_EnableIRQ(ETH_DMA_IRQn);        /* DMA2_Stream3 — SPI1 TX */
}

/* ── GPIO ────────────────────────────────────────────────────────────────── */
/*
 * FIX 1 — Complete GPIO initialisation for every ADC control pin.
 *
 * Root cause of the HAL_GetTick() freeze:
 *   PB1 (BUSY) was left uninitialised → floating → EXTI1 fired
 *   continuously at priority 1, starving SysTick (priority 15).
 *
 * Fix applied:
 *   • BUSY (PB1)   — INPUT with GPIO_PULLDOWN so the line rests LOW
 *                    when the AD7606 is idle.  Prevents spurious EXTI1
 *                    triggers before the ADC is powered/reset.
 *   • CONVST (PB0) — OUTPUT_PP, pre-set HIGH (idle state).
 *   • RESET (PB2)  — OUTPUT_PP, pre-set LOW  (deasserted).
 *   • HOLD  (PB10) — OUTPUT_PP, pre-set HIGH (released/inactive).
 *   • CH_SEL(PB15) — OUTPUT_PP, pre-set LOW  (X bank default).
 *   • ADC_CS(PA4)  — OUTPUT_PP, pre-set HIGH (deasserted).
 *
 * NOTE: The EXTI1 interrupt itself (NVIC) is enabled in
 *       stm32f4xx_hal_msp.c alongside the SPI2 init, after BUSY has
 *       been given a defined pull-down state here.
 */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Enable all port clocks used */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();

    /* ── Pre-set outputs to safe idle levels BEFORE init ──────── */
    /* ADC CS — deasserted (HIGH) */
    HAL_GPIO_WritePin(ADC_CS_PORT,    ADC_CS_PIN,    GPIO_PIN_SET);
    /* CONVST — idle HIGH; falling edge triggers conversion */
    HAL_GPIO_WritePin(ADC_CONVST_PORT, ADC_CONVST_PIN, GPIO_PIN_SET);
    /* RESET — deasserted (LOW active-HIGH device, so keep LOW) */
    HAL_GPIO_WritePin(ADC_RESET_PORT, ADC_RESET_PIN, GPIO_PIN_RESET);
    /* HOLD — released (HIGH = tracking) */
    HAL_GPIO_WritePin(ADC_HOLD_PORT,  ADC_HOLD_PIN,  GPIO_PIN_SET);
    /* CH_SEL — X bank default (LOW) */
    HAL_GPIO_WritePin(ADC_CHSEL_PORT, ADC_CHSEL_PIN, GPIO_PIN_RESET);
    /* ETH CS — deasserted */
    HAL_GPIO_WritePin(ETH_CS_PORT,  ETH_CS_PIN,   GPIO_PIN_SET);
    HAL_GPIO_WritePin(ETH_RST_PORT, ETH_RST_PIN,  GPIO_PIN_SET);
    /* LED — off (active LOW) */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

    /* ── GPIOA outputs ────────────────────────────────────────── */
    /* PA4  — ADC_CS */
    /* PA8  — ETH_CS */
    GPIO_InitStruct.Pin   = ADC_CS_PIN | ETH_CS_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* ── GPIOB outputs ────────────────────────────────────────── */
    /* PB0  — CONVST  (pulse LOW ≥25 ns, idle HIGH)              */
    /* PB2  — RESET   (active HIGH, ≥100 ns pulse)               */
    /* PB5  — ETH_RST (active LOW)                               */
    /* PB10 — HOLD    (active LOW, freezes T&H)                  */
    /* PB15 — CH_SEL  (LOW=X, HIGH=Y)                            */
    GPIO_InitStruct.Pin   = ADC_CONVST_PIN | ADC_RESET_PIN
                          | ETH_RST_PIN
                          | ADC_HOLD_PIN | ADC_CHSEL_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* ── GPIOB input — BUSY (PB1) ────────────────────────────── */
    /*
     * FIX 1 — CRITICAL: pull-down applied.
     *
     * The AD7606 BUSY pin is open-drain and is driven HIGH during
     * conversion.  When the ADC is idle (or not yet reset) it floats.
     * Without a pull-down the floating PB1 caused continuous EXTI1
     * interrupts (priority 1) that prevented SysTick (priority 15)
     * from ever running, freezing HAL_GetTick() / HAL_Delay().
     *
     * GPIO_PULLDOWN ensures PB1 reads LOW when BUSY is not asserted,
     * which is the correct idle/default state for the EXTI1 edge
     * detector configured for falling-edge only.
     */
    GPIO_InitStruct.Pin   = ADC_BUSY_PIN;             /* PB1 */
    GPIO_InitStruct.Mode  = GPIO_MODE_IT_FALLING;     /* EXTI1 falling edge */
    GPIO_InitStruct.Pull  = GPIO_PULLDOWN;             /* FIX: was floating */
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(ADC_BUSY_PORT, &GPIO_InitStruct);

    /* ── GPIOC outputs ────────────────────────────────────────── */
    /* PC13 — LED (active LOW, open-drain on blue-pill boards)    */
    GPIO_InitStruct.Pin   = GPIO_PIN_13;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* ── EXTI1 NVIC (BUSY pin interrupt) ─────────────────────── */
    /*
     * Enabled HERE, after the GPIO is configured with a pull-down.
     * Enabling the NVIC before the pull-down is applied risks a
     * spurious interrupt on power-up if BUSY floats HIGH momentarily.
     */
    HAL_NVIC_SetPriority(EXTI1_IRQn, ADC_EXTI1_NVIC_PRIORITY, 0);  /* priority 1 */
    HAL_NVIC_EnableIRQ(EXTI1_IRQn);
}

/* ── Error handler ───────────────────────────────────────────────────────── */
void Error_Handler(void)
{
    __disable_irq();
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);  /* LED on */
    while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) { (void)file; (void)line; }
#endif
