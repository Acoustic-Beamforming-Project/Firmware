/* ad7606.c — AD7606 16-channel ADC driver
 * STM32F401RCT | SPI2 + SPI3 @ 16 MHz | DMA1 Stream3 + Stream0 | 48 kHz
 *
 * ═══════════════════════════════════════════════════════════════
 * OPTIMIZED DATA PATH:
 *   1. SPI2/SPI3 set to 8-bit mode (main.c).
 *   2. DMA destination points directly into Batch.packet.
 *   3. No memcpy for frame building.
 *   4. Triple buffering for batches to ensure W5500 transmission
 *      never blocks ADC acquisition.
 * ═══════════════════════════════════════════════════════════════
 */

#include "ad7606.h"
#include <string.h>

/* ── Peripheral handles ──────────────────────────────────────── */
static SPI_HandleTypeDef *_hspi_douta = NULL;
static SPI_HandleTypeDef *_hspi_doutb = NULL;
static TIM_HandleTypeDef *_htim       = NULL;

/* ── Driver state ────────────────────────────────────────────── */
static volatile AD7606_State _state = AD7606_STATE_UNINITIALIZED;

/* ── Acquisition state machine ───────────────────────────────── */
static volatile uint8_t _busy_count = 0;
static volatile uint8_t _round      = 0;
static volatile uint8_t _spi2_done  = 0;
static volatile uint8_t _spi3_done  = 0;

/* ── Period tick counter ─────────────────────────────────────── */
static volatile uint32_t _frame_tick = 0;

/* ── Triple Batch Buffering ──────────────────────────────────── */
static AD7606_Batch _batches[3];
static volatile uint8_t _fill_batch = 0;
static volatile uint8_t _ready_batch = 0xFF;  /* 0xFF = none ready */
static volatile uint8_t _frame_in_batch = 0;

/* ── Diagnostics ─────────────────────────────────────────────── */
static volatile uint32_t _total_frame_count = 0;
static volatile uint32_t _dropped_batches   = 0;
static volatile uint32_t _overrun_count     = 0;
static volatile uint32_t _spi_error_count   = 0;

/* ── Private prototypes ──────────────────────────────────────── */
static void _HardwareReset(void);
static void _LaunchSPIRound(void);

/* ================================================================
 * _HardwareReset
 * ================================================================ */
static void _HardwareReset(void)
{
    AD7606_CS_DEASSERT();
    HAL_GPIO_WritePin(ADC_CONVST_PORT, ADC_CONVST_PIN, GPIO_PIN_SET);
    AD7606_HOLD_RELEASE();
    AD7606_CHSEL_X();
    AD7606_RESET_DEASSERT();

    HAL_Delay(1);
    AD7606_RESET_ASSERT();
    HAL_Delay(1);
    AD7606_RESET_DEASSERT();
    HAL_Delay(5);
}

/* ================================================================
 * _LaunchSPIRound
 * ================================================================ */
static void _LaunchSPIRound(void)
{
    _spi2_done = 0;
    _spi3_done = 0;

    __HAL_SPI_CLEAR_OVRFLAG(_hspi_douta);
    __HAL_SPI_CLEAR_OVRFLAG(_hspi_doutb);

    /* Get pointer to the start of data for current frame in current batch */
    uint8_t *p = _batches[_fill_batch].packet + (_frame_in_batch * AD7606_FRAME_SIZE) + 4;

    /* SPI2 -> CH1-4 (Round 0) or CH5-8 (Round 1)
     * SPI3 -> CH9-12 (Round 0) or CH13-16 (Round 1) */
    uint8_t *dst2, *dst3;
    if (_round == 0) {
        dst2 = p;           /* CH1-4 */
        dst3 = p + 16;      /* CH9-12 */
    } else {
        dst2 = p + 8;       /* CH5-8 */
        dst3 = p + 24;      /* CH13-16 */
    }

    HAL_SPI_Receive_DMA(_hspi_douta, dst2, 8);  /* 4 channels * 2 bytes */
    HAL_SPI_Receive_DMA(_hspi_doutb, dst3, 8);
}

/* ================================================================
 * Public API
 * ================================================================ */

void AD7606_Init(SPI_HandleTypeDef *hspi_douta,
                 SPI_HandleTypeDef *hspi_doutb,
                 TIM_HandleTypeDef *htim)
{
    _hspi_douta = hspi_douta;
    _hspi_doutb = hspi_doutb;
    _htim       = htim;

    _HardwareReset();

    memset(_batches, 0, sizeof(_batches));
    for (int b=0; b<3; b++) {
        for (int f=0; f<AD7606_BATCH_FRAMES; f++) {
            uint8_t *fp = _batches[b].packet + (f * AD7606_FRAME_SIZE);
            fp[0] = 0xDE; fp[1] = 0xAD; fp[2] = 0xBE; fp[3] = 0xEF;
        }
    }

    _fill_batch = 0;
    _ready_batch = 0xFF;
    _frame_in_batch = 0;
    _total_frame_count = 0;
    _overrun_count = 0;

    _state = AD7606_STATE_IDLE;
}

void AD7606_Start(void)
{
    if (_state != AD7606_STATE_IDLE) return;
    _state = AD7606_STATE_RUNNING;
    HAL_TIM_Base_Start_IT(_htim);
}

void AD7606_Stop(void)
{
    HAL_TIM_Base_Stop_IT(_htim);
    HAL_SPI_DMAStop(_hspi_douta);
    HAL_SPI_DMAStop(_hspi_doutb);
    AD7606_CS_DEASSERT();
    _state = AD7606_STATE_IDLE;
}

uint8_t AD7606_IsBatchReady(void) { return (_ready_batch != 0xFF); }

void AD7606_GetBatch(AD7606_Batch *out)
{
    if (out == NULL || _ready_batch == 0xFF) return;

    memcpy(out, &_batches[_ready_batch], sizeof(AD7606_Batch));
    
    __disable_irq();
    _ready_batch = 0xFF;
    __enable_irq();
}

uint32_t AD7606_GetFrameCount(void) { return _total_frame_count; }

int32_t AD7606_RawToMillivolts(uint16_t raw)
{
    return ((int32_t)(int16_t)raw * (int32_t)AD7606_VREF_MV) / (int32_t)AD7606_FULLSCALE_CODE;
}

void AD7606_OnSPIError(void)
{
    _spi_error_count++;
    _busy_count = 0;
    _round = 0;
}

/* ── ISR hooks ───────────────────────────────────────────────── */

void AD7606_TriggerConversion(void)
{
    if (_state != AD7606_STATE_RUNNING) return;
    _frame_tick++;
    if (_busy_count != 0) _overrun_count++;
    _busy_count = 0;
    _round = 0;

    AD7606_HOLD_ASSERT();
    AD7606_CHSEL_X();
    AD7606_CONVST_PULSE();
    AD7606_CHSEL_Y();
}

void AD7606_OnBusyFallingEdge(void)
{
    if (_state != AD7606_STATE_RUNNING) return;
    _busy_count++;
    if (_busy_count == 1) {
        AD7606_CS_ASSERT();
        _LaunchSPIRound();
    } else if (_busy_count == 2) {
        AD7606_CS_ASSERT();
        _LaunchSPIRound();
    }
}

void AD7606_OnSPIComplete(SPI_HandleTypeDef *hspi)
{
    if (_state != AD7606_STATE_RUNNING) return;
    if (hspi->Instance == SPI2) _spi2_done = 1;
    else if (hspi->Instance == SPI3) _spi3_done = 1;
    else return;

    if (!_spi2_done || !_spi3_done) return;
    AD7606_CS_DEASSERT();

    if (_busy_count == 1) {
        _round = 1;
        AD7606_CONVST_PULSE();
        AD7606_HOLD_RELEASE();
        AD7606_CHSEL_X();
    } else if (_busy_count == 2) {
        _total_frame_count++;
        _frame_in_batch++;
        if (_frame_in_batch >= AD7606_BATCH_FRAMES) {
            /* Current batch is full */
            _ready_batch = _fill_batch;
            _fill_batch = (_fill_batch + 1) % 3;
            if (_fill_batch == _ready_batch) {
                /* Overrunning all 3 buffers, skip next */
                _fill_batch = (_fill_batch + 1) % 3;
                _dropped_batches++;
            }
            _frame_in_batch = 0;
        }
        _round = 0;
        _busy_count = 0;
    }
}
