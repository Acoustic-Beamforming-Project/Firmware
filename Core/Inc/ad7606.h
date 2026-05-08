/* ad7606.h
 */

#ifndef AD7606_H
#define AD7606_H

#include "stm32f4xx_hal.h"
#include "ad7606_config.h"
#include <stdint.h>

/*  Pin control macros  */
#define AD7606_CS_ASSERT()       HAL_GPIO_WritePin(ADC_CS_PORT,    ADC_CS_PIN,    GPIO_PIN_RESET)
#define AD7606_CS_DEASSERT()     HAL_GPIO_WritePin(ADC_CS_PORT,    ADC_CS_PIN,    GPIO_PIN_SET)
#define AD7606_RESET_ASSERT()    HAL_GPIO_WritePin(ADC_RESET_PORT, ADC_RESET_PIN, GPIO_PIN_SET)
#define AD7606_RESET_DEASSERT()  HAL_GPIO_WritePin(ADC_RESET_PORT, ADC_RESET_PIN, GPIO_PIN_RESET)
#define AD7606_LED_TOGGLE()      HAL_GPIO_TogglePin(ADC_LED_PORT,  ADC_LED_PIN)

/* CONVST: falling edge triggers SAR. MIN LOW = 25 ns.
 * 4 NOPs @ 84 MHz = 47.6 ns. */
#define AD7606_CONVST_PULSE()                                                   \
    do {                                                                        \
        HAL_GPIO_WritePin(ADC_CONVST_PORT, ADC_CONVST_PIN, GPIO_PIN_RESET);     \
        __NOP(); __NOP(); __NOP(); __NOP();                                     \
        HAL_GPIO_WritePin(ADC_CONVST_PORT, ADC_CONVST_PIN, GPIO_PIN_SET);       \
    } while (0)

/*  Driver state  */
typedef enum {
    AD7606_STATE_UNINITIALIZED = 0,
    AD7606_STATE_IDLE,
    AD7606_STATE_RUNNING,
    AD7606_STATE_ERROR
} AD7606_State;

/*  AD7606_Frame  */
typedef struct {
    uint16_t raw[AD7606_NUM_CHANNELS];
    uint8_t  packet[AD7606_FRAME_SIZE];
    uint32_t timestamp_ticks;
} AD7606_Frame;

/*  AD7606_Batch  */
typedef struct {
    uint8_t  packet[AD7606_BATCH_SIZE];
    uint32_t batch_index;
    uint32_t timestamp_ticks;
} AD7606_Batch;

/*  Public API  */
void AD7606_Init(SPI_HandleTypeDef *hspi_douta,
                 SPI_HandleTypeDef *hspi_doutb,
                 TIM_HandleTypeDef *htim);
void         AD7606_Start(void);
void         AD7606_Stop(void);
uint8_t      AD7606_IsBatchReady(void);
void         AD7606_GetBatch(AD7606_Batch *out);
AD7606_State AD7606_GetState(void);
uint32_t     AD7606_GetDroppedBatches(void);
uint32_t     AD7606_GetFrameCount(void);
uint32_t     AD7606_GetOverruns(void);
uint32_t     AD7606_GetYDrops(void);
uint32_t     AD7606_GetSPIErrors(void);
int32_t      AD7606_RawToMillivolts(uint16_t raw);

/*  ISR hooks (called from stm32f4xx_it.c ONLY)  */
void AD7606_TriggerConversion(void);
void AD7606_OnBusyFallingEdge(void);
void AD7606_OnSPIComplete(SPI_HandleTypeDef *hspi);
void AD7606_OnSPIError(void);

#endif /* AD7606_H */
