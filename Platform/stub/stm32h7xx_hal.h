/* stm32h7xx_hal.h - HOST-ONLY STUB for syntax checking Platform/stm32h7 (scripts/check.sh). It reproduces, from the
 * HAL documentation, only the names the platform layer uses. It is NOT the real HAL and proves nothing about
 * behaviour; it catches typos, type mismatches and missing includes in OUR code. Never add it to a firmware build. */
#ifndef STUB_STM32H7XX_HAL_H
#define STUB_STM32H7XX_HAL_H
#include <stdint.h>
typedef enum { HAL_OK = 0, HAL_ERROR = 1, HAL_BUSY = 2, HAL_TIMEOUT = 3 } HAL_StatusTypeDef;
typedef enum { GPIO_PIN_RESET = 0, GPIO_PIN_SET } GPIO_PinState;
typedef struct { int dummy; } GPIO_TypeDef;
extern GPIO_TypeDef *GPIOA, *GPIOB;
#define GPIO_PIN_0 0x0001u
#define GPIO_PIN_1 0x0002u
#define GPIO_PIN_3 0x0008u
#define GPIO_PIN_4 0x0010u
#define SPI_POLARITY_LOW  0u
#define SPI_PHASE_1EDGE   0u
#define SPI_PHASE_2EDGE   1u
typedef struct { uint32_t CLKPolarity, CLKPhase; } SPI_InitTypeDef;
typedef struct { SPI_InitTypeDef Init; } SPI_HandleTypeDef;
typedef struct { int dummy; } TIM_HandleTypeDef;
typedef struct { int dummy; } FDCAN_HandleTypeDef;
typedef struct {
    uint32_t Identifier, IdType, TxFrameType, DataLength, ErrorStateIndicator, BitRateSwitch, FDFormat, TxEventFifoControl, MessageMarker;
} FDCAN_TxHeaderTypeDef;
#define FDCAN_EXTENDED_ID 0x40000000u
#define FDCAN_DATA_FRAME 0u
#define FDCAN_ESI_ACTIVE 0u
#define FDCAN_BRS_OFF 0u
#define FDCAN_CLASSIC_CAN 0u
#define FDCAN_NO_TX_EVENTS 0u
#define FDCAN_DLC_BYTES_0 0x00000000u
#define FDCAN_DLC_BYTES_1 0x00010000u
#define FDCAN_DLC_BYTES_2 0x00020000u
#define FDCAN_DLC_BYTES_3 0x00030000u
#define FDCAN_DLC_BYTES_4 0x00040000u
#define FDCAN_DLC_BYTES_5 0x00050000u
#define FDCAN_DLC_BYTES_6 0x00060000u
#define FDCAN_DLC_BYTES_7 0x00070000u
#define FDCAN_DLC_BYTES_8 0x00080000u
uint32_t stub_tim_counter(TIM_HandleTypeDef *h);
#define __HAL_TIM_GET_COUNTER(h) stub_tim_counter(h)
HAL_StatusTypeDef HAL_SPI_Init(SPI_HandleTypeDef *hspi);
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size, uint32_t Timeout);
void HAL_GPIO_WritePin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState);
HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef *hfdcan, FDCAN_TxHeaderTypeDef *pTxHeader, uint8_t *pTxData);
#endif
