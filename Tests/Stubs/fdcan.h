#pragma once

#include <stdint.h>

typedef enum {
    HAL_OK = 0,
    HAL_ERROR = 1
} HAL_StatusTypeDef;

typedef struct {
    void *Instance;
} FDCAN_HandleTypeDef;

typedef struct {
    uint32_t Identifier;
    uint32_t IdType;
    uint32_t TxFrameType;
    uint32_t DataLength;
    uint32_t ErrorStateIndicator;
    uint32_t BitRateSwitch;
    uint32_t FDFormat;
    uint32_t TxEventFifoControl;
    uint32_t MessageMarker;
} FDCAN_TxHeaderTypeDef;

typedef struct {
    uint32_t Identifier;
    uint32_t IdType;
    uint32_t RxFrameType;
    uint32_t DataLength;
    uint32_t ErrorStateIndicator;
    uint32_t BitRateSwitch;
    uint32_t FDFormat;
} FDCAN_RxHeaderTypeDef;

typedef struct {
    uint32_t IdType;
    uint32_t FilterIndex;
    uint32_t FilterType;
    uint32_t FilterConfig;
    uint32_t FilterID1;
    uint32_t FilterID2;
} FDCAN_FilterTypeDef;

typedef struct {
    uint32_t LastErrorCode;
    uint32_t DataLastErrorCode;
    uint32_t Activity;
    uint32_t ErrorPassive;
    uint32_t Warning;
    uint32_t BusOff;
    uint32_t RxESIflag;
    uint32_t RxBRSflag;
    uint32_t RxFDFflag;
    uint32_t ProtocolException;
    uint32_t TDCvalue;
} FDCAN_ProtocolStatusTypeDef;

#define FDCAN1 ((void *)(uintptr_t)0x4000A400U)

#define FDCAN_STANDARD_ID                 0x00000000U
#define FDCAN_EXTENDED_ID                 0x40000000U
#define FDCAN_DATA_FRAME                  0x00000000U
#define FDCAN_REMOTE_FRAME                0x20000000U
#define FDCAN_ESI_ACTIVE                  0x00000000U
#define FDCAN_BRS_OFF                     0x00000000U
#define FDCAN_CLASSIC_CAN                 0x00000000U
#define FDCAN_FD_CAN                      0x00200000U
#define FDCAN_NO_TX_EVENTS                0x00000000U

#define FDCAN_DLC_BYTES_0                 0x00000000U
#define FDCAN_DLC_BYTES_1                 0x00000001U
#define FDCAN_DLC_BYTES_2                 0x00000002U
#define FDCAN_DLC_BYTES_3                 0x00000003U
#define FDCAN_DLC_BYTES_4                 0x00000004U
#define FDCAN_DLC_BYTES_5                 0x00000005U
#define FDCAN_DLC_BYTES_6                 0x00000006U
#define FDCAN_DLC_BYTES_7                 0x00000007U
#define FDCAN_DLC_BYTES_8                 0x00000008U

#define FDCAN_FILTER_RANGE                0x00000000U
#define FDCAN_FILTER_TO_RXFIFO0           0x00000001U
#define FDCAN_REJECT                      0x00000002U
#define FDCAN_REJECT_REMOTE               0x00000003U
#define FDCAN_RX_FIFO0                    0x00000000U
#define FDCAN_IT_RX_FIFO0_NEW_MESSAGE     0x00000001U
#define FDCAN_TX_BUFFER0                  0x00000001U
#define FDCAN_TX_BUFFER1                  0x00000002U
#define FDCAN_TX_BUFFER2                  0x00000004U

extern FDCAN_HandleTypeDef hfdcan1;

HAL_StatusTypeDef HAL_FDCAN_ConfigFilter(
    FDCAN_HandleTypeDef *hfdcan,
    const FDCAN_FilterTypeDef *filter);

HAL_StatusTypeDef HAL_FDCAN_ConfigGlobalFilter(
    FDCAN_HandleTypeDef *hfdcan,
    uint32_t non_matching_std,
    uint32_t non_matching_ext,
    uint32_t reject_remote_std,
    uint32_t reject_remote_ext);

HAL_StatusTypeDef HAL_FDCAN_Start(
    FDCAN_HandleTypeDef *hfdcan);

HAL_StatusTypeDef HAL_FDCAN_Stop(
    FDCAN_HandleTypeDef *hfdcan);

HAL_StatusTypeDef HAL_FDCAN_ActivateNotification(
    FDCAN_HandleTypeDef *hfdcan,
    uint32_t active_interrupts,
    uint32_t buffer_indexes);

uint32_t HAL_FDCAN_GetTxFifoFreeLevel(
    const FDCAN_HandleTypeDef *hfdcan);

HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(
    FDCAN_HandleTypeDef *hfdcan,
    const FDCAN_TxHeaderTypeDef *header,
    const uint8_t *data);

HAL_StatusTypeDef HAL_FDCAN_AbortTxRequest(
    FDCAN_HandleTypeDef *hfdcan,
    uint32_t buffer_index);

HAL_StatusTypeDef HAL_FDCAN_GetProtocolStatus(
    const FDCAN_HandleTypeDef *hfdcan,
    FDCAN_ProtocolStatusTypeDef *status);

uint32_t HAL_FDCAN_GetRxFifoFillLevel(
    const FDCAN_HandleTypeDef *hfdcan,
    uint32_t rx_fifo);

HAL_StatusTypeDef HAL_FDCAN_GetRxMessage(
    FDCAN_HandleTypeDef *hfdcan,
    uint32_t rx_location,
    FDCAN_RxHeaderTypeDef *header,
    uint8_t *data);
