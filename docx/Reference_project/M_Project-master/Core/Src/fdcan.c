/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    fdcan.c
  * @brief   This file provides code for the configuration
  *          of the FDCAN instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "fdcan.h"

/* USER CODE BEGIN 0 */
#include "usart.h"
#include <stdio.h>
#include <string.h>

#ifndef CAN_UART_DEBUG
#define CAN_UART_DEBUG 0
#endif

// CAN 调试打印（通过串口输出）
static void can_debug_print(const char *msg)
{
#if CAN_UART_DEBUG
    extern UART_HandleTypeDef huart1;
    HAL_UART_Transmit(&huart1, (uint8_t *)msg, (uint16_t)strlen(msg), 100);
#else
    (void)msg;
#endif
}

/* USER CODE END 0 */

FDCAN_HandleTypeDef hfdcan1;

/* FDCAN1 init function */
void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 17;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 15;
  hfdcan1.Init.NominalTimeSeg2 = 4;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.StdFiltersNbr = 0;
  hfdcan1.Init.ExtFiltersNbr = 0;    // 用 GlobalFilter 的 ACCEPT 模式代替滤波器
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef* fdcanHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  if(fdcanHandle->Instance==FDCAN1)
  {
  /* USER CODE BEGIN FDCAN1_MspInit 0 */

  /* USER CODE END FDCAN1_MspInit 0 */

  /** Initializes the peripherals clocks
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* FDCAN1 clock enable */
    __HAL_RCC_FDCAN_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**FDCAN1 GPIO Configuration
    PA11     ------> FDCAN1_RX
    PA12     ------> FDCAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* FDCAN1 interrupt Init */
    HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
    HAL_NVIC_SetPriority(FDCAN1_IT1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(FDCAN1_IT1_IRQn);
  /* USER CODE BEGIN FDCAN1_MspInit 1 */

  /* USER CODE END FDCAN1_MspInit 1 */
  }
}

void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef* fdcanHandle)
{

  if(fdcanHandle->Instance==FDCAN1)
  {
  /* USER CODE BEGIN FDCAN1_MspDeInit 0 */

  /* USER CODE END FDCAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_FDCAN_CLK_DISABLE();

    /**FDCAN1 GPIO Configuration
    PA11     ------> FDCAN1_RX
    PA12     ------> FDCAN1_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

    /* FDCAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(FDCAN1_IT0_IRQn);
    HAL_NVIC_DisableIRQ(FDCAN1_IT1_IRQn);
  /* USER CODE BEGIN FDCAN1_MspDeInit 1 */

  /* USER CODE END FDCAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
/**
  * @brief   FDCAN发送多帧命令（自动拆分包），每帧首字节为功能码
  * @param   cmd: 命令数据缓冲区指针（cmd[0]=地址, cmd[1]=功能码, cmd[2..]=数据+校验）
  * @param   len: 命令数据总长度
  * @retval  无
  */
void can_SendCmd(uint8_t *cmd, uint8_t len)
{
  FDCAN_TxHeaderTypeDef TxHeader;
  uint8_t              txData[8];
  uint8_t              i = 0, j = 0, k = 0, l = 0, packNum = 0;

  // 去掉ID地址和功能码后的数据长度
  j = len - 2;

  // 初始化FDCAN发送报头（固定字段，每帧相同）
  TxHeader.IdType              = FDCAN_EXTENDED_ID;
  TxHeader.TxFrameType         = FDCAN_DATA_FRAME;
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  TxHeader.BitRateSwitch       = FDCAN_BRS_OFF;
  TxHeader.FDFormat            = FDCAN_CLASSIC_CAN;
  TxHeader.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
  TxHeader.MessageMarker       = 0;

  // ⚡ 调试：打印CAN发送信息
  {
#if CAN_UART_DEBUG
    char dbg[80];
    snprintf(dbg, sizeof(dbg), "[CAN_TX] addr=0x%02X func=0x%02X len=%d\r\n",
             cmd[0], cmd[1], len);
    can_debug_print(dbg);
#endif
  }

  while (i < j)
  {
    memset(txData, 0, sizeof(txData));

    // 剩余待发送数据长度
    k = j - i;

    // 装载帧：扩展ID = (地址<<8) | 包序号；帧首字节 = 功能码
    TxHeader.Identifier = ((uint32_t)cmd[0] << 8) | (uint32_t)packNum;
    txData[0] = cmd[1];

    // 剩余数据不足7字节（CAN帧最多8字节，首字节已占1个），全部填入
    if (k < 7)
    {
      for (l = 0; l < k; l++, i++) { txData[l + 1] = cmd[i + 2]; }
      TxHeader.DataLength = (uint32_t)(k + 1);  // 1 ~ 7
    }
    // 剩余数据 >= 7字节，发送满8字节帧
    else
    {
      for (l = 0; l < 7; l++, i++) { txData[l + 1] = cmd[i + 2]; }
      TxHeader.DataLength = FDCAN_DLC_BYTES_8;
    }

    // 写入TX FIFO，带超时重试保护（防止CAN总线异常时死循环）
    {
      HAL_StatusTypeDef status;
      uint32_t start_tick = HAL_GetTick();
      while ((status = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, txData)) != HAL_OK) {
        if ((HAL_GetTick() - start_tick) > 20U) {
          can_debug_print("[CAN_TX] FAIL: FIFO full or bus error\r\n");
          return;
        }
        HAL_Delay(1);
      }
      // ⚡ 调试：打印成功发送的帧
      {
#if CAN_UART_DEBUG
        char dbg[120];
        snprintf(dbg, sizeof(dbg),
                 "[CAN_TX] OK pkt=%d id=0x%08lX dlc=%lu data: %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
                 packNum, (unsigned long)TxHeader.Identifier,
                 (unsigned long)TxHeader.DataLength,
                 txData[0], txData[1], txData[2], txData[3],
                 txData[4], txData[5], txData[6], txData[7]);
        can_debug_print(dbg);
#endif
      }
    }

    // 包序号递增
    ++packNum;
  }
}

/* USER CODE END 1 */
