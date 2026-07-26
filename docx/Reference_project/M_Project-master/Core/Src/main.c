/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  ******************************************************************************
  */
/* USER CODE END Header */
#include "main.h"
#include "dma.h"
#include "fdcan.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "state_machine.h"
#include "protocol.h"
/* USER CODE END Includes */

#ifndef APP_UART_DEBUG
#define APP_UART_DEBUG 0
#endif

void SystemClock_Config(void);

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_FDCAN1_Init();
  MX_USART1_UART_Init();
  MX_TIM6_Init();

  /* USER CODE BEGIN 2 */
  SET_BIT(huart1.Instance->CR1, USART_CR1_TE);

#if APP_UART_DEBUG
  char buf[128];
  int n = snprintf(buf, sizeof(buf),
                   "\r\n===== M_Project v2 DLC_FIX =====\r\n");
  HAL_UART_Transmit(&huart1, (uint8_t *)buf, n, 1000);
#endif

  __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);

  Protocol_Init();

  HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
      FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0,
      FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);

#if APP_UART_DEBUG
  HAL_UART_Transmit(&huart1, (uint8_t *)"Filter=OK\r\n", 11, 100);
#endif

  HAL_StatusTypeDef st = HAL_FDCAN_Start(&hfdcan1);
#if APP_UART_DEBUG
  n = snprintf(buf, sizeof(buf), "FDCAN_Start=%d\r\n", (int)st);
  HAL_UART_Transmit(&huart1, (uint8_t *)buf, n, 100);
#else
  (void)st;
#endif

  st = HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
#if APP_UART_DEBUG
  n = snprintf(buf, sizeof(buf), "Notif=%d PSR=0x%08lX\r\n",
               (int)st, (unsigned long)hfdcan1.Instance->PSR);
  HAL_UART_Transmit(&huart1, (uint8_t *)buf, n, 1000);
#else
  (void)st;
#endif

  HAL_TIM_Base_Start_IT(&htim6);

#if APP_UART_DEBUG
  n = snprintf(buf, sizeof(buf), "PRE_SM_INIT\r\n");
  HAL_UART_Transmit(&huart1, (uint8_t *)buf, n, 1000);
#endif

  SM_Init();

#if APP_UART_DEBUG
  n = snprintf(buf, sizeof(buf), "POST_SM_INIT RX=%lu ST=%d\r\n",
               (unsigned long)g_can_rx_count, (int)g_sm_state);
  HAL_UART_Transmit(&huart1, (uint8_t *)buf, n, 1000);
#endif

  /* USER CODE END 2 */
  while (1) {
    SM_Run();
  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK |
                                RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 |
                                RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
    Error_Handler();
  }
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) {
  }
}
