/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "can.h"

/* USER CODE BEGIN 0 */
#include "usart.h"
#include "tim.h"

CAN_RxHeaderTypeDef   	RxHeader;
uint8_t               	RxData[8];
CAN_TxHeaderTypeDef   	TxHeader;
uint8_t               	TxData[8];
uint32_t              	TxMailbox;

/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 8;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_8TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_7TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);

    CAN_FilterTypeDef sFilterConfig;

    sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    sFilterConfig.FilterIdHigh = 0x1 << 5;
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = 0x7F << 5;
    sFilterConfig.FilterMaskIdLow = 0x0000;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.FilterBank = 0;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.SlaveStartFilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);


  	if (HAL_CAN_Start(&hcan1) != HAL_OK) {
  		Error_Handler();
  	}

  	if(HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  	{
  		Error_Handler();
  	}

    CAN_TX_filter_init();


  /* USER CODE END CAN1_Init 2 */

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**CAN1 GPIO Configuration
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN1 GPIO Configuration
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

void CAN_TX_filter_init(void)
{
	TxHeader.StdId = 0x360;
	TxHeader.ExtId = 0;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.DLC = 8;
	TxHeader.TransmitGlobalTime = DISABLE;
}


void CAN_SendState(int state)
{
    TxData[0] = (state & 0xFF00) >> 8;
    TxData[1] = state & 0x00FF;
    TxData[2] = 0;
    TxData[3] = 0;
    TxData[4] = 0;
    TxData[5] = 0;
    TxData[6] = 0;
    TxData[7] = 0;

    if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, (uint8_t*)TxData, &TxMailbox) != HAL_OK)
    {
    	Error_Handler();
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
  {
	  Error_Handler();
  }

  __HAL_TIM_SET_COUNTER(&htim7, 0);
  timExpired = 0;
}

void CAN_Print_Errors(void) {
	char 					uart_buf[50];
	uint8_t 				uart_buf_len;
	uint32_t error_code = HAL_CAN_GetError(&hcan1);

    // Log the error code (Optional, use UART or a debugger)
    uart_buf_len = sprintf(uart_buf, "CAN Error Code: 0x%08lX\n\r", error_code);
    UART_Send(uart_buf, uart_buf_len);

    // Error Handling
    if (error_code == HAL_CAN_ERROR_NONE) {
        // No error, do nothing
        return;
    }

    if (error_code & HAL_CAN_ERROR_EWG) {
        SEND_STRING("Protocol Error Warning detected.\n\r");
    }

    if (error_code & HAL_CAN_ERROR_EPV) {
    	SEND_STRING("Error Passive state detected.\n\r");
    }

    if (error_code & HAL_CAN_ERROR_BOF) {
    	SEND_STRING("Bus-Off error detected.\n\r");
    }

    if (error_code & HAL_CAN_ERROR_STF) {
    	SEND_STRING("Stuff error detected.\n\r");
    }

    if (error_code & HAL_CAN_ERROR_FOR) {
    	SEND_STRING("Form error detected.\n\r");
    }

    if (error_code & HAL_CAN_ERROR_ACK) {
    	SEND_STRING("Acknowledgment error detected.\n\r");
    }

    if (error_code & HAL_CAN_ERROR_BR) {
    	SEND_STRING("Bit Recessive error detected.\n\r");
    }

    if (error_code & HAL_CAN_ERROR_BD) {
    	SEND_STRING("Bit Dominant error detected.\n\r");
    }

    if (error_code & HAL_CAN_ERROR_CRC) {
    	SEND_STRING("CRC error detected.\n\r");
    }

    if (error_code & HAL_CAN_ERROR_RX_FOV0) {
    	SEND_STRING("RX FIFO0 Overrun detected.\n\r");
    }

    if (error_code & HAL_CAN_ERROR_RX_FOV1) {
    	SEND_STRING("RX FIFO1 Overrun detected.\n\r");
    }

    if (error_code & HAL_CAN_ERROR_TX_ALST0) {
    	SEND_STRING("TX Mailbox 0 Arbitration lost.\n\r");
    }

    if (error_code & HAL_CAN_ERROR_TX_TERR0) {
    	SEND_STRING("TX Mailbox 0 Transmission error.\n\r");
    }

    if (error_code & HAL_CAN_ERROR_TX_ALST1) {
    	SEND_STRING("TX Mailbox 1 Arbitration lost.\n\r");
    }

    if (error_code & HAL_CAN_ERROR_TX_TERR1) {
    	SEND_STRING("TX Mailbox 1 Transmission error.\n\r");
    }

    if (error_code & HAL_CAN_ERROR_TX_ALST2) {
    	SEND_STRING("TX Mailbox 2 Arbitration lost.\n\r");
    }

    if (error_code & HAL_CAN_ERROR_TX_TERR2) {
    	SEND_STRING("TX Mailbox 2 Transmission error.\n\r");
    }

    if (error_code & HAL_CAN_ERROR_TIMEOUT) {
    	SEND_STRING("Timeout error detected.\n\r");
    }

    if (error_code & HAL_CAN_ERROR_NOT_INITIALIZED) {
    	SEND_STRING("CAN Peripheral not initialized.\n\r");
    }

    if (error_code & HAL_CAN_ERROR_NOT_READY) {
    	SEND_STRING("CAN Peripheral not ready.\n\r");
    }

    if (error_code & HAL_CAN_ERROR_NOT_STARTED) {
    	SEND_STRING("CAN Peripheral not started.\n\r");
    }

    if (error_code & HAL_CAN_ERROR_PARAM) {
    	SEND_STRING("Parameter error detected.\n\r");
    }

#if USE_HAL_CAN_REGISTER_CALLBACKS == 1
    if (error_code & HAL_CAN_ERROR_INVALID_CALLBACK) {
    	SEND_STRING("Invalid callback error detected.\n\r");
    }
#endif

    if (error_code & HAL_CAN_ERROR_INTERNAL) {
    	SEND_STRING("Internal error detected.\n\r");
    }
}


/* USER CODE END 1 */
