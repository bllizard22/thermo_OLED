/**
  ******************************************************************************
  * File Name          : CAN.h
  * Description        : This file provides code for the configuration
  *                      of the CAN instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __can_H
#define __can_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan;

/* USER CODE BEGIN Private defines */
///**
//  * @brief  CAN Tx message structure definition
//  */
//typedef struct
//{
//  uint32_t StdId;    /*!< Specifies the standard identifier.
//                          This parameter must be a number between Min_Data = 0 and Max_Data = 0x7FF */
//
//  uint32_t ExtId;    /*!< Specifies the extended identifier.
//                          This parameter must be a number between Min_Data = 0 and Max_Data = 0x1FFFFFFF */
//
//  uint32_t IDE;      /*!< Specifies the type of identifier for the message that will be transmitted.
//                          This parameter can be a value of @ref CAN_Identifier_Type */
//
//  uint32_t RTR;      /*!< Specifies the type of frame for the message that will be transmitted.
//                          This parameter can be a value of @ref CAN_remote_transmission_request */
//
//  uint32_t DLC;      /*!< Specifies the length of the frame that will be transmitted.
//                          This parameter must be a number between Min_Data = 0 and Max_Data = 8 */
//
//  uint8_t Data[8];  /*!< Contains the data to be transmitted.
//                          This parameter must be a number between Min_Data = 0 and Max_Data = 0xFF */
//
//}CanTxMsgTypeDef;
//
///**
//  * @brief  CAN Rx message structure definition
//  */
//typedef struct
//{
//  uint32_t StdId;       /*!< Specifies the standard identifier.
//                             This parameter must be a number between Min_Data = 0 and Max_Data = 0x7FF */
//
//  uint32_t ExtId;       /*!< Specifies the extended identifier.
//                             This parameter must be a number between Min_Data = 0 and Max_Data = 0x1FFFFFFF */
//
//  uint32_t IDE;         /*!< Specifies the type of identifier for the message that will be received.
//                             This parameter can be a value of @ref CAN_Identifier_Type */
//
//  uint32_t RTR;         /*!< Specifies the type of frame for the received message.
//                             This parameter can be a value of @ref CAN_remote_transmission_request */
//
//  uint32_t DLC;         /*!< Specifies the length of the frame that will be received.
//                             This parameter must be a number between Min_Data = 0 and Max_Data = 8 */
//
//  uint8_t Data[8];      /*!< Contains the data to be received.
//                             This parameter must be a number between Min_Data = 0 and Max_Data = 0xFF */
//
//  uint32_t FMI;         /*!< Specifies the index of the filter the message stored in the mailbox passes through.
//                             This parameter must be a number between Min_Data = 0 and Max_Data = 0xFF */
//
//  uint32_t FIFONumber;  /*!< Specifies the receive FIFO number.
//                             This parameter can be CAN_FIFO0 or CAN_FIFO1 */
//
//}CanRxMsgTypeDef;
/* USER CODE END Private defines */

void MX_CAN_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ can_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
