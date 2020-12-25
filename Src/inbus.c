
#include <string.h>

#include "can.h"

//#include "generator.h"
#include "inbus.h"

__no_init CAN_FilterTypeDef		sFilterConfig;
__no_init CAN_TxHeaderTypeDef 	TxMsgHdr;
__no_init CAN_RxHeaderTypeDef 	RxMsgHdr;
//__no_init CAN_RxHeaderTypeDef 	RxMsgHdr2;
__no_init char rx_msg_data[8];
__no_init char tx_msg_data[8];
uint32_t TxMailbox;
//CanTxMsgTypeDef			TxMsg;
//CanRxMsgTypeDef			RxMsg;
//CanRxMsgTypeDef			RxMsg0;
HAL_StatusTypeDef 		stat;
//HAL_CAN_StateTypeDef 	can_stat;
//uint32_t 				can_err;
//char					is_transmit_ready;
char can_receive_flag;

//------------------------------------------------------------------------------
/// Init CAN bus and start receiving packets
//------------------------------------------------------------------------------
void inbus_init( void )
{
	// Init variables
	//is_transmit_ready = 1;
	can_receive_flag = 0;
	memset( (void *)&TxMsgHdr, 0, sizeof(TxMsgHdr) );
	memset( (void *)&RxMsgHdr, 0, sizeof(RxMsgHdr) );
	//memset( (void *)&RxMsgHdr2, 0, sizeof(RxMsgHdr2) );
	memset( tx_msg_data, 0, sizeof(tx_msg_data) );
	memset( rx_msg_data, 0, sizeof(rx_msg_data) );
	// Init CAN structures
	TxMsgHdr.IDE = CAN_ID_STD;
	TxMsgHdr.RTR = CAN_RTR_DATA;
        TxMsgHdr.DLC = 8;
	//	hcan.pTxMsg = &TxMsg;
	//	hcan.pRxMsg = &RxMsg0;
	//	hcan.pTxMsg->StdId = INBUS_DEVICE_ID;
	//	hcan.pTxMsg->ExtId = 0;
	//	hcan.pTxMsg->RTR = CAN_RTR_DATA;
	//	hcan.pTxMsg->IDE = CAN_ID_STD;
	// Configure the CAN Filter
	sFilterConfig.FilterBank = 0;
	//	sFilterConfig.FilterNumber = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDLIST;//CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_16BIT;//CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = INBUS_DEVICE_ID << 5;//INBUS_DEVICE_ID << 5;
	sFilterConfig.FilterIdLow = INBUS_MASTER_ID << 5;//0x0000;
	sFilterConfig.FilterMaskIdHigh = 0x0000;//INBUS_FILTER_MASK << 5;
	sFilterConfig.FilterMaskIdLow = 0x0000;
	sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 14;
	stat = HAL_CAN_ConfigFilter( &hcan, &sFilterConfig );
	if( stat != HAL_OK ){
		/* Filter configuration Error */
		Error_Handler();
	}
	// Start the CAN interface
	stat = HAL_CAN_Start( &hcan );
	if( stat != HAL_OK ){
		/* CAN start Error */
		Error_Handler();
	}
	// Set the receiver notification
	stat = HAL_CAN_ActivateNotification( &hcan, CAN_IT_RX_FIFO0_MSG_PENDING );
	if( stat != HAL_OK ){
		/* CAN Set notification Error */
		Error_Handler();
	}
	// Start receiving
	//HAL_CAN_Receive_IT( &hcan, CAN_FIFO0 );
        TxMailbox = 0;
        HAL_CAN_AddTxMessage(&hcan, &TxMsgHdr, tx_msg_data, &TxMailbox);
}

//------------------------------------------------------------------------------
/// Init CAN bus and start receiving packets
//------------------------------------------------------------------------------
void inbus_process( void )
{
	uint32_t val;

	// Check for data received
	if( !can_receive_flag ){
		return;
	}
	can_receive_flag = 0;
	// Check for bus request
	switch( RxMsgHdr.StdId ){
		//	case INBUS_MASTER_ID:		// Device address request
		//		if( RxMsg.Data[0] == 0 && RxMsg.Data[1] == 0 ){
		//			hcan.pTxMsg->StdId = INBUS_MASTER_ID;
		//			hcan.pTxMsg->DLC = 2;
		//			*(uint16_t *)hcan.pTxMsg->Data = (uint16_t)INBUS_DEVICE_ID;
		//			if( is_transmit_ready ){
		//				is_transmit_ready = 0;
		//				stat =  HAL_CAN_Transmit_IT( &hcan );
		//			}
		//		}
		//		break;
	case INBUS_DEVICE_ID:	// Data request. Specific for every device type
		if( rx_msg_data[0] & 0x80 ){
			// Write the data
			val = (uint32_t)rx_msg_data[1] | ((uint32_t)rx_msg_data[2] << 8) | ((uint32_t)rx_msg_data[3] << 16) | ((uint32_t)rx_msg_data[4] << 24);
			gen_set_frequency( val & 0xFFFF );
			//printf("set %d\n", val );
		} else {
			// Read the data
			//			hcan.pTxMsg->StdId = INBUS_DEVICE_ID;
			//			hcan.pTxMsg->DLC = INBUS_FRAME_SIZE;
			TxMsgHdr.StdId = INBUS_DEVICE_ID;
			TxMsgHdr.ExtId = 0;
			TxMsgHdr.DLC = 6;//INBUS_FRAME_SIZE;
			TxMsgHdr.IDE = CAN_ID_STD;
			TxMsgHdr.RTR = CAN_RTR_DATA;
			//			hcan.pTxMsg->Data[0] = 0x01;
			//			hcan.pTxMsg->Data[1] = val & 0xFF;
			//			hcan.pTxMsg->Data[2] = (val >>  8) & 0xFF;
			//			hcan.pTxMsg->Data[3] = (val >> 16) & 0xFF;
			//			hcan.pTxMsg->Data[4] = (val >> 24) & 0xFF;
			tx_msg_data[0] = 0x01;					// 01 = Actual Data Returned
			tx_msg_data[1] = gen_get_status();		// Generator state
			val = gen_get_frequency();				// Frequency in Hz
			tx_msg_data[2] = val & 0xFF;
			tx_msg_data[3] = (val >>  8) & 0xFF;
			tx_msg_data[4] = (val >> 16) & 0xFF;
			tx_msg_data[5] = (val >> 24) & 0xFF;
			//			if( is_transmit_ready ){
			//				is_transmit_ready = 0;
			//				stat =  HAL_CAN_Transmit_IT( &hcan );
			//			}
			//HAL_CAN_GetTxMailboxesFreeLevel
			//        if (HAL_CAN_AddTxMessage(&CanHandle, &TxHeader, TxData, &TxMailbox) != HAL_OK)
			//HAL_CAN_AddTxMessage( &hcan, &TxMsgHdr, (uint8_t *)tx_msg_data, (uint32_t *)CAN_TX_MAILBOX0 );
			HAL_CAN_AddTxMessage( &hcan, &TxMsgHdr, (uint8_t *)tx_msg_data, &TxMailbox );
			//printf("freq = %d\n", val );
		}
		break;
	default:
		break;
	}
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//==============================================================================
///  @brief  CAN callback. Has been calles then there is at least 1 message
///          received in FIFO.
///  @param  CanHandle: pointer to a CAN_HandleTypeDef structure that contains
///          the configuration information for the specified CAN.
///  @retval None
//==============================================================================
void HAL_CAN_RxFifo0MsgPendingCallback( CAN_HandleTypeDef *phcan )
{
	HAL_StatusTypeDef   HAL_RetVal;

	HAL_RetVal = HAL_CAN_GetRxMessage( phcan, CAN_RX_FIFO0, &RxMsgHdr, (uint8_t *)rx_msg_data );
	if( HAL_RetVal == HAL_OK ){
		//inbus_process();
		// Copy the header
		//memcpy( (void *)&RxMsgHdr2, (void *)&RxMsgHdr, sizeof(RxMsgHdr) );
		can_receive_flag = 1;
	}
}

/*
//==============================================================================
///  @brief
///  @param  CanHandle: pointer to a CAN_HandleTypeDef structure that contains
///          the configuration information for the specified CAN.
///  @retval None
//==============================================================================
void HAL_CAN_TxCpltCallback( CAN_HandleTypeDef *CanHandle )
{
	 is_transmit_ready = 1;
}


//==============================================================================
///  @brief  Receiving complete callback in non blocking mode
///  @param  CanHandle: pointer to a CAN_HandleTypeDef structure that contains
///          the configuration information for the specified CAN.
///  @retval None
//==============================================================================
void HAL_CAN_RxCpltCallback( CAN_HandleTypeDef *CanHandle )
{
	// Check for standard ID
	if( (CanHandle->pRxMsg->IDE == CAN_ID_STD) ){
		// ID is standard. Check for known IDs
		inbus_process();
	}
	// Receive
	HAL_CAN_Receive_IT( CanHandle, CAN_FIFO0 );
}

//==============================================================================
///  @brief  Error CAN callback.
///  @param  CanHandle: pointer to a CAN_HandleTypeDef structure that contains
///          the configuration information for the specified CAN.
///  @retval None
//==============================================================================
void HAL_CAN_ErrorCallback( CAN_HandleTypeDef *CanHandle )
{
	uint32_t errno;

	errno = HAL_CAN_GetError( CanHandle );
	switch( errno ){
	case HAL_CAN_ERROR_EWG:		// Transmit/receive error counter >= 96
		//printf( "EWG\n" );
		break;
	case HAL_CAN_ERROR_EPV:		// Transmit/receive error counter > 127
		//printf( "EPV\n" );
		break;
	case HAL_CAN_ERROR_BOF:		// Device is in the bus-off state (automatic when the Transmit error counter > 255)
		//printf( "BOF\n" );
		break;
	case HAL_CAN_ERROR_STF:		// Bit stuffing error - the 6th bit is 1 instead of 0
		//printf( "STF\n" );
		break;
	case HAL_CAN_ERROR_FOR:		// CAN frame error - one or more static fields is bad
		//printf( "FOR\n" );
		break;
	case HAL_CAN_ERROR_ACK:		// Acknowledgment Error - Sent frqme received incorrectly by 1 or more receivers
		//printf( "ACK\n" );
		break;
	case HAL_CAN_ERROR_BR:		// Bit recessive Error
	case HAL_CAN_ERROR_BD:		// Bit dominant Error
		//printf( "BIT\n" );
		break;
	case HAL_CAN_ERROR_CRC:		// CRC Error
		//printf( "CRC\n" );
		break;
	default:	// HAL_CAN_ERROR_NONE
		//printf( "%x\n", errno );
		break;
	}
}
*/
