/*
 * can_utils.h
 *
 *  Created on: May 6, 2025
 *      Author: motii
 */

#ifndef INC_CAN_UTILS_H_
#define INC_CAN_UTILS_H_


void CAN_TX(uint32_t id, uint8_t *txBuf, CAN_HandleTypeDef* can)
{
	static CAN_TxHeaderTypeDef TxHeader;
	TxHeader.StdId = id;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.DLC = 8;
	TxHeader.TransmitGlobalTime = DISABLE;
	static uint32_t TxMailBox;

	if(HAL_CAN_GetTxMailboxesFreeLevel(can) > 0)
	{
		HAL_CAN_AddTxMessage(can, &TxHeader, txBuf, &TxMailBox);
	}
}


#endif /* INC_CAN_UTILS_H_ */
