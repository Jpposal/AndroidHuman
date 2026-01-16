#include "bsp_can.h"
#include "can_send_config.h"
#include "ChasisController.h"
#include "testTask.h"

/**********************************************************************************************************
 *函 数 名: can_filter_init
 *功能说明: can配置
 *形    参: 无
 *返 回 值: 无
 **********************************************************************************************************/
void can_filter_init(void)
{
	CAN_FilterTypeDef can_filter_st;
	
	// CAN 1 Filter Configuration - ID List Mode for 0x581 and 0x701
	can_filter_st.FilterBank = 0;
	can_filter_st.FilterActivation = ENABLE;
	can_filter_st.FilterMode = CAN_FILTERMODE_IDLIST;
	can_filter_st.FilterScale = CAN_FILTERSCALE_16BIT;
	
	// List of IDs to accept (StdId << 5)
	can_filter_st.FilterIdHigh = 0x581 << 5;
	can_filter_st.FilterIdLow = 0x701 << 5;
	can_filter_st.FilterMaskIdHigh = 0x181 << 5;
	can_filter_st.FilterMaskIdLow = 0x182 << 5; 
	
	can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
	can_filter_st.SlaveStartFilterBank = 14;
	HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);

	// 额外为 CAN1 配置一个 bank，将指定的两个 CAN ID 放入 FIFO1
	can_filter_st.FilterBank = 1;
	can_filter_st.FilterActivation = ENABLE;
	can_filter_st.FilterMode = CAN_FILTERMODE_IDLIST;
	can_filter_st.FilterScale = CAN_FILTERSCALE_16BIT;
	// 接受 0x582 和 0x702（StdId << 5）
	can_filter_st.FilterIdHigh = 0x582 << 5;
	can_filter_st.FilterIdLow  = 0x702 << 5;
	can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO1; // 放到 FIFO1
	can_filter_st.SlaveStartFilterBank = 14;
	HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);

	// CAN 2 Filter Configuration - Accept All (or configure as needed)
	can_filter_st.FilterBank = 14;
	can_filter_st.FilterActivation = ENABLE;
	can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
	can_filter_st.FilterScale = CAN_FILTERSCALE_16BIT;
	can_filter_st.FilterIdHigh =  0x0000 ;
	can_filter_st.FilterIdLow = 0x0000 ;
	can_filter_st.FilterMaskIdHigh = 0x0000;
	can_filter_st.FilterMaskIdLow = 0x0000;
	can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
	can_filter_st.SlaveStartFilterBank = 14;
	HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);

	// Start CAN instances AFTER configuration
	HAL_CAN_Start(&hcan1);
	HAL_CAN_Start(&hcan2);

	// Enable Interrupts
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO1_MSG_PENDING);
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_TX_MAILBOX_EMPTY);
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_TX_MAILBOX_EMPTY);
}

void MotorReceive(CAN_HandleTypeDef *hcan, CAN_RxHeaderTypeDef *rx_header, uint8_t *data)
{
	if (hcan->Instance == DJI_WHEELS_CAN )
	{
        switch (rx_header->StdId)
        {

        //LossUpdate(&global_debugger.gimbal_debugger[0], 0.0015f);
			}
	}

}
/**********************************************************************************************************
 *函 数 名: HAL_CAN_RxFifo0MsgPendingCallback
 *功能说明:FIFO 0邮箱中断回调函数
 *形    参:
 *返 回 值: 无
 **********************************************************************************************************/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef rx_header;
	uint8_t rx_data[8];
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
	__HAL_CAN_CLEAR_FLAG(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

	if (hcan->Instance == CAN1)
	{
		// 传入多个电机实例以便按 COB-ID 匹配处理（left/right）
		extern ZLA_Motor front_drive;
		extern ZLA_Motor rear_drive;
		ZLA_Motor_ProcessRx(&rx_header, rx_data, &front_drive);
		ZLA_Motor_ProcessRx(&rx_header, rx_data, &rear_drive);
	}
	else if (hcan->Instance == CAN2)
	{
		// CAN2 处理逻辑
	}
}
/**********************************************************************************************************
 *函 数 名: HAL_CAN_RxFifo1MsgPendingCallback
 *功能说明:FIFO 1邮箱中断回调函数
 *形    参:
 *返 回 值: 无
 **********************************************************************************************************/
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) // FIFO 1邮箱中断回调函数
{
	CAN_RxHeaderTypeDef rx_header;
	uint8_t rx_data[8];
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rx_header, rx_data);
	__HAL_CAN_CLEAR_FLAG(hcan, CAN_IT_RX_FIFO1_MSG_PENDING);

	if (hcan->Instance == CAN1)
	{
		extern ZLA_Motor front_drive;
		extern ZLA_Motor rear_drive;
		ZLA_Motor_ProcessRx(&rx_header, rx_data, &front_drive);
		ZLA_Motor_ProcessRx(&rx_header, rx_data, &rear_drive);
	}
	else if (hcan->Instance == CAN2)
	{
		// CAN2 处理逻辑
	}
}