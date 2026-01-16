#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "struct_typedef.h"
#include "can.h"
#include "can_config.h"

#include "GimbalTask.h"
#include "debug.h"

void can_filter_init(void);

int8_t CanSend(CAN_HandleTypeDef *hcan, int8_t *data, uint32_t std_id, CAN_TxHeaderTypeDef *Motor_Send, uint32_t *wait_time);
#endif
