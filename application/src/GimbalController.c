#include "GimbalController.h"
#include "robot_config.h"

extern float cur_pitch;
extern float cur_yaw;

static CAN_TxHeaderTypeDef  Can2BodanSend,CAN2GraphSend;
static uint8_t bodan_send_data[8];
void CAN1_YawSend(short a,short b)
{
    uint32_t send_mail_box;
    Can2BodanSend.IDE = CAN_ID_STD;
    Can2BodanSend.RTR = CAN_RTR_DATA;
    Can2BodanSend.DLC = 0x08;
    Can2BodanSend.StdId = 0x1FF;
    a=LIMIT_MAX_MIN(a,16000,-16000);
	  b =LIMIT_MAX_MIN(b,9500,-9500);
	short q =0,c=0;
	  bodan_send_data[0] = (unsigned char)((b>>8)&0xff);
    bodan_send_data[1] = (unsigned char)(b&0xff);
    bodan_send_data[2] = (unsigned char)((a>>8)&0xff);
    bodan_send_data[3] = (unsigned char)(a&0xff);
    bodan_send_data[4] = (unsigned char)((c>>8)&0xff);
    bodan_send_data[5] = (unsigned char)(c&0xff);
    bodan_send_data[6] = (unsigned char)((b>>8)&0xff);
    bodan_send_data[7] = (unsigned char)(b&0xff);

    HAL_CAN_AddTxMessage(&hcan1, &Can2BodanSend, bodan_send_data, &send_mail_box);
}



static uint8_t Graph_send_data[8];
void Can1PitchSend(short a,short b,short c,short d)
{
	uint32_t send_mail_box;
  CAN2GraphSend.IDE = CAN_ID_STD;
  CAN2GraphSend.RTR = CAN_RTR_DATA;
  CAN2GraphSend.DLC = 0x08;
  CAN2GraphSend.StdId = 0x1FF;
	
	a=LIMIT_MAX_MIN(a,8000,-8000);
	b=LIMIT_MAX_MIN(b,8000,-8000);
	c=LIMIT_MAX_MIN(c,8000,-8000);
	d=LIMIT_MAX_MIN(d,8000,-8000);
	
	
	
	Graph_send_data[0] = (unsigned char)((a>>8)&0xff);
  Graph_send_data[1] = (unsigned char)(a&0xff);
  Graph_send_data[2] = (unsigned char)((b>>8)&0xff);
  Graph_send_data[3] = (unsigned char)(b&0xff);
  Graph_send_data[4] = (unsigned char)((c>>8)&0xff);
  Graph_send_data[5] = (unsigned char)(c&0xff);
  Graph_send_data[6] = (unsigned char)((d>>8)&0xff);
  Graph_send_data[7] = (unsigned char)(d&0xff);
	
	 HAL_CAN_AddTxMessage(&hcan1, &CAN2GraphSend, Graph_send_data, &send_mail_box);
}




void GimbalupdataSensors()
{
    GM6020_Decode(&gimbal_controller.yaw_recv, &gimbal_controller.yaw_info);
    M2006_Decode(&gimbal_controller.pitch_recv, &gimbal_controller.pitch_info, WITH_REDUCTION, 0.70);
    updateGyro();
}

void gimbal_turn(GimbalController *gimbal_controller)
{
		gimbal_controller->control = Gimbal_Yaw_Calculate(gimbal_controller->target_yaw_angle);
}

void execute_gimbal(GimbalController *gimbal_controller)
{
			uint32_t motor_wait;
			uint32_t motor_wait1;
//      GM6020_SendPack(gimbal_controller->gimbal_send_data, C620_STD_ID_5_8, DJI_6020_MOTORS_1 - 0x204, (int16_t)cur_yaw, GM6020_CUR_MODE);
//			CanSend(&hcan1 , gimbal_controller->gimbal_send_data, C620_STD_ID_5_8 ,&motor_tx_header[1], &motor_wait);
      CAN1_YawSend(cur_yaw,cur_pitch);
//	    Can1PitchSend(cur_pitch,0,0,0);
//	    M2006_SendPack(gimbal_controller->gimbal_send_data, C610_STD_ID_1_4, DJI_2006_MOTORS_1 - 0x200, (int16_t)cur_pitch);
//			CanSend(&hcan1 , gimbal_controller->gimbal_send_data, C610_STD_ID_1_4 ,&motor_tx_header[2], &motor_wait1);
}
void gimbal_init(GimbalController *gimbal_controller)
{

		gimbal_controller->target_yaw_angle=2600;
}
