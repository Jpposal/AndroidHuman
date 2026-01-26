/**
 ******************************************************************************
 * @file    pc_uart.c
 * @brief   serial数据接发
 ******************************************************************************
 * @attention
 ******************************************************************************
 */

#include "pc_serial.h"
#include "arm_atan2_f32.h"
#include "debug.h"
#include "algorithmOfCRC.h"
#include "usbd_cdc_if.h"
#include <string.h>

unsigned char PCbuffer[PC_RECVBUF_SIZE];
unsigned char SendToPC_Buff[PC_SENDBUF_SIZE];

PCRecvData pc_recv_data;
// Legacy compatibility structure retained for ABI compatibility only; not populated by new code.
PCSendData pc_send_data;

void PCSolve(void)
{
	LossUpdate(&global_debugger.pc_receive_debugger, 0.02);
}

// Helper: 发送通用帧 HEAD(0x5A) TYPE LEN DATA CRC16_LE TAIL(0xA5)
static int pc_send_frame(uint8_t type, const void *data, uint8_t data_len)
{
	uint8_t frame[1 + 1 + 1 + 255 + 2 + 1];
	size_t idx = 0;
	frame[idx++] = 0x5A; // HEAD
	frame[idx++] = type;
	frame[idx++] = data_len;
	if (data_len && data) {
		memcpy(&frame[idx], data, data_len);
		idx += data_len;
	}
	// CRC 从 HEAD 到 DATA 末尾
	uint16_t crc = Get_CRC16_Check_Sum(frame, idx, 0xFFFF);
	frame[idx++] = (uint8_t)(crc & 0xFF);
	frame[idx++] = (uint8_t)((crc >> 8) & 0xFF);
	frame[idx++] = 0xA5; // TAIL

	// 使用 USB CDC 发送
	return CDC_Transmit_FS(frame, (uint16_t)idx);
}

// 解析接收到的字节流，支持协议帧 HEAD/TYPE/LEN/DATA/CRC/TAIL
void PCReceive(unsigned char *buf, uint32_t len)
{
	if (!buf || len == 0) return;

	// 优先兼容旧 '!' 协议（固定大小帧）
	if (len >= PC_RECVBUF_SIZE && buf[0] == '!' && Verify_CRC16_Check_Sum(buf, PC_RECVBUF_SIZE)) {
		memcpy(&pc_recv_data, buf, PC_RECVBUF_SIZE);
		PCSolve();
		return;
	}

	// 解析新协议帧
	uint32_t i = 0;
	while (i + 6 <= len) {
		if (buf[i] != 0x5A) { i++; continue; }
		if (i + 6 > len) break; // 需要至少 HEAD TYPE LEN CRC(2) TAIL
		uint8_t type = buf[i+1];
		uint8_t data_len = buf[i+2];
		size_t frame_len = 1 + 1 + 1 + data_len + 2 + 1;
		if (i + frame_len > len) break; // 等待更多字节
		if (buf[i + frame_len - 1] != 0xA5) { i++; continue; }
		// 计算 CRC
		uint16_t rec_crc = (uint16_t)buf[i+3+data_len] | ((uint16_t)buf[i+3+data_len+1] << 8);
		uint16_t calc = Get_CRC16_Check_Sum((uint8_t *)&buf[i], 3 + data_len, 0xFFFF);
		if (rec_crc != calc) { i++; continue; }

		const uint8_t *pdata = &buf[i+3];
		switch (type) {
			case 0x10: // 0x10
				if (data_len == sizeof(ChassisControlData)) {
					ChassisControlData ctrl;
					memcpy(&ctrl, pdata, sizeof(ctrl));
					// 将 ctrl 映射到通用 pc_recv_data（已有上层使用 pitch/yaw 字段）
					pc_recv_data.pitch = ctrl.vx; // 复用 pitch 字段表示前向速度
					pc_recv_data.yaw = ctrl.wz;   // 复用 yaw 字段表示角速度
					pc_recv_data.is_valid = 1;
					pc_recv_data.should_shoot = (ctrl.control_type == 0xFF) ? 1 : 0; // 协议无射击位，保留扩展
					PCSolve();
				}
				break;
			default:
				// 目前只处理控制消息，其它类型可以在上层实现
				break;
		}
		i += frame_len;
	}
}

/**
 * @brief 在这里写发送数据的封装（构造并发送 Type 0x11 的 ChassisStatus）
 */
void SendtoPCPack(unsigned char *buff)
{
	// 构造并发送 ChassisStatus（Type 0x11）
	// 仅发送新的文档结构；旧的兼容包已移除。
	ChassisStatusData st;
	// 若需填入真实速度/位置数据，请告知数据来源，我可以替换下列默认值。
	st.vx = 0.0f;
	st.vy = 0.0f;
	st.wz = 0.0f;
	st.x = 0.0f;
	st.y = 0.0f;
	st.theta = 0.0f;

	pc_send_frame(TYPE_CHASSIS_STATUS, &st, sizeof(st));
}

void SendtoPC(void)
{
	SendtoPCPack(SendToPC_Buff);
	// 旧接口也会通过 CDC 发送兼容包（如果保留）
	// CDC_Transmit_FS(SendToPC_Buff, PC_SENDBUF_SIZE);
}

int PC_SendChassisStatus(const ChassisStatusData *st)
{
	if (!st) return -1;
	return pc_send_frame(TYPE_CHASSIS_STATUS, st, sizeof(ChassisStatusData));
}

int PC_SendBatteryInfo(const BatteryInfoData *bat)
{
	if (!bat) return -1;
	return pc_send_frame(TYPE_BATTERY_INFO, bat, sizeof(BatteryInfoData));
}
