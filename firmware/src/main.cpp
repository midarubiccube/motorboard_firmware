#include "main.h"
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <array>

#include "CANFD.hpp"
#include "FullColorLED.hpp"
#include "Encoder.hpp"
#include "Motor.hpp"

#include "messageFormat/motorBoard.hpp"

CANFD *canfd;
FullColorLED led{&htim20, TIM_CHANNEL_3};

extern osTimerId_t controlTimerHandle;
std::array<Motor *, 4> motors;
int angle = 1000;

namespace
{
	constexpr uint8_t kSts3215Id = 0;			// サーボID
	constexpr uint8_t kInstructionWrite = 0x03; // WRITE_DATA
	constexpr uint8_t kAddrGoalPositionL = 0x2A;
	constexpr uint8_t kAddrGoalSpeedL = 0x2E; // モード3用速度アドレス

	int last_target_position = 0;
	int last_position = 0;

	void sendSts3215GoalPositionUsart1(uint8_t id, int16_t position)
	{
		// STS3215(Protocol 1.0互換):
		// 0xFF 0xFF ID LEN INST ADDR DATA... CHKSUM
		uint8_t packet[13];
  		packet[0] = 0xFF;  // ヘッダ
  		packet[1] = 0xFF;  // ヘッダ
  		packet[2] = id;    // サーボID
  		packet[3] = 9;     // パケットデータ長
  		packet[4] = 3;     // コマンド（3は書き込み命令）
		packet[5] = 42;    // レジスタ先頭番号
		packet[6] = position & 0xFF; // 位置情報バイト下位
  		packet[7] = (position >> 8) & 0xFF; // 位置情報バイト上位
  		packet[8] = 0x00;  // 時間情報バイト下位
  		packet[9] = 0x00;  // 時間情報バイト上位
  		packet[10] = 0x00; // 速度情報バイト下位
  		packet[11] = 0x00; // 速度情報バイト上位	

		uint8_t sum = 0;
		for (int i = 2; i < 12; ++i)
		{
			sum += packet[i];
		}
		packet[12] = static_cast<uint8_t>(~sum);

 		HAL_UART_Transmit(&huart4, packet, sizeof(packet), 10);
	}

	void sendSts3215GoalSpeedUsart1(uint8_t id, int16_t speed)
	{
		// STS3215(Protocol 1.0互換) モード3用:
		// 0xFF 0xFF ID LEN INST ADDR SPEED_L SPEED_H CHKSUM
		uint8_t packet[9];
		packet[0] = 0xFF;
		packet[1] = 0xFF;
		packet[2] = id;
		packet[3] = 5; // パケットデータ長(INST～CHKSUM直前)
		packet[4] = 3;
		packet[5] = 0x2E;
		packet[6] = speed & 0xFF;
		packet[7] = (speed >> 8) & 0xFF;

		uint8_t sum = 0;
		for (int i = 2; i <= 7; ++i)
		{
			sum += packet[i];
		}
		packet[8] = static_cast<uint8_t>(~sum);

		HAL_UART_Transmit(&huart4, packet, sizeof(packet), 10);
	}

	void STS3215_SetID_Broadcast(uint8_t new_id)
	{
    	uint8_t packet[8];
    	packet[0] = 0xFF;
    	packet[1] = 0xFF;
    	// Broadcast ID
    	packet[2] = 0xFE;
    	// Length = Instruction + Address + Data + Checksum
    	packet[3] = 0x04;
    	// WRITE
    	packet[4] = 0x03;
    	// ID register
    	packet[5] = 0x05;
    	// New ID
    	packet[6] = new_id;
    	// Checksum
    	uint8_t sum = packet[2] +
                  packet[3] +
                  packet[4] +
                  packet[5] +
                  packet[6];
    	packet[7] = ~sum;
    	HAL_UART_Transmit(&huart4, packet, sizeof(packet), 100);
}

	void sendSts3215ModeStepUsart1(uint8_t id)
	{
		// STS3215(Protocol 1.0互換) モード設定:
		// 0xFF 0xFF ID LEN INST ADDR MODE CHKSUM
		uint8_t packet[8];
		packet[0] = 0xFF;      // ヘッダ
		packet[1] = 0xFF;      // ヘッダ
		packet[2] = id;        // サーボID
		packet[3] = 0x04;      // パケットデータ長(INST～CHKSUM直前)
		packet[4] = 0x03;      // コマンド（3は書き込み命令）
		packet[5] = 33;      // レジスタアドレス(Operating Mode)
		packet[6] = 0;      // モード値

		uint8_t sum = 0;
		for (int i = 2; i < 7; ++i)
		{
			sum += packet[i];
		}
		packet[7] = static_cast<uint8_t>(~sum);

		HAL_UART_Transmit(&huart4, packet, sizeof(packet), 10);
	}
} // namespace

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
	{
		canfd->rx_interrupt_task();
	}
}


extern "C" void StartDefaultTask(void *argument)
{
	led.start();
	led.set_rgb(0, 255, 0);

	sendSts3215ModeStepUsart1(0);
	sendSts3215GoalPositionUsart1(0,2000);

	uint8_t id = HAL_GPIO_ReadPin(ID0_GPIO_Port, ID0_Pin) |
				 (HAL_GPIO_ReadPin(ID1_GPIO_Port, ID1_Pin) << 1) |
				 (HAL_GPIO_ReadPin(ID2_GPIO_Port, ID2_Pin) << 2) |
				 (HAL_GPIO_ReadPin(ID3_GPIO_Port, ID3_Pin) << 3);

	canfd = new CANFD(&hfdcan1);
	canfd->start();
	ID_Format filter_id;
	filter_id.format.broadcast = true;
	canfd->set_filter_mask(0, filter_id.id, filter_id.id);

	filter_id.id = 0;
	filter_id.format.to_BoardType = Board_Type::MotorBoard;
	filter_id.format.to_BoardID = 1;
	canfd->set_filter_mask(1, filter_id.id, 0xFF);

	motors[0] = new Motor(&htim2, TIM_CHANNEL_1, TIM_CHANNEL_2, SD_0_GPIO_Port, SD_0_Pin, get_encoder1);
	motors[1] = new Motor(&htim3, TIM_CHANNEL_1, TIM_CHANNEL_2, SD_1_GPIO_Port, SD_1_Pin, get_encoder2);
	motors[2] = new Motor(&htim3, TIM_CHANNEL_3, TIM_CHANNEL_4, SD_2_GPIO_Port, SD_2_Pin, get_encoder3);
	motors[3] = new Motor(&htim2, TIM_CHANNEL_4, TIM_CHANNEL_3, SD_3_GPIO_Port, SD_3_Pin, get_encoder3);

	for (auto m : motors)
	{
		m->init();
		m->setPIDLimit(0, 800);
		m->setPIDGain(0.5, 0.001, 0.001);
		m->start();
	}

	/*__HAL_LPTIM_START_CONTINUOUS(&hlptim1);
	HAL_LPTIM_Encoder_Start(&hlptim1, 4095);*/
	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
	// HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);

	osTimerStart(controlTimerHandle, 10);

	for (;;)
	{
		if (canfd->rx_available())
		{
			CANFD_Frame data;
			canfd->rx(data);
			auto target_msg = reinterpret_cast<MotorBoard_Target *>(data.data);
			last_target_position = target_msg->target[1];

			motors[2]->setTarget(target_msg->target[2]);
			motors[2]->setMode(ControlMode::PWM_Mode);
			motors[3]->setTarget(target_msg->target[3]);
			motors[3]->setMode(ControlMode::PWM_Mode);
		}
		osDelay(10);
	}
}

extern "C" void controlCallback(void *argument)
{
	for (int i = 0; i < 4; i++)
	{
		if (i == 1)
		{
			motors[i]->control(1);
		}
		else
		{
			motors[i]->control(0);
		}
	}
}