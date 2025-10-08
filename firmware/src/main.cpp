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

extern osTimerId_t controlTimerHandle;
CANFD *canfd;
std::array<Motor *, 4> motors;

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
	{
		canfd->rx_interrupt_task();
	}
}

extern "C" void StartDefaultTask(void *argument)
{
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
	filter_id.format.to_BoardID = id;
	canfd->set_filter_mask(1, filter_id.id, 0xFF);

	motors[0] = new Motor(&htim2, TIM_CHANNEL_1, TIM_CHANNEL_2, SD_0_GPIO_Port, SD_0_Pin, get_encoder1);
	motors[1] = new Motor(&htim3, TIM_CHANNEL_1, TIM_CHANNEL_2, SD_1_GPIO_Port, SD_1_Pin, get_encoder2);
	motors[2] = new Motor(&htim3, TIM_CHANNEL_3, TIM_CHANNEL_4, SD_2_GPIO_Port, SD_2_Pin, get_encoder3);
	motors[3] = new Motor(&htim2, TIM_CHANNEL_4, TIM_CHANNEL_3, SD_3_GPIO_Port, SD_3_Pin, get_encoder4);

	for (auto m : motors)
	{
		m->init();
		m->setPIDLimit(0, 800);
		m->setPIDGain(0.5, 0.001, 0.001);
		m->start();
	}

	__HAL_LPTIM_START_CONTINUOUS(&hlptim1);
	HAL_LPTIM_Encoder_Start(&hlptim1, 4095);
	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);

	osTimerStart(controlTimerHandle, 10);

	for (;;)
	{
		if (canfd->rx_available())
		{
			CANFD_Frame data;
			canfd->rx(data);
			auto target_msg = reinterpret_cast<MotorBoard_Target *>(data.data);
			for (int i = 0; i < 4; i++)
			{
				motors[i]->setTarget(target_msg->target[i]);
				motors[i]->setMode(target_msg->mode);
			}
		}
		osDelay(5);
	}
}

extern "C" void controlCallback(void *argument)
{
	for (auto m : motors)
		m->control();
}