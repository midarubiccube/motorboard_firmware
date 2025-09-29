#include "main.h"
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <array>

#include "CANFD.hpp"
#include "FullColorLED.hpp"
#include "Encoder.hpp"
#include "Motor.hpp"
#include "message.hpp" 

extern osTimerId_t controlTimerHandle;
CANFD* canfd;
std::array<Motor*, 4> motors;

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
  if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) {
    canfd->rx_interrupt_task();
  }
}


extern "C" void StartDefaultTask(void *argument)
{
  //FullColorLED led{&htim15, TIM_CHANNEL_2};
  //led.set_rgb(255, 255, 255);
  //led.start();
  
  	canfd = new CANFD(&hfdcan1);
	canfd->start();
	canfd->set_filter_mask(2097152, 0x1FF0FFFF);

	/*CANFD_Frame test;
	test.id=can_id;
	test.size = 32;
	for (int i = 0; i < 32; i++) test.data[i] = i;
	canfd->tx(test);*/
	motors[0] = new Motor(&htim2, TIM_CHANNEL_1, TIM_CHANNEL_2, SD_0_GPIO_Port, SD_0_Pin, get_encoder1);
	motors[1] = new Motor(&htim3, TIM_CHANNEL_1, TIM_CHANNEL_2, SD_1_GPIO_Port, SD_1_Pin, get_encoder2);
	motors[2] = new Motor(&htim3, TIM_CHANNEL_3, TIM_CHANNEL_4, SD_2_GPIO_Port, SD_2_Pin, get_encoder3);
	motors[3] = new Motor(&htim2, TIM_CHANNEL_4, TIM_CHANNEL_3, SD_3_GPIO_Port, SD_3_Pin, get_encoder4);
	
	for (auto m : motors){
		m->init();
		m->setPIDLimit(0, 800);
		m->setPIDGain(0.5,0.001, 0.001);
		m->setMode(1);
		m->start();
	}

	__HAL_LPTIM_START_CONTINUOUS(&hlptim1);
	HAL_LPTIM_Encoder_Start(&hlptim1, 4095);
	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);

	osTimerStart(controlTimerHandle, 10);

	for(;;)
  	{
    	osDelay(1000);
  	}
}

int target = 0;

extern "C" void controlCallback(void *argument)
{
	
	for (auto m : motors) m->control();
	if (canfd->rx_available())
    {
      	CANFD_Frame data;
      	canfd->rx(data);
		if (data.is_remote) {
			printf("remote\n");
		} else {
      		Message_format msg = {0};
      		memcpy(&msg.data, data.data, 32);
			target = msg.data.motor_rsv.target / 2;
			printf("motor=%d %d\n", (data.id >> 16) & 0xF, target);
			motors[(data.id >> 16) & 0xF]->setTarget(target);
		}
    }
}