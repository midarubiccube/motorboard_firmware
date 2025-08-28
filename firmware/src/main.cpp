#include "main.h"
#include <cstring>

#include "CANFD.hpp"
#include "FullColorLED.hpp"

CANFD* canfd;

extern "C" void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
  if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) {
	  canfd->rx_interrupt_task();
  }
}

extern "C" void StartDefaultTask(void *argument)
{
  FullColorLED led{&htim15, TIM_CHANNEL_2};
  led.set_rgb(255, 255, 255);
  led.start();
  
  canfd = new CANFD(&hfdcan1);
	canfd->init();

	CANFD_Frame test;
	test.id=10;
	test.size = 32;
	memset(test.data, 0, 64);
	canfd->tx(test);

  while (1)
  {
    
  }
}