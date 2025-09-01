#include "main.h"
#include <cstring>
#include <cstdio>

#include "CANFD.hpp"
#include "FullColorLED.hpp"
#include <PID.hpp>


/*PID pid1;
PID pid2;
PID pid3;
PID pid4;*/

int32_t get_encoder1( void )
{
	  uint16_t enc_buff = LPTIM1->CNT;
	  printf("%d\n", LPTIM1->CNT);
	  LPTIM1->CNT = 0;
	  if (enc_buff > 32767)
	  {
	    return (int16_t)enc_buff * -1;
	  }
	  else
	  {
	    return (int16_t)enc_buff;
	  }
}

int32_t get_encoder2( void )
{
	  uint16_t enc_buff = TIM8->CNT;
	  TIM8->CNT = 0;
	  if (enc_buff > 32767)
	  {
	    return (int16_t)enc_buff * -1;
	  }
	  else
	  {
	    return (int16_t)enc_buff;
	  }
}

int32_t get_encoder3( void )
{
	  uint16_t enc_buff = TIM1->CNT;
	  TIM1->CNT = 0;
	  if (enc_buff > 32767)
	  {
	    return (int16_t)enc_buff * -1;
	  }
	  else
	  {
	    return (int16_t)enc_buff;
	  }
}

int32_t get_encoder4( void )
{
	  uint16_t enc_buff = TIM8->CNT;
	  TIM8->CNT = 0;
	  if (enc_buff > 32767)
	  {
	    return (int16_t)enc_buff * -1;
	  }
	  else
	  {
	    return (int16_t)enc_buff;
	  }
}

CANFD* canfd;

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
	canfd->init();

	CANFD_Frame test;
	test.id=10;
	test.size = 32;
	for (int i = 0; i < 32; i++) test.data[i] = i;
	canfd->tx(test);

  	/*pid1.set_limit(10, 900);
	pid1.set_gain(5,3,0.2);
	//pid1.set_gain(0.3,2,0.2);
	pid2.set_limit(10, 900);
	//pid2.set_gain(5,3,0.2);
	pid2.set_gain(0.3,2,0.2);

	pid3.set_limit(10, 900);
	 //pid1.set_gain(5,3,0.2);
	pid3.set_gain(0.3,2,0.2);
	pid4.set_limit(10, 900);
	//pid2.set_gain(5,3,0.2);
	pid4.set_gain(0.3,2,0.2);*/

  	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

	__HAL_LPTIM_START_CONTINUOUS(&hlptim1);
	HAL_LPTIM_Encoder_Start(&hlptim1, 100000);
	
	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);

	HAL_GPIO_WritePin(SD_0_GPIO_Port, SD_0_Pin, GPIO_PIN_SET);

	PID pid1{true, 0.1};
	pid1.set_limit(10, 100);
	pid1.set_gain(0.1,0.001, 0.2);;
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 10);
 	 while (1)
  	{
		int32_t encoder = get_encoder2();
		int pwm = (int)pid1.calc(200.0, (float)encoder);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwm);
		osDelay(10);
  	}
}