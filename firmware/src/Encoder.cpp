#include "main.h"

#include <cstdio>

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
	  uint16_t enc_buff = TIM4->CNT;
	  TIM4->CNT = 0;
	  if (enc_buff > 32767)
	  {
	    return (int16_t)enc_buff * -1;
	  }
	  else
	  {
	    return (int16_t)enc_buff;
	  }
}