#pragma once

#include <cstdint>

union ID
{
  struct ID_format {
    int message_type : 4;
    
    int from_id : 6;
    int from_type : 4;

    int to_id : 6;
    int to_type : 4;
  } format;
  uint32_t id; 
};

struct PowerBoard_receive
{
  bool ON_OFF;
  uint8_t LED_R;
  uint8_t LED_G;
  uint8_t LED_B;
  float MAX_current;
  bool LED_FULLColor;
};

struct PowerBoard_send
{
  float Current;
  float output_voltage;
  float battery1_voltage;
  float battery2_voltage;
  bool emengecy_on;
};


struct MotorBoard_receive
{
  uint8_t mode;
  int target;
  float GAIN_P = -1;
  float GAIN_I = -1;
  float GAIN_D = -1;
  float MAX_current;
  bool must_receive;
};

struct MotorBoard_send
{
  uint8_t mode;
  bool target;
  float current;
};


struct Message_format {
  ID id;
  bool is_remote = false;
  union{
    PowerBoard_receive power_rsv;
    PowerBoard_send power_send;
    MotorBoard_receive motor_rsv;
    MotorBoard_send  motor_send;
    char data[32];
  } data;
};