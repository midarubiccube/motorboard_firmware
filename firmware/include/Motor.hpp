#pragma once

#include <cstdint>

#include "main.h"
#include "PID.hpp"
#include "messageFormat/motorBoard.hpp"

class Motor{
public:
    typedef int32_t (*getEncoder_fp)();

    Motor(TIM_HandleTypeDef* tim, int ch_A, int ch_B, GPIO_TypeDef* SD_port, uint16_t SD_pin, getEncoder_fp fp) :  
        tim_(tim), pid_(true, 0.01), get_encoder_fp_(fp),
        ch_A_(ch_A), ch_B_(ch_B), SD_port_(SD_port), SD_pin_(SD_pin)
	{
	}

    void init();

    void setTarget(int16_t target);
    void setMode(ControlMode mode) {
        this->mode = mode;
    }

    void control();

    void setPIDGain(float kp, float ki, float kd) {
        pid_.set_gain(kp, ki, kd);
    }
    void setPIDLimit(int16_t min, int16_t max) {
        pid_.set_limit(min, max);
    }
    void stop();
    void start();
    int16_t gettarget() { return target; }
private:
    PID pid_;
    TIM_HandleTypeDef* tim_;
    int ch_A_;
    int ch_B_;
    GPIO_TypeDef* SD_port_;
    uint16_t SD_pin_;
    getEncoder_fp get_encoder_fp_;

    void PWMModeControl();
    void EncoderModeControl();
    void CurrentModeControl();

    int16_t target = 0;
    ControlMode mode; // 0: PWM, 1: ENCODER, 2: CURRENT
};