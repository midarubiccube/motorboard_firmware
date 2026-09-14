#include "Motor.hpp"

#include <cstdlib>
#include <cstdio>

#include <string.h>

#include "main.h"
#include "usart.h"


void Motor::init(){
    HAL_TIM_PWM_Start(tim_, ch_A_);
    HAL_TIM_PWM_Start(tim_, ch_B_);
    stop();
}

void Motor::setTarget(int16_t target){
     this->target = target;
}

void Motor::control(int test){
    if(mode == ControlMode::PWM_Mode){ // PWM mode
        PWMModeControl(test);
    } else if(mode == ControlMode::Encoder_Mode){ // ENCODER mode
        EncoderModeControl(test);
    } else if(mode == ControlMode::Current_Mode){ // CURRENT mode
        // Not implemented
    }
}


void Motor::PWMModeControl(int test){

    
    if(target > 0){
        __HAL_TIM_SET_COMPARE(tim_, ch_A_, target);
        __HAL_TIM_SET_COMPARE(tim_, ch_B_, 0);
    } else {
        __HAL_TIM_SET_COMPARE(tim_, ch_A_, 0);
        __HAL_TIM_SET_COMPARE(tim_, ch_B_, abs(target));
    }
}

void Motor::EncoderModeControl(int test){
    int16_t feedback = 0;
    if(get_encoder_fp_ != nullptr){
        feedback = get_encoder_fp_();
    }

    if(feedback == 0 && target == 0){
        __HAL_TIM_SET_COMPARE(tim_, ch_A_, 0);
        __HAL_TIM_SET_COMPARE(tim_, ch_B_, 0);
        HAL_GPIO_WritePin(SD_port_, SD_pin_, GPIO_PIN_RESET);
        pid_.reset();
        return;
    }
    
     HAL_GPIO_WritePin(SD_port_, SD_pin_, GPIO_PIN_SET);

    int32_t control = pid_.calc(abs(target), feedback);
    if (target < 0){
        __HAL_TIM_SET_COMPARE(tim_, ch_A_, abs(control));
        __HAL_TIM_SET_COMPARE(tim_, ch_B_, 0);
    } else {
        __HAL_TIM_SET_COMPARE(tim_, ch_A_, 0);
        __HAL_TIM_SET_COMPARE(tim_, ch_B_, control);
    }

    /*if (test){
        printf("t:%d f:%d c:%d\r\n", target, feedback,control);
    }*/
}

void Motor::start(){
    HAL_GPIO_WritePin(SD_port_, SD_pin_, GPIO_PIN_SET);
    pid_.reset();
}

void Motor::stop(){
    HAL_GPIO_WritePin(SD_port_, SD_pin_, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(tim_, ch_A_, 0);
    __HAL_TIM_SET_COMPARE(tim_, ch_B_, 0);
    pid_.reset();
}