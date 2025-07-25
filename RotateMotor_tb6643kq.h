/*
<PWMの説明>
・freq(周波数): PWMの周期。高いほどモータの回転は滑らかになるが消費電力が増える。
　また周波数が小さいとモタドラに流れる電流が非常に大きくなるらしいです。
・resolution(分解能): 例えば分解能8の時、PWMを2^8段階(0~255)で調節できる。(8で十分だと思う)
・Duty cycle(デューティ比): PWM1周期のうちのHighの時間の割合。resolutionの範囲から指定してください。
*/
#ifndef RotateMotor_h
#define RotateMotor_h
#pragma once

#include <Arduino.h>


class RotateMotor {
    public:
        RotateMotor(uint8_t r_ain1, uint8_t r_ain2, uint8_t l_ain1, uint8_t l_ain2,    //ピン配置
                    uint16_t freq = 5000, uint8_t resolution = 8);    //周波数, 分解能

        void rotateRight(int8_t mode, uint16_t duty_cycle);
        void rotateLeft(int8_t mode, uint16_t duty_cycle);

    private:
    uint8_t _R_Ain1;
    uint8_t _R_Ain2;
    uint8_t _L_Ain1;
    uint8_t _L_Ain2;
    uint16_t _freq;
    uint8_t _resolution;
    uint16_t _maxDutyCycle;
};

#endif
