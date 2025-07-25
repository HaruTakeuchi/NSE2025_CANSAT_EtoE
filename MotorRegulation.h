// PWMを使用してモータの速度制御を行うためのヘッダファイル
//Developed by Hiro
//07/10/2025

#ifndef MotorRegulation_h
#define MotorRegulation_h
#pragma once

#include <Arduino.h>

class MotorRegulation {
    public:
        //ピン番号を受け取る
        MotorRegulation(uint8_t r_ain1, uint8_t r_ain2, uint8_t l_ain1, uint8_t l_ain2);
        //modeとデューティーサイクルと時間を引数に追加
        void rotateRight(int8_t mode, uint8_t speed);
        void rotateLeft(int8_t mode, uint8_t speed);
        //モータを停止させる関数を追加
        void stopRightmotor();
        void stopLeftmotor();

    private:
    uint8_t _R_Ain1;
    uint8_t _R_Ain2;
    uint8_t _L_Ain1;
    uint8_t _L_Ain2;

};

#endif