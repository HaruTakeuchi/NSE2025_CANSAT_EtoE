// PWMを使用してモータの速度制御を行うためのC++ソースファイル
//Developed by Hiro
//07/10/2025

#include "MotorRegulation.h"
#include <Arduino.h>

MotorRegulation::MotorRegulation(uint8_t r_ain1, uint8_t r_ain2, uint8_t l_ain1, uint8_t l_ain2) {
    _R_Ain1 = r_ain1;
    _R_Ain2 = r_ain2;
    _L_Ain1 = l_ain1;
    _L_Ain2 = l_ain2;

    //ピンを出力モードに設定
    pinMode(_R_Ain1, OUTPUT);
    pinMode(_R_Ain2, OUTPUT);
    pinMode(_L_Ain1, OUTPUT);
    pinMode(_L_Ain2, OUTPUT);

    //全てのモータを停止（初期状態）
    analogWrite(_R_Ain1, 0);
    analogWrite(_R_Ain2, 0);
    analogWrite(_L_Ain1, 0);
    analogWrite(_L_Ain2, 0);
}

void MotorRegulation::rotateRight(int8_t mode, uint8_t speed) {
    switch (mode) {
        case 0: // 停止
            analogWrite(_R_Ain1, 0);
            analogWrite(_R_Ain2, 0);
            break;

        case 1: // 前進
            analogWrite(_R_Ain1, speed);
            analogWrite(_R_Ain2, 0);
            break;

        case 2: // 逆転
            analogWrite(_R_Ain1, 0);
            analogWrite(_R_Ain2, speed);
            break;

        case 3: // 両方LOW（ブレーキ）
            analogWrite(_R_Ain1, 255 - speed);
            analogWrite(_R_Ain2, 255 - speed);

        default: // defaultは停止
            stopRightmotor();
            break;
    }

}

void MotorRegulation::rotateLeft(int8_t mode, uint8_t speed) {
    switch (mode) {
        case 0: // 停止
            analogWrite(_L_Ain1, 0);
            analogWrite(_L_Ain2, 0);
            break;
           
        case 1: // 前進
            analogWrite(_L_Ain1, speed);    
            analogWrite(_L_Ain2, 0);
            break;
            
        case 2: // 逆転
            analogWrite(_L_Ain1, 0);
            analogWrite(_L_Ain2, speed);
            break;

        case 3: // 両方LOW (ブレーキ)
            analogWrite(_L_Ain1, 255 - speed);
            analogWrite(_L_Ain2, 255 - speed);
            break;

        default: // defaultは停止
            stopLeftmotor();
            break;
    }   
}

void MotorRegulation::stopRightmotor() {
    analogWrite(_R_Ain1, 0);
    analogWrite(_R_Ain2, 0);
}

void MotorRegulation::stopLeftmotor() {
    analogWrite(_L_Ain1, 0);
    analogWrite(_L_Ain2, 0);
}
