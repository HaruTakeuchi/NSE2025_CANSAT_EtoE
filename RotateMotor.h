
#ifndef RotateMotor_h
#define RotateMotor_h
#pragma once

#include <Arduino.h>

class RotateMotor {
    public:
        RotateMotor(uint8_t r_ain1, uint8_t r_ain2, uint8_t l_ain1, uint8_t l_ain2);
        void rotateRight(int8_t mode);
        void rotateLeft(int8_t mode);

    private:
    uint8_t _R_Ain1;
    uint8_t _R_Ain2;
    uint8_t _L_Ain1;
    uint8_t _L_Ain2;
};

#endif