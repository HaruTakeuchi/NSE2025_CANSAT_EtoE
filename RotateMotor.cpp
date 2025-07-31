#include "RotateMotor.h"
#include <Arduino.h>

RotateMotor::RotateMotor(uint8_t r_ain1, uint8_t r_ain2, uint8_t l_ain1, uint8_t l_ain2) {
    _R_Ain1 = r_ain1;
    _R_Ain2 = r_ain2;
    _L_Ain1 = l_ain1;
    _L_Ain2 = l_ain2;

    pinMode(_R_Ain1, OUTPUT);
    pinMode(_R_Ain2, OUTPUT);
    pinMode(_L_Ain1, OUTPUT);
    pinMode(_L_Ain2, OUTPUT);

    digitalWrite(_R_Ain1, HIGH);
    digitalWrite(_R_Ain2, HIGH);
    digitalWrite(_L_Ain1, HIGH);
    digitalWrite(_L_Ain2, HIGH);
}

void RotateMotor::rotateRight(int8_t mode) {
    switch (mode)
    {
    case 0://ブレーキ
        /* mode is 0 */
        digitalWrite(_R_Ain1, HIGH);
        digitalWrite(_R_Ain2, HIGH);
        break;
    
    case 1://前転
        /* mode is 1 */
        digitalWrite(_R_Ain1, HIGH);
        digitalWrite(_R_Ain2, LOW);
        break;

    case 2://後転
        /* mode is 2 */
        digitalWrite(_R_Ain1, LOW);
        digitalWrite(_R_Ain2, HIGH);
        break;

    case 3://空転
        /* mode is 3 */
        digitalWrite(_R_Ain1, LOW);
        digitalWrite(_R_Ain2, LOW);
        break;

    default://ブレーキ
        digitalWrite(_R_Ain1, HIGH);
        digitalWrite(_R_Ain2, HIGH);
        break;
    }
}

void RotateMotor::rotateLeft(int8_t mode) {
    switch (mode)
    {
    case 0://ブレーキ
        /* mode is 0*/
        digitalWrite(_L_Ain1, HIGH);
        digitalWrite(_L_Ain2, HIGH);
        break;
    
    case 1://前転
        /* mode is 1 */
        digitalWrite(_L_Ain1, LOW);
        digitalWrite(_L_Ain2, HIGH);
        break;
        
    case 2://後転
        /* mode is 2 */
        digitalWrite(_L_Ain1, HIGH);
        digitalWrite(_L_Ain2, LOW);
        break;

    case 3://空転
        /* mode is 3 */
        digitalWrite(_L_Ain1, LOW);
        digitalWrite(_L_Ain2, LOW);
        break;

    default://ブレーキ
        digitalWrite(_L_Ain1, HIGH);
        digitalWrite(_L_Ain2, HIGH);
        break;
    }    
}