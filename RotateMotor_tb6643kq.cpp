#include "RotateMotor_tb6643kq.h"
#include <Arduino.h>

RotateMotor::RotateMotor(uint8_t r_ain1, uint8_t r_ain2, uint8_t l_ain1, uint8_t l_ain2, uint16_t freq, uint8_t resolution) {
    _R_Ain1 = r_ain1;
    _R_Ain2 = r_ain2;
    _L_Ain1 = l_ain1;
    _L_Ain2 = l_ain2;
    _freq = freq;
    _resolution = resolution;
    _maxDutyCycle = (1 << _resolution) - 1;    //2^(resolution)-1: 最大デューティ比

    ledcAttachChannel(_R_Ain1, _freq, _resolution, 0);
    ledcAttachChannel(_R_Ain2, _freq, _resolution, 1);
    ledcAttachChannel(_L_Ain1, _freq, _resolution, 2);
    ledcAttachChannel(_L_Ain2, _freq, _resolution, 3);

}

//右
void RotateMotor::rotateRight(int8_t mode, uint16_t duty_cycle) {
    uint16_t duty = constrain(duty_cycle, 0, _maxDutyCycle);//dutyが範囲内かどうか(未満なら0, 超過ならmaxを返す)
    switch (mode)
    {
    case 0:    //ショートストップ(ブレーキ)
        /* mode is 0 */
        ledcWriteChannel(0, _maxDutyCycle);
        ledcWriteChannel(1, _maxDutyCycle);
        break;                            
    
    case 1:    //正転
        /* mode is 1 */
        ledcWriteChannel(0, 0);
        ledcWriteChannel(1, duty);
        break;

    case 2:    //後転
        /* mode is 2 */
        ledcWriteChannel(0, duty);
        ledcWriteChannel(1, 0);
        break;

    case 3:    //空転
        /* mode is 3 */
        ledcWriteChannel(0, 0);
        ledcWriteChannel(1, 0);
        break;

    default:    //ショートストップ(ブレーキ)
        ledcWriteChannel(0, _maxDutyCycle);
        ledcWriteChannel(1, _maxDutyCycle);
        break;                       
    }
}

//左
void RotateMotor::rotateLeft(int8_t mode, uint16_t duty_cycle) {
    uint16_t duty = constrain(duty_cycle, 0, _maxDutyCycle);//dutyが範囲内かどうか(未満なら0, 超過ならmaxを返す)
    switch (mode)
    {
    case 0:    //ショートストップ(ブレーキ)
        /* mode is 0*/
        ledcWriteChannel(2, _maxDutyCycle);
        ledcWriteChannel(3, _maxDutyCycle);
        break;                            
    
    case 1:    //正転
        /* mode is 1 */
        ledcWriteChannel(2, 0);
        ledcWriteChannel(3, duty);
        break;
        
    case 2:    //後転
        /* mode is 2 */
        ledcWriteChannel(2, duty);
        ledcWriteChannel(3, 0);
        break;

    case 3:    //空転
        /* mode is 3 */
        ledcWriteChannel(2, 0);
        ledcWriteChannel(3, 0);
        break;     

    default:    //ショートストップ(ブレーキ)
        ledcWriteChannel(2, _maxDutyCycle);
        ledcWriteChannel(3, _maxDutyCycle);
        break;                      
    }    
}

