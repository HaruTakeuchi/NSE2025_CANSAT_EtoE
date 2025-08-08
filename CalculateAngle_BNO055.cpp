// BNO055_Heading.cpp
// BNO055方位角取得ライブラリ ソースファイル

#include "CalculateAngle_BNO055.h"
#include <Wire.h>

// コンストラクタ
BNO055_Heading::BNO055_Heading(Adafruit_BNO055* BNO)  {
  // 初期化リストでbnoオブジェクトを初期化
  this->bno = BNO;
}

// センサーの初期化
bool BNO055_Heading::begin() {
  /*if (!bno.begin()) {
    return false;
  }
  delay(1000);*/

  if (read8(0x00) != 0xA0) {
    return false;
  }

  write8(0x3D, 0x0C); // NDOF_MODE
  delay(30);

  sensors_event_t mevent;
  bno->getEvent(&mevent, Adafruit_BNO055::VECTOR_MAGNETOMETER);
  double initial_yaw_rad = 0;


  current_orientation = imu::Quaternion(
    cos(initial_yaw_rad / 2.0), 0.0, 0.0, sin(initial_yaw_rad / 2.0)
  );

  write8(0x3D, 0x0C); // GYROONLY_MODE
  delay(30);

  last_time = micros();
  return true;
}

// センサー値の更新
void BNO055_Heading::update() {
  unsigned long current_time = micros();
  double delta_time_s = (current_time - last_time) / 1000000.0;
  last_time = current_time;

  sensors_event_t gevent;
  bno->getEvent(&gevent, Adafruit_BNO055::VECTOR_GYROSCOPE);
  float gx_rads = gevent.gyro.x;
  float gy_rads = gevent.gyro.y;
  float gz_rads = gevent.gyro.z;

  imu::Quaternion delta_q(1.0, gx_rads * delta_time_s / 2.0, gy_rads * delta_time_s / 2.0, gz_rads * delta_time_s / 2.0);

  current_orientation = current_orientation * delta_q;
  current_orientation.normalize();

  calculateEulerAngles();
  updateCalibrationStatus();
}

// --- ゲッターメソッドの定義 ---
float BNO055_Heading::getYaw()   { return yaw_deg; }
float BNO055_Heading::getPitch() { return pitch_deg; }
float BNO055_Heading::getRoll()  { return roll_deg; }
uint8_t BNO055_Heading::getSysCal() { return cal_sys; }
uint8_t BNO055_Heading::getGyrCal() { return cal_gyr; }
uint8_t BNO055_Heading::getAccCal() { return cal_acc; }
uint8_t BNO055_Heading::getMagCal() { return cal_mag; }

// --- プライベートメソッドの定義 ---
void BNO055_Heading::calculateEulerAngles() {
  double w = current_orientation.w();
  double x = current_orientation.x();
  double y = current_orientation.y();
  double z = current_orientation.z();

  double sinr_cosp = 2.0 * (w * x + y * z);
  double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
  roll_deg = atan2(sinr_cosp, cosr_cosp) * (180.0 / PI);

  double sinp = 2.0 * (w * y - z * x);
  if (abs(sinp) >= 1)
    pitch_deg = copysign(90.0, sinp);
  else
    pitch_deg = asin(sinp) * (180.0 / PI);

  double siny_cosp = 2.0 * (w * z + x * y);
  double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
  yaw_deg = atan2(siny_cosp, cosy_cosp) * (180.0 / PI);
  
  if (yaw_deg < 0) {
    yaw_deg += 360.0;
  }
}

void BNO055_Heading::updateCalibrationStatus() {
  uint8_t status = read8(0x35);
  cal_sys = (status >> 6) & 0x0C;
  cal_gyr = (status >> 4) & 0x0C;
  cal_acc = (status >> 2) & 0x0C;
  cal_mag = status & 0x0C;
}

void BNO055_Heading::write8(byte reg, byte value) {
  Wire.beginTransmission(0x28);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

byte BNO055_Heading::read8(byte reg) {
  Wire.beginTransmission(0x28);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((byte)0x28, (byte)1);
  return Wire.read();
}
