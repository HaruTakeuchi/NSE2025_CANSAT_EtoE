// BNO055_Heading.h
// BNO055方位角取得ライブラリ ヘッダファイル

#ifndef BNO055_HEADING_H
#define BNO055_HEADING_H

#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

class BNO055_Heading {
public:
  // コンストラクタ
  BNO055_Heading(Adafruit_BNO055* BNO);

  // --- パブリックメソッド (メインプログラムから呼び出す関数) ---
  bool begin();
  void update();

  // --- ゲッターメソッド (計算結果を取得する関数) ---
  float getYaw();
  float getPitch();
  float getRoll();
  uint8_t getSysCal();
  uint8_t getGyrCal();
  uint8_t getAccCal();
  uint8_t getMagCal();

private:
  // --- プライベートメンバー変数 ---
  Adafruit_BNO055* bno;
  imu::Quaternion current_orientation;
  unsigned long last_time;
  float yaw_deg, pitch_deg, roll_deg;
  uint8_t cal_sys, cal_gyr, cal_acc, cal_mag;

  // --- プライベートメソッド ---
  void calculateEulerAngles();
  void updateCalibrationStatus();
  void write8(byte reg, byte value);
  byte read8(byte reg);
};

#endif // BNO055_HEADING_H
