#include <cstdint>
#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BME280.h>
#include <SparkFun_u-blox_GNSS_v3.h>
#include <Adafruit_Sensor.h>
#include <utility/imumaths.h>
#include "CalculateDistance.h"
#include "CalculateAngle.h"
#include <Arduino.h>
//#include <RotateMotor_tb6643kq.h>
#include <RotateMotor.h>
#include "MotorRegulation.h"
#include "CalculateAngle_BNO055.h"
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include "SD_Record.h"

Adafruit_BME280 bme;
SFE_UBLOX_GNSS myGNSS;

// GNSSをhigh performance modeにするやつ
uint8_t conf_ublox_high_performance_mode[] = {0xB5, 0x62, 0x06, 0x41, 0x10, 0x00, 0x03, 0x00, 0x04, 0x1F, 0x54, 0x5E, 0x79, 0xBF, 0x28, 0xEF, 0x12, 0x05, 0xFD, 0xFF, 0xFF, 0xFF, 0x8F, 0x0D, 0xB5, 0x62, 0x06, 0x41, 0x1C,
0x00, 0x04, 0x01, 0xA4, 0x10, 0xBD, 0x34, 0xF9, 0x12, 0x28, 0xEF, 0x12, 0x05, 0x05, 0x00, 0xA4, 0x40, 0x00, 0xB0, 0x71, 0x0B, 0x0A, 0x00, 0xA4, 0x40, 0x00, 0xD8, 0xB8, 0x05, 0xDE, 0xAE};
// QZSS L1S有効化&SLAS有効化
uint8_t conf_ublox_slas_mode[] = {0xB5, 0x62, 0x06, 0x8A, 0x0E, 0x00, 0x00, 0x01, 0x00, 0x00, 0x14, 0x00, 0x31, 0x10, 0x01, 0x05, 0x00, 0x37, 0x10, 0x01, 0x42, 0x28};

// gnssモジュールのi2cアドレス
uint8_t gnssAddr = 0x42;

/*並列処理用タスク定義*/
TaskHandle_t _p_task;

/*　↓I2Cの設定*/
Adafruit_BNO055 BNO055;

//bnoの方位角取得ライブラリのインスタンス化
BNO055_Heading headingSensor(&BNO055);

//GetAccl
float acc[3];

/*標準気圧設定*/
#define SEALEVELPRESSURE_HPA (1013.25)
float BottomPress = 1013.25; //地上気圧
double ThresholdPress = 999;//頂点検知の気圧　現地で入力

//モタドラ設定
#define R_Ain1 0
#define R_Ain2 1
#define L_Ain1 2
#define L_Ain2 21

//SD用SPI設定
#define sck 19
#define miso 20
#define mosi 18
#define cs 17

//モータの出力は70％→100％（255・0.7＝178.5）
const uint8_t duty_70 = 180;
const uint8_t Max_Duty = 255;

//drv用
//MotorRegulaationクラスのインスタンスを作成
//MotorRegulation Motors(R_Ain1, R_Ain2, L_Ain1, L_Ain2);

//tb6643kq用
/*
RotateMotor RotateMotor(R_Ain1, R_Ain2, L_Ain1, L_Ain2, 50000, 8);
*/
//degitalWrite
RotateMotor RotateMotor(R_Ain1, R_Ain2, L_Ain1, L_Ain2);


/*タイマー設定*/
unsigned long previous_Millis = 0; //delay書き換え用に使ってます
unsigned long peak_detect_Millis = 0;
unsigned long bombtime_1 = 240000; //頂点検知してから分離するまでの時限爆弾(4分)
unsigned long shock_detect_Millis = 0;
unsigned long bombtime_2 = 30000; //開傘検知してから分離するまでの時限爆弾
unsigned long previous_SD_Millis = 0;

//SDで記録するグローバル変数
uint16_t mission_time_SD = 1;
double LatMe_deg = 0;
double LongMe_deg = 0;
float pressure = 0.0;
//float camera = 0.0;
double distance = 0.0;//SD用phase3のみで記録
double angle = 0.0;//SD用phase3のみで記録
char state[50] = "brk";//タイヤの運動
char data[200]; //SDに書き込む内容

//北からゴールの角度を度数法に直したもの
float degree_alpha1 = 0.0;

//北からCanSatの向き(度数法)
float myYaw = 0.0;

//初期phase
int8_t phase = 3; //!!!仮！


/*　ゴールの緯度経度　*/
double LatG_deg = 33.5947198;  //後で変えて！！！
double LongG_deg = 130.2181638; //後で変えて！！！

CalculateDistance* Factors_Distance;
CalculateAngle* Factors_Angle;

/*落下検知設定*/
int8_t IsShocked = 0;

//-------------------------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  Wire.begin(); // I2C通信を開始

  /*以下BNO055*/
  if (BNO055.begin()) {
    Serial.println("Sensor initialized successfully.");
  } else {
    Serial.println("Failed to initialize BNO055. Check connections.");
    while (1); // 失敗した場合はここで停止
  }

  if (headingSensor.begin()) {
    Serial.println("Sensor initialized successfully.");
  } else {
    Serial.println("Failed to initialize BNO055. Check connections.");
    while (1); // 失敗した場合はここで停止
  }

  BNO055.setExtCrystalUse(true);

  /*以下bme280*/
  if (!bme.begin(0x76, &Wire)) {
    Serial.println("BME280が見つかりません。接続を確認してください。");
    while (1);
  }
  Serial.println("BME280 初期化成功");

  /*以下sam-m10q*/
  while (myGNSS.begin() == false) //Connect to the u-blox module using Wire port
  {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Retrying..."));
    delay(1000);
  }

  myGNSS.setI2COutput(COM_TYPE_UBX);

  //GNSSを高精度モードに変更
  change_gnss_working_mode();

  /*並列処理用タスク設定*/
  xTaskCreatePinnedToCore(
    ParallelTask,   // タスク関数へのポインタ(スレッドで実行させたい関数を設定)
    "ParallelTask", // タスク名(最大16文字まで)
    4096,          // スタックサイズ(Byte)
    NULL,          // タスクのパラメータのポインタ
    1,             // タスクの優先順位(0:低 - 25:高)
    &_p_task,      // タスクのHandleへのポインタ
    0              // 利用するCPUコア(0か1を指定できるがXIAO ESP32C3は1つのCPUしかないので0となる)
  );

  LatMe_deg = myGNSS.getLatitude() / pow(10, 7);
  LongMe_deg = myGNSS.getLongitude() / pow(10, 7);

  Factors_Distance = new CalculateDistance(LatMe_deg, LatG_deg, LongMe_deg, LongG_deg);
  Factors_Angle = new CalculateAngle(LatMe_deg, LatG_deg, LongMe_deg, LongG_deg);



  /*モタドラ*/
  //tb6643kq用
  //degitalWrite用
  //R_Ain1等のpinの設定をしてください

  pinMode(R_Ain1, OUTPUT);
  pinMode(R_Ain2, OUTPUT);
  pinMode(L_Ain1, OUTPUT);
  pinMode(L_Ain2, OUTPUT);

  //SPI(SD用)の初期化
  SPI.begin(sck, miso, mosi, cs);
}

//gnss高精度モード
void change_gnss_working_mode() {
  Wire.beginTransmission(gnssAddr);
  Wire.write(conf_ublox_high_performance_mode, 60);
  delay(500);
  Wire.write(conf_ublox_slas_mode, 22);
  delay(500);
  myGNSS.setDynamicModel(DYN_MODEL_STATIONARY);
}
//---------------------------------------------------------------------------------------------
void loop() {
  Serial.print("current phase: ");
  Serial.println(phase);

  int Rotate_Time = 0;//モータを回転させる時間
  
  /*
  phase0 : スイッチON→頂点検知
  phase1 : 頂点検知→第一回衝撃→着地検知
  phase2 : 着地検知→分離機構作動
  phase3 : 分離機構作動→ゴール検知
  phase4 : ゴール検知
  */
  switch (phase)
    {
  case 0:
    /* phase is 0 */
    /*スイッチON→頂点検知*/
    //AveVarianceは後から数値を変更!要確認!
    //ThresholdPress,ListNumも後から数値を変更!要確認!
    if (AveVariance(10) < 1.4 && ComparePress(ThresholdPress, 10) == 0) {//気圧が閾値より低い
      phase = 1;
    }
    break;

  case 1:
    /* phase is 1 */
    /*頂点検知→第一回衝撃→着地検知*/
    if(IsShocked == 0) {
      if (FallDetect(39.2 /*gx4*/, 50, 10, 10) == 1) {//多分FallDetectは検知されない。空中分解が怖いので。
        IsShocked = 1;
        shock_detect_Millis = millis();
      }
    }
    break;

  case 2:
    /* phae is 2 */
    /* 分離 */

    //drv
    /*
    Motors.rotateRight(1, Max_Duty);//マックススピードで分離をする
    Motors.rotateLeft(1, Max_Duty);
    */

    //tb6643kq
    /*
    RotateMotor.rotateRight(1, Max_Duty);
    RotateMotor.rotateLeft(1, Max_Duty);
    */

    //degitalWrite
    RotateMotor.rotateRight(1);
    RotateMotor.rotateLeft(1);

    Serial.println("分離作動開始");

    previous_Millis = millis();
    while(millis() - previous_Millis < 5000){
      RecordCsv();
      headingSensor.update();
      myYaw = headingSensor.getYaw();
    }

    //tb6643kq用
    /*
    RotateMotor.rotateRight(0, duty_70);
    RotateMotor.rotateLeft(0, duty_70);
    */
    //drv用
    /*
    Motors.rotateRight(0, duty_70);
    Motors.rotateLeft(0, duty_70);
    */
    //degitalWrite
    RotateMotor.rotateRight(0);
    RotateMotor.rotateLeft(0);

    phase = 3;
    break;

  case 3:
    /* phase is 3 */
    /* ゴールまで移動 */
    /*7/25メモ
      回す時間は最低300msくらいじゃないとトルク的にきつい気がする
      十分長い時間でPWM7割:PWM10割=7:8くらいの走行距離差
      回す時間が短いほど相対的な走行距離の差は大きくなる
      PWMMAXで理論値979mm/sの理論値速さ*/
    if (myGNSS.getPVT() == true) {
      LatMe_deg = myGNSS.getLatitude() / pow(10, 7);
      LongMe_deg = myGNSS.getLongitude() / pow(10, 7);
    }

    headingSensor.update();
    myYaw = headingSensor.getYaw();
    Factors_Distance->updateLocation(LatMe_deg, LongMe_deg);
    distance = Factors_Distance->GetDistance(LatMe_deg, LatG_deg, LongMe_deg, LongG_deg);
    Factors_Angle->Coordinates(LatMe_deg, LatG_deg, LongMe_deg, LongG_deg);
    degree_alpha1 = Factors_Angle->GetFactor_Alpha1();
    degree_alpha1 = degree_alpha1 * 180 / PI;
    angle = degree_alpha1 - myYaw;

    Serial.print("LatMe_deg= ");
    Serial.print(LatMe_deg , 7);
    Serial.println();
    Serial.print("LongMe_deg= ");
    Serial.print(LongMe_deg , 7);
    Serial.println();
    Serial.print("distance= ");
    Serial.print(distance , 10);
    Serial.println();
    Serial.print("degree_alpha1=");
    Serial.print(degree_alpha1 , 7);
    Serial.println();
    Serial.print("myYaw=");
    Serial.print(myYaw , 7);
    Serial.println();
    Serial.print("angle=");
    Serial.println(angle , 7);
    Serial.println();

    if (distance < 5 /*|| カメラでゴール検知*/) {
      //ここをちゃんと書かなきゃ0mゴールはないです！！！！！！！！！！
      phase = 4;

    } else if(distance < 10){ //ゴールからの距離10m以下
      if (fabs(angle) > 10) {//10度以上
      //方向修正

      //↓160度=2.792rad    つまりangle/2.792は回転させる秒数
      //(回す時間は最低でも300ms  これ以上短いとトルクが足りない気がする(要検討))
      //回る角度は最大でも350度.つまり2187msの回転が最大

      //両輪回転で620度/s
      Rotate_Time = constrain(fabs(angle)*1000/620/9, fabs(angle)*1000/620/9, 1000);

      /*右回転//drv用
      Motors.rotateRight(0, duty_70);
      Motors.rotateLeft(2, Max_Duty);
      //tb6643kq
      RotateMotor.rotateRight(0, duty_70);
      RotateMotor.rotateLeft(1, Max_Duty);
      *///dwgitalWrite
      if(angle >= 0){
        RotateMotor.rotateRight(2);
        RotateMotor.rotateLeft(1);
        strcpy(state, "turn_R");
      } else{
        RotateMotor.rotateRight(1);
        RotateMotor.rotateLeft(2);
        strcpy(state, "turn_L");
      }
      previous_Millis = millis();
      while(millis() - previous_Millis < Rotate_Time){
        RecordCsv();
        headingSensor.update();
        myYaw = headingSensor.getYaw();
      }

      /*8秒止まる//drv
      Motors.rotateRight(0, duty_70);
      Motors.rotateLeft(0, duty_70);
      //tb6643kq
      RotateMotor.rotateRight(0, duty_70);
      RotateMotor.rotateLeft(0, duty_70);
      *///degitalWrite
      RotateMotor.rotateRight(0);
      RotateMotor.rotateLeft(0);
      strcpy(state, "brk");
      previous_Millis = millis();
      while(millis() - previous_Millis < 8000){
        RecordCsv();
        headingSensor.update();
        myYaw = headingSensor.getYaw();
      }

      } else{
        /*1m進む//drv
        Motors.rotateRight(2, duty_70);
        Motors.rotateLeft(2, duty_70);
        //tb6643kq
        RotateMotor.rotateRight(1, duty_70);
        RotateMotor.rotateLeft(1, duty_70);
        *///degitalWrite
        RotateMotor.rotateRight(1);
        RotateMotor.rotateLeft(1);
        strcpy(state, "fwd");
        previous_Millis = millis();  //約1m進む
        while(millis() - previous_Millis < 1200){
          RecordCsv();
          headingSensor.update();
          myYaw = headingSensor.getYaw();
        }

        /*8秒止まる//drv
        Motors.rotateRight(0, duty_70);
        Motors.rotateLeft(0, duty_70);
        //tb6643kq
        RotateMotor.rotateRight(0, duty_70);
        RotateMotor.rotateLeft(0, duty_70);
        *///degitalWrite
        RotateMotor.rotateRight(0);
        RotateMotor.rotateLeft(0);
        strcpy(state, "brk");
        previous_Millis = millis();
        while(millis() - previous_Millis < 8000){
          RecordCsv();
          headingSensor.update();
          myYaw = headingSensor.getYaw();
        }
      }

  }else {  //ゴールからの距離10m以上
      if (fabs(angle) > 60) {//60度以上
        //↓160度=2.792rad    つまりangle/2.792は回転させる秒数
        //回す時間は最低でも500ms  ゴールとの距離がまだ遠いので安定性重視
        //回る角度は最大でも300度.つまり1875msの回転が最大

        //両輪回転で620度/s
        Rotate_Time = constrain(fabs(angle)*1000 / 620 / 9, fabs(angle)*1000/620/9, 1000);

        /*右回転//drv
        Motors.rotateRight(0, duty_70);
        Motors.rotateLeft(2, Max_Duty);
        //tb6643kq
        RotateMotor.rotateRight(0, duty_70);
        RotateMotor.rotateLeft(1, Max_Duty);
        *///degitalWrite
        if(angle >= 0){
          RotateMotor.rotateRight(2);
          RotateMotor.rotateLeft(1);
          strcpy(state, "turn_R");
        }else {
          RotateMotor.rotateRight(1);
          RotateMotor.rotateLeft(2);
          strcpy(state, "turn_L");
        }
        previous_Millis = millis();
        while(millis() - previous_Millis < Rotate_Time){
          RecordCsv();
          headingSensor.update();
          myYaw = headingSensor.getYaw();
        }

        /*8秒止まる//drv
        Motors.rotateRight(0, duty_70);
        Motors.rotateLeft(0, duty_70);
        //tb6643kq
        RotateMotor.rotateRight(0, duty_70);
        RotateMotor.rotateLeft(0, duty_70);
        *///degitalWrite
        RotateMotor.rotateRight(0);
        RotateMotor.rotateLeft(0);
        strcpy(state, "brk");
        previous_Millis = millis();
        while(millis() - previous_Millis < 8000){
          RecordCsv();
          headingSensor.update();
          myYaw = headingSensor.getYaw();
        }
      }else {
        /*5m進む//drv
        Motors.rotateRight(2, duty_70);
        Motors.rotateLeft(2, duty_70);
        //tb6643kq
        RotateMotor.rotateRight(1, duty_70);
        RotateMotor.rotateLeft(1, duty_70);
        *///degitalWrite
        RotateMotor.rotateRight(1);
        RotateMotor.rotateLeft(1);
        strcpy(state, "fwd");
        previous_Millis = millis();
        while(millis() - previous_Millis < 6000){
          RecordCsv();
          headingSensor.update();
          myYaw = headingSensor.getYaw();
        }

        //8秒止まる
        /*//drv
        Motors.rotateRight(0, duty_70);
        Motors.rotateLeft(0, duty_70);
        //tb6643kq
        RotateMotor.rotateRight(0, duty_70);
        RotateMotor.rotateLeft(0, duty_70);
        *///degitalWrite
        RotateMotor.rotateRight(0);
        RotateMotor.rotateLeft(0);
        strcpy(state, "brk");
        previous_Millis = millis();
        while(millis() - previous_Millis < 8000){
          RecordCsv();
          headingSensor.update();
          myYaw = headingSensor.getYaw();
        }
      }
    }

  break;

  case 4:
    /* phase is 4 */
    /* ゴール検知 */
        ///degitalWrite
    RotateMotor.rotateRight(0);
    RotateMotor.rotateLeft(0);
    strcpy(state, "brk");
  break;
  }
}


//-------------------------------------------------------------------------------------------
void ParallelTask(void *param) {
  while(true) {
    RecordCsv();
    headingSensor.update();
    myYaw = headingSensor.getYaw();
    switch (phase)
    {
      case 0:
        /* phase is 0 */
        /*スイッチON→頂点検知*/
        peak_detect_Millis = millis();
        break;

      case 1:
        /* phase is 1*/
        /*頂点検知→第一回衝撃→着地検知*/
        //時限爆弾1
        if((millis() - peak_detect_Millis) >= bombtime_1) {
          phase = 2;
        }

        //開傘衝撃検知後の時限爆弾
        if(IsShocked == 1) {
          if ((millis() - shock_detect_Millis) >= bombtime_2) {
            phase = 2;
          }
        }else {
          shock_detect_Millis = millis();
        }

        //BottomPress, ListNumは後から数値を変更!要確認!
        if(CompareToBottomPress(BottomPress, 10) == 0) {
        }else {
          phase = 2;
        }

        break;

      //case 2:
        /* phase is 2 */
        /*　分離 */
      //  break;

      //case 3:
        /* phase is 3 */

      //  break;

      //default:
      //  break;

    }
  }
}

//-------------------------------------------------------------------------------------------
int8_t FallDetect(float threshold, int8_t addtime, int8_t OverThresholdNum, int8_t ListNum) {
  float FallList[ListNum] = {};
  float AvgAcc;
  int8_t OverThresholdCount = 0; //加速度が閾値を超えた回数を記録する変数
  //FallListにListNum個の加速度を代入
  for (int i = 0; i < ListNum; i++) {
    GetAccl(acc); //acc更新
    FallList[i] = sqrtf(powf(acc[0],2) + powf(acc[1],2) + powf(acc[2],2));
  }
  while (OverThresholdCount < OverThresholdNum) {
    //FallListを一つ前にずらす
    for (int j = 1; j < ListNum; j++) {
      FallList[j-1] = FallList[j];
    }
    GetAccl(acc); //acc更新
    FallList[ListNum-1] = sqrt(powf(acc[0],2) + powf(acc[1],2) + powf(acc[2],2)); //FallListの最後の値を更新

    AvgAcc = 0.0; //AvgAcc初期化(0にする)
    //FallListの合計をAvgAccに代入
    for (int k =0; k < ListNum; k++) {
      AvgAcc += FallList[k];
    }
    //AvgAccをListNumで割って平均化
    AvgAcc = AvgAcc/ListNum;

    if (AvgAcc < threshold) {  //平均が閾値より小さい
      if(OverThresholdCount >0 ) {  //且つOverThresholdCountが0以下じゃない
        OverThresholdCount -= 1;  //OverThresholdCountを1減らす
      }
    }else {  //平均が閾値より大きい
      OverThresholdCount++;  //OverThresholdCountを1増やす
    }

    previous_Millis = millis();
    while(millis() - previous_Millis < addtime){
      RecordCsv();
      headingSensor.update();
      myYaw = headingSensor.getYaw();
    }
  }
  return 1;
}

//FallDetectで使う
void GetAccl(float* Acclist){
  sensors_event_t accelerometerData;
  BNO055.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  double x = -1000000, y = -1000000 , z = -1000000;
  Acclist[0] = accelerometerData.acceleration.x;
  Acclist[1] = accelerometerData.acceleration.y;
  Acclist[2] = accelerometerData.acceleration.z;
}


//頂点検知で使う
double ComparePress(double ThresholdPress, int8_t ListNum) {
  //ListNumは後から数値を変更!要確認!
  double Presslist[ListNum] = {};
  double AvePress = 0.0;

  for (int i = 0; i < ListNum; i++) {
    Presslist[i] = GetPress();
  }

  for (int k = 0; k < ListNum; k++) {
    AvePress += Presslist[k]; //
  }

  AvePress = AvePress / ListNum;

  if (AvePress <= ThresholdPress) {//気圧の平均が閾値より低ければ0・高ければ1を返す
    return 0;
  } else {
    return 1;
  }
}

/*古い気圧データを捨てられてなくないか？(6/17)*/
int8_t CompareToBottomPress(float BottomPress, int8_t ListNum) {
  float PressList[ListNum] = {};
  float AvePress = 0.0;

  for (int i = 0; i < ListNum; i++) {
    PressList[i] = GetPress();
  }

  for (int k = 0; k < ListNum; k++) {
    AvePress += PressList[k]; //
  }

  AvePress = AvePress / ListNum;

  if (AvePress <= BottomPress) {
    return 0;
  } else {
    return 1;
  }
}

float GetPress() {
  float Press = bme.readPressure() / 100;
  return Press;
}

float PressVariance(int Period, int Num) {
  float PressList[Num];
  float Variance = 0.0;
  float SquareList[Num - 1];
  float AreaAve = 0.0;

  /* Period[ms]間隔でNum[個]の気圧の値をとってリストにする。　*/
  for (int i = 0; i < Num; i++) {
    PressList[i] = GetPress();
    previous_Millis = millis();
    while(millis() - previous_Millis < Period){
      RecordCsv();
      headingSensor.update();
      myYaw = headingSensor.getYaw();
    }
  }

  /* P-tグラフにおいて、連続する2つの気圧とPeriodが成す四角形の面積を、気圧がNum[個]、四角形Num-1[個]求めてリストにする。　*/
  for (int j = 0; j < Num - 1; j++) {
    SquareList[j] = (PressList[j + 1] + PressList[j]) * Period / 2 / 1000;
  }

  for (int k = 0; k < Num - 1; k++) {
    AreaAve += SquareList[k]/(Num-1);
  }

  for (int i = 0; i < Num-1; i++) {
    Variance += powf((SquareList[i] - AreaAve), 2) / (Num - 1);
  }

  return Variance;
}

float AveVariance(int8_t VarianceNum) {
  float AveAreaVariance = 0.0;
  for (int8_t i = 0; i < VarianceNum; i++) {
    AveAreaVariance += PressVariance(100, 10) / VarianceNum;
    /* τ=400で10個データを取る */
  }
  return AveAreaVariance;
}

uint8_t Record_Start_Flag = 0; //開始から5秒立ったかのフラグ
void RecordCsv(){

  if(Record_Start_Flag == 0){
    if(millis() - previous_SD_Millis > 5000){
      previous_SD_Millis = millis();

      if (!SD.begin(cs)) {
        Serial.println("！SDカードのマウントに失敗しました");
        return;
      }else {
        Serial.println("SDカードのマウントに成功しました");
      }

      uint8_t cardType = SD.cardType();
      if (cardType == CARD_NONE) {
        Serial.println("！SDカードがありません");
        return;
      }
      writeFile(SD, "/EtoE.csv", "mission_time,phase,longitude,latitude,pressure,camera,distance,angle,state,degree_alpha1,myYaw \n");//ヘッダ
      Record_Start_Flag = 1;
    }

  }else {
    if(millis() - previous_SD_Millis > 1000){

      pressure = GetPress();//SD用気圧取得

      if (myGNSS.getPVT() == true) {  //SD用座標取得
        LatMe_deg = myGNSS.getLatitude() / pow(10, 7);
        LongMe_deg = myGNSS.getLongitude() / pow(10, 7);

      }else {
        Serial.println("座標の取得に失敗しました");
      }

      if(phase<3){
        snprintf(data, sizeof(data),
        "%d,%d,%lf,%lf,%f,0,NA,NA,NA\n",
        mission_time_SD, phase, LatMe_deg, LongMe_deg, pressure);

      } else{

/*    headingSensor.update();
    myYaw = headingSensor.getYaw();
    distance = Factors_Distance->GetDistance(LatMe_deg, LatG_deg, LongMe_deg, LongG_deg);
    Factors_Angle->Coordinates(LatMe_deg, LatG_deg, LongMe_deg, LongG_deg);
    degree_alpha1 = Factors_Angle->GetFactor_Alpha1() * 180 / PI;
    angle = degree_alpha1 - myYaw;*/

        snprintf(data, sizeof(data),
        "%d,%d,%lf,%lf,%f,0,%lf,%lf,%s,\n",
        mission_time_SD, phase, LatMe_deg, LongMe_deg, pressure,distance,angle,state);
      }
      //SDに記録
      appendFile(SD, "/EtoE.csv", data);//要素
      mission_time_SD += 1;
      previous_SD_Millis = millis();
    }
  }
}