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
#include <RotateMotor_tb6643kq.h>
#include "MotorRegulation.h"
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include "SD_Record.h"

Adafruit_BME280 bme;
SFE_UBLOX_GNSS myGNSS;

/*並列処理用タスク定義*/
TaskHandle_t _p_task;

/*　↓I2Cの設定*/
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

//GetAccl
float acc[3];

/*標準気圧設定*/
#define SEALEVELPRESSURE_HPA (1013.25);
float BottomPress = 1013.25;
double ThresholdPress = 999999;//頂点検知の閾値の気圧　現地で入力

//モタドラ設定
#define R_Ain1 0
#define R_Ain2 1
#define L_Ain1 2
#define L_Ain2 21

//モータの出力は70％→100％（255・0.7＝178.5）
const uint8_t duty_70 = 180;
const uint8_t Max_Duty = 255;

//drv用
//MotorRegulaationクラスのインスタンスを作成
MotorRegulation Motors(R_Ain1, R_Ain2, L_Ain1, L_Ain2);

//tb6643kq用
/*
RotateMotor RotateMotor(R_Ain1, R_Ain2, L_Ain1, L_Ain2, 50000, 8);
*/


/*タイマー設定*/
unsigned long previous_Millis = 0; //delay書き換え用に使ってます
unsigned long previous_bomb_Millis = 0;
unsigned long bombtime_1 = 40000; /*後で変える*/
unsigned long previous_bomb_Millis_2 = 0;
unsigned long bombtime_2 = 30000; /*後で変える bombtime_2 must be less than bombtime_1*/
unsigned long previous_SD_Millis = 0;
unsigned long current_SD_Millis = 0;

//SDで記録するグローバル変数
uint16_t mission_time_SD = 1;
double LatMe_deg = 33.61020798927673;
double LongMe_deg = 130.26847097534915;
float pressure = 0.0;
//float camera = 0.0;
double distance = 0.0;//phase3のみで記録
double angle = 0.0;//phase3のみで記録
char state[50] = "brk";//タイヤの運動
char data[200]; //SDに書き込む内容

int8_t phase = 3;

/**/
/*　ゴールの緯度経度　*/
double LatG_deg = 0;  //後で変える
double LongG_deg = 0; //後で変える

CalculateDistance* Factors_Distance;
CalculateAngle* Factors_Angle;

/*落下検知設定*/
int8_t IsShocked = 0;

//-------------------------------------------------------------------------------------------
void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);

  /*以下BNO055*/
  if (!bno.begin())
  {
    Serial.println("No BNO055 detected ... Check your wiring or I2C Address");
    while (1);
  }

  bno.setExtCrystalUse(true);

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
  //R_Ain1等のpinの設定をしてください
  /*
  pinMode(R_Ain1, OUTPUT);
  pinMode(R_Ain2, OUTPUT);
  pinMode(L_Ain1, OUTPUT);
  pinMode(L_Ain2, OUTPUT);
  */

  #ifdef REASSIGN_PINS
    SPI.begin(sck, miso, mosi, cs);
    if (!SD.begin(cs)) {
  #else
    if (!SD.begin()) {
  #endif
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

  writeFile(SD, "/EtoE.csv", "mission_time,phase,longitude,latitude,pressure,camera,distance,angle,state \n");//ヘッダ
}


//-------------------------------------------------------------------------------------------
void loop() {
  
  Serial.print("current phase: ");
  Serial.println(phase);
  Serial.print(acc[0]);

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
    if (AveVariance(10) < 1.4 && ComparePress(ThresholdPress, 10) == 0) {
      phase = 1;
    } else {
    }
    break;

  case 1:
    /* phase is 1 */
    /*頂点検知→第一回衝撃→着地検知*/
    if (IsShocked == 0) {
      if (FallDetect(19.6 /*gx2*/, 50, 10, 10) == 1) {
        IsShocked = 1;
      } else {
      }
    } else {
      phase = 2;
    }
    break;
    
  case 2:
    /* phae is 2 */
    /* 分離 */

    /*drv*/
    
    Motors.rotateRight(1, Max_Duty);//マックススピードで分離をする
    Motors.rotateLeft(1, Max_Duty);
    
    //tb6643kq
    /*
    RotateMotor.rotateRight(1, Max_Duty);
    RotateMotor.rotateLeft(1, Max_Duty);
    */
    Serial.println("分離作動開始");

    previous_Millis = millis();
    while(millis() - previous_Millis < 5000){
      RecordCsv();
    } 
    //↑delay(5000);

    //tb6643kq用
    /*
    RotateMotor.rotateRight(0, duty_70);
    RotateMotor.rotateLeft(0, duty_70);
    */

    //drv用
    Motors.rotateRight(0, duty_70);
    Motors.rotateLeft(0, duty_70);

    phase = 3;
    break;

  case 3:
    /* phase is 3 */
    /* ゴールまで移動 */
    /*7/25メモ
      片方のタイヤをマックスパワーで動かすとき160度/s(本番ではしたくない)、また測ってください
      →片方前転片方後転でいいんじゃないか？
      回す時間は最低300msくらいじゃないとトルク的にきつい気がする
      十分長い時間でPWM7割:PWM10割=7:8くらいの走行距離差
      回す時間が短いほど相対的な走行距離の差は大きくなる
      PWMMAXで理論値979mm/sの理論値速さ*/
    distance = Factors_Distance->GetDistance(LatMe_deg, LatG_deg, LongMe_deg, LongG_deg);
    angle = Factors_Angle->GetFactor_Alpha1();
    
    if (distance < 2.5 /*|| カメラでゴール検知*/) {
      //ここをちゃんと書かなきゃ0mゴールはないです！！！！！！！！！！
      phase = 4;

    } else if(distance < 10){ //ゴールからの距離10m以下
        if (angle > 0.174) {//10度以上
          //方向修正

          //↓160度=2.792rad    つまりangle/2.792は回転させる秒数
          //回す時間は最低でも300ms  これ以上短いとトルクが足りない気がする(要検討)
          //回る角度は最大でも350度.つまり2187msの回転が最大
          //↑これ怖すぎ  350度は絶対安定して回せない  代案要検討
          Rotate_Time = constrain(angle/2.792, 300, 2187);

          //drv用
          Motors.rotateRight(0, duty_70);
          Motors.rotateLeft(1, Max_Duty);

          //tb6643kq
          /*
          RotateMotor.rotateRight(0, duty_70);
          RotateMotor.rotateLeft(1, Max_Duty);
          */

          Serial.println("右回転中小周り");

          strcpy(state, "turn_R");
          
          previous_Millis = millis();
          while(millis() - previous_Millis < Rotate_Time){
            RecordCsv();
          }
          //↑delay(Rotate_Time);

        }
        //進む

        //drv
        
        Motors.rotateRight(1, duty_70);
        Motors.rotateLeft(1, duty_70);

        //tb6643kq
        /*
        RotateMotor.rotateRight(1, duty_70);
        RotateMotor.rotateLeft(1, duty_70);
        */
       
        Serial.println("前進中小距離");

        strcpy(state, "fwd");

        previous_Millis = millis();  //約1m進む
        while(millis() - previous_Millis < 1200){
          RecordCsv();
        }
        //↑delay(1200)

        //止まる
        //drv
        Motors.rotateRight(0, duty_70);
        Motors.rotateLeft(0, duty_70);        

        //tb6643kq
        /*
        RotateMotor.rotateRight(0, duty_70);
        RotateMotor.rotateLeft(0, duty_70);
        */

        Serial.println("ブレーキ小距離");

        strcpy(state, "brk");

    } else {  //ゴールからの距離10m以上
        if (angle > 1.047) {//60度以上
          //方向修正（モーターライブラリ）

          //↓160度=2.792rad    つまりangle/2.792は回転させる秒数
          //回す時間は最低でも500ms  ゴールとの距離がまだ遠いので安定性重視
          //回る角度は最大でも300度.つまり1875msの回転が最大          
          Rotate_Time = constrain(angle/2.792, 500, 1875);
          
          //drv
          Motors.rotateRight(1, duty_70); 
          Motors.rotateLeft(1, Max_Duty);

          //tb6643kq
          /*
          RotateMotor.rotateRight(0, duty_70);
          RotateMotor.rotateLeft(1, Max_Duty);
          */
          Serial.println("右回転中大回り");

          strcpy(state, "turn_R");
          
          previous_Millis = millis();
          while(millis() - previous_Millis < Rotate_Time){
            RecordCsv();
          }
          //↑delay(Rotate_Time);

        }
        //進む
        //drv
        Motors.rotateRight(1, duty_70);
        Motors.rotateLeft(1, duty_70);
        
        //tb6643kq
        /*
        RotateMotor.rotateRight(1, duty_70);
        RotateMotor.rotateLeft(1, duty_70);
        */
        strcpy(state, "fwd");

        previous_Millis = millis();
        while(millis() - previous_Millis < 12000){
          RecordCsv();
        }
        //↑delay(12000);  //約10m進む

        //止まる
        //drv
        
        Motors.rotateRight(0, duty_70);
        Motors.rotateLeft(0, duty_70);  
        
        //tb6643kq
        /*
        RotateMotor.rotateRight(0, duty_70);
        RotateMotor.rotateLeft(0, duty_70);
        */
        strcpy(state, "brk");
        
    }
  break;

  case 4:
    /* phase is 4 */
    /* ゴール検知 */

  break;

  
  }
}

//-------------------------------------------------------------------------------------------
void ParallelTask(void *param) {
  while(true) {
    RecordCsv();
    switch (phase)
    {
      case 0:
        /* phase is 0 */
        /*スイッチON→頂点検知*/
        previous_bomb_Millis = millis();
        break;

      case 1:
        /* phase is 1*/
        /*頂点検知→第一回衝撃→着地検知*/
        unsigned long currentMillis = millis();
        //時限爆弾1
        if((currentMillis - previous_bomb_Millis) >= bombtime_1) {
        phase = 2;
        }

        if (IsShocked == 1) {
          unsigned long currentMillis_2 = millis();
          //時限爆弾２
          if ((currentMillis_2 - previous_bomb_Millis_2) >= bombtime_2) {
            phase = 2;
          }
        } else {
          previous_bomb_Millis_2 = millis();
        }
        //BottomPress, ListNumは後から数値を変更!要確認!
        if (CompareToBottomPress(BottomPress, 10) == 0) {
          } else {
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
  int8_t OverThresholdCount = 0;

  //FallListにListNum個の加速度を代入
  for (int i = 0; i < ListNum; i++) {
    GetAccl(acc);
    FallList[i] = sqrtf(powf(acc[0],2) + powf(acc[1],2) + powf(acc[2],2));
  }

  //OverThresholdCountがOverThresholdNumを超える、まで繰り返し。
  while (OverThresholdCount < OverThresholdNum) {
    
    //FallListを一つ前にずらして最後の値を更新
    for (int j = 1; j < ListNum; j++) {
      FallList[j-1] = FallList[j];
    }

    GetAccl(acc);
    FallList[ListNum-1] = sqrt(powf(acc[0],2) + powf(acc[1],2) + powf(acc[2],2));
  
    AvgAcc = 0.0;
    for (int k =0; k < ListNum; k++) {
      AvgAcc += FallList[k]; 
    }

    AvgAcc = AvgAcc/ListNum;

    if (AvgAcc < threshold) {  //平均が閾値より小さい
      if(OverThresholdCount >0 ) {  //且つOverThresholdCountが0以下じゃない
        OverThresholdCount -= 1;  //OverThresholdCountを1減らす
      } else {
      }
    } else {  //平均が閾値より大きい
      OverThresholdCount++;  //OverThresholdCountを1増やす
    }
    
    //delay(addtime);
    previous_Millis = millis();
    while(millis() - previous_Millis < addtime){
      RecordCsv();
    }
    Serial.println("ただいまFallDetect中");
  }

  return 1;

}

//FallDetectで使う
void GetAccl(float* Acclist){
  sensors_event_t accelerometerData;
  bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  double x = -1000000, y = -1000000 , z = -1000000;  
  Acclist[0] = accelerometerData.acceleration.x;
  Acclist[1] = accelerometerData.acceleration.y;
  Acclist[2] = accelerometerData.acceleration.z;
}

//いるか？これ
float *GetMag() {
  sensors_event_t magnetometerData;
  bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
  double x = -1000000, y = -1000000 , z = -1000000; //dumb values, easy to spot problem
  float Maglist[3] = {0, 0, 0};
  Maglist[0] = magnetometerData.magnetic.x;
  Maglist[1] = magnetometerData.magnetic.y;
  Maglist[2] = magnetometerData.magnetic.z;
  return Maglist;
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

  if (AvePress <= ThresholdPress) {
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
    }
    //↑delay(Period);
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
  float AveAreaVariance;
  for (int8_t i = 0; i < VarianceNum; i++) {
    AveAreaVariance += PressVariance(100, 10) / VarianceNum;
    /* τ=400で10個データを取る */
  }
  return AveAreaVariance;
}


//ライブラリ作ったから多分いらん
float GetAzimuth() {
  /*
  float AccRead[] = GetAcc();
  float MagRead[] = GetMag();
  float Roll = atan2(AccRead[1] / AccRead[2]);
  float Pitch = atan2(-Accread[0] / sqrt(powf(AccRead[1] ,2) + powf((AccRead[2], 2))));
  float YawNumerator = -((cos(Roll) * MagRead[1]) - (sin(Roll) * MagRead[2]));
  float YawDenominator = (cos(Pitch) * MagRead[0]) + (sin(Pitch) * sin(Pitch) * MagRead[1]) + (sin(Pitch) * cos(Roll) * MagRead[2]);
  float Yaw = atan2(YawNumerator / YawDenominator);

  return Yaw;
  */

  imu::Quaternion quat = bno.getQuat();
  imu::Vector<3> euler = quat.toEuler(); // クォータニオンからオイラー角に変換

  float yaw = euler.z(); // Yaw (方位角)
  //float roll = euler.x(); // Roll
  //float pitch = euler.y(); // Pitch

  return yaw;
}


void RecordCsv(){
  Serial.print("current phase: ");
  Serial.println(phase);

  current_SD_Millis = millis();

  if(current_SD_Millis - previous_SD_Millis > 1000){

    pressure = GetPress();//SD用気圧取得

    if (myGNSS.getPVT() == true) {  //SD用座標取得
      LatMe_deg = myGNSS.getLatitude() / pow(10, 7);
      LongMe_deg = myGNSS.getLongitude() / pow(10, 7);

      Serial.print("lat: ");
      Serial.println(LatMe_deg);
      Serial.print("lon: ");
      Serial.println(LongMe_deg);

    }else {
      Serial.println("座標の取得に失敗しました");  
    }

    if(phase<3){
      snprintf(data, sizeof(data),
      "%d,%d,%lf,%lf,%f,NA,NA,NA,NA\n",
      mission_time_SD, phase, LatMe_deg, LongMe_deg, pressure);

      appendFile(SD, "/EtoE.csv", data);//要素
      mission_time_SD += 1;
      previous_SD_Millis = millis();

    } else{
      
      distance = Factors_Distance->GetDistance(LatMe_deg, LatG_deg, LongMe_deg, LongG_deg);
      angle = Factors_Angle->GetFactor_Alpha1();

      snprintf(data, sizeof(data),
      "%d,%d,%lf,%lf,%f,NA,%lf,%lf,%s\n",
      mission_time_SD, phase, LatMe_deg, LongMe_deg, pressure, distance, angle, state);

      appendFile(SD, "/EtoE.csv", data);//要素
      mission_time_SD += 1;
      previous_SD_Millis = millis();

    }
  }
}

/*
対地用
行数はver.2のメインでの位置に準拠
1.240行目  OverThresholdCountの初期値0が定義されていなかったので定義した
2.247行目  247行目のwhile内でaccllistが更新されていないのでaccllist = GetAccl();を追加した
3.247行目　247行目のwhile内でAvgAccに最初の代入&再代入が行われていないので255行目の繰り返しの前にAvgAcc=0.0;を追加した
4.305行目　AvePress=0.0で初期化した
5.328行目　AvePress=0.0で初期化した
6.354行目　Variance=0.0で初期化した

*/