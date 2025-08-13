/*
 * CalculateAngle.h (改善版)
 * 2点間の距離と方位角を計算するクラス
 * 作成者: 鳥井太智 (2025年7月)
 * 改善者: *** (2025年8月)
 */

#ifndef CalculateAngle_h
#define CalculateAngle_h
#pragma once

#include <Arduino.h>
#include <cmath>

class CalculateAngle {
public:
    // コンストラクタ: 出発点と目標点を設定
    #ifdef ESP32
    CalculateAngle(double myLat_deg, double myLon_deg, double goalLat_deg, double goalLon_deg);
    #endif

    // --- Public API ---
    // 現在地を更新する
    void updateMyLocation(double myLat_deg, double myLon_deg);
    
    // 目標点までの方位角を取得する (0-360°)
    double getAzimuth();

    // 目標点までの距離(m)を取得する
    double getDistance();

private:
    // --- 内部計算用メソッド (旧GetFactor群) ---
    double toRad(double degree) const;
    double getL1() const; // l'
    double getCapitalL() const;
    double getU1() const;
    double getU2() const;
    double getSigma1() const; // Σ'
    double getDelta1() const; // Δ'
    double getX() const;
    double getY() const;
    double getC() const;
    double getSigma(double theta) const;
    double getJ(double theta) const;
    double getK(double theta) const;
    double getGamma(double theta) const; // γ
    double getCapitalGamma(double theta) const; // Γ
    double getZeta(double theta) const; // ζ
    double getA(double theta) const;
    double getB(double theta) const;
    double calculateTheta(); // 内部でθの収束計算を行う

    // --- メンバ変数 ---
    double myLat_rad;     // 現在地の緯度 (ラジアン)
    double myLon_rad;     // 現在地の経度 (ラジアン)
    double goalLat_rad;   // 目標地の緯度 (ラジアン)
    double goalLon_rad;   // 目標地の経度 (ラジアン)

    // 地球に関する定数
    static constexpr double EQUATORIAL_RADIUS = 6378137.0; // a: 赤道半径
    static constexpr double FLATTENING = 1.0 / 298.257222101; // f: 扁平率
};

#endif // CalculateAngle_h