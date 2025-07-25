/* Created by 鳥井太智 in July 2025 */

#include <cstdint>

#ifndef CalculateAngle_h
#define CalculateAngle_h
#pragma once
#include <cmath>

#include <Arduino.h>

constexpr double EQUATORIAL_RADIUS_A = 6378137.00; /* a */
constexpr double POLAR_RADIUS_A = 6356752.00;
constexpr double f_A = 0.003352859934;


class CalculateAngle {
    public:
        #ifdef ESP32
        CalculateAngle(double LatMe_deg, double LatG_deg, double LongMe_deg, double LongG_deg);
        #else
        #endif
        void Coordinates(double LatMe_deg, double LatG_deg, double LongMe_deg, double LongG_deg);
        double ToRad(double degree);
        double GetFactor_l();
        double GetFactor_l_1();
        double GetFactor_L();
        double GetFactor_L_1();
        double GetFactor_DELTA();
        double GetFactor_SIGMA();
        double GetFactor_u1();
        double GetFactor_u2();
        double GetFactor_SIGMA_1();
        double GetFactor_DELTA_1();
        double GetFactor_Xi();
        double GetFactor_Xi_1();
        double GetFactor_Eta();
        double GetFactor_Eta_1();
        double GetFactor_Alpha();
        double GetFactor_AlphaHalf();
        double GetFactor_Alpha_1();
        double GetFactor_Alpha1_1();
        double GetFactor_Alpha_2();
        double GetFactor_Alpha21_1();
        double GetFactor_Alpha1();
        double GetFactor_x();
        double GetFactor_y();
        double GetFactor_c();
        double GetFactor_g();
        double GetFactor_h();
        double GetFactor_sigma();
        double GetFactor_J();
        double GetFactor_K();
        double GetFactor_gamma();
        double GetFactor_GAMMA(); 
        double GetFactor_zeta();
        double GetFactor_zeta_1();
        double GetFactor_R();
        double GetFactor_q();
        double GetFactor_f1();
        double GetFactor_GammaZero();
        double GetFactor_j();
        double GetFactor_k();
        double GetFactor_d1();
        double GetFactor_d2();
        double GetFactor_A0();
        double GetFactor_B0();
        double GetFactor_Psi();
        double GetFactor_j1();
        double GetFactor_Psi_1();
        double GetFactor_Psi_2();
        double GetFactor_D();
        double GetFactor_E();
        double GetFactor_F();
        double GetFactor_G();
        double SetThetaZero();
        double CalculateTheta();




    private:

        double LatG_rad;
        double LongG_rad;
        double LatMe_rad;
        double LongMe_rad;
        double PreTheta;
};

#endif