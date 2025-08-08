/* Created by 鳥井太智 in June 2025 */

#include <cstdint>

#ifndef CalculateDistance_h
#define CalculateDistance_h
#pragma once
#include <cmath>

#include <Arduino.h>

constexpr double EQUATORIAL_RADIUS_D = 6378137.00; /* a */
constexpr double POLAR_RADIUS_D = 6356752.00;
constexpr double f_D = 0.003352859934;


class CalculateDistance {
    public:
        #ifdef ESP32
        CalculateDistance(double LatMe_deg, double LatG_deg, double LonMe_deg, double LonG_deg);
        #else
        #endif
        void updateLocation(double latMe_deg, double lonMe_deg);
        double ToRad(double degree);
        double GetFactor_l();
        double GetFactor_l_1();
        double GetFactor_L();
        double GetFactor_L_1();
        double GetFactor_SIGMA();
        double GetFactor_u1();
        double GetFactor_u2();
        double GetFactor_SIGMA_1();
        double GetFactor_DELTA_1();
        double GetFactor_Xi();
        double GetFactor_Xi_1();
        double GetFactor_Eta();
        double GetFactor_Eta_1();
        double GetFactor_x();
        double GetFactor_y();
        double GetFactor_c();
        double GetFactor_Epsilon();
        double GetFactor_g();
        double GetFactor_h();
        double GetFactor_sigma();
        double GetFactor_J();
        double GetFactor_K();
        double GetFactor_gamma();
        double GetFactor_GAMMA();
        double GetFactor_zeta();
        double GetFactor_zeta_1();
        double GetFactor_D();
        double GetFactor_E();
        double GetFactor_F();
        double GetFactor_G();
        double GetFactor_n0();
        double GetFactor_A();
        double GetFactor_B();
        double GetDistance(double LatMe_deg, double LatG_deg, double LongMe_deg, double LongG_deg);
        double SetThetaZero();
        double CalculateTheta();
        double GetFactor_R();
        double GetFactor_d1();
        double GetFactor_d2();
        double GetFactor_q();
        double GetFactor_f1();
        double GetFactor_GammaZero();
        double GetFactor_A0();
        double GetFactor_B0();
        double GetFactor_Psi();
        double GetFactor_j();
        double GetFactor_k();
        double GetFactor_j1();
        double GetFactor_Psi_1();
        double GetFactor_Psi_2();



    private:

        double LatG_rad;
        double LongG_rad;
        double LatMe_rad;
        double LongMe_rad;
        double PreTheta;
};

#endif