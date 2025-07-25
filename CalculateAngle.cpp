/* Created by 鳥井太智 in July 2025 */

#include "CalculateAngle.h"
#include <Arduino.h>
#include <cmath>
#include <math.h>

#ifdef ESP32

CalculateAngle::CalculateAngle(double LatMe_deg, double LatG_deg, double LongMe_deg, double LongG_deg) {
    
    LatG_rad = ToRad(LatG_deg);
    LongG_rad = ToRad(LongG_deg);
    LatMe_rad = ToRad(LatMe_deg);
    LongMe_rad = ToRad(LongMe_deg);

    //PreTheta = SetThetaZero();

}

#else
#endif

void CalculateAngle::Coordinates(double LatMe_deg_new, double LatG_deg_new, double LongMe_deg_new, double LongG_deg_new) {
    
    LatG_rad = ToRad(LatG_deg_new);
    LongG_rad = ToRad(LongG_deg_new);
    LatMe_rad = ToRad(LatMe_deg_new);
    LongMe_rad = ToRad(LongMe_deg_new);

    PreTheta = SetThetaZero();
}

double CalculateAngle::ToRad(double degree) {
    return degree * DEG_TO_RAD;
}

/* l */
double CalculateAngle::GetFactor_l(){
    double Factor_l = LongG_rad - LongMe_rad;
    return Factor_l;
}

/* l' */
double CalculateAngle::GetFactor_l_1() {
    double l = GetFactor_l();
    double Factor_l_1;

    if (l > PI) {
        Factor_l_1 = l - (2 * PI);
    } else if (l < -PI) {
        Factor_l_1 = l + (2 * PI);
    } else {
        Factor_l_1 = l;
    }

    return Factor_l_1;
}

/* L */
double CalculateAngle::GetFactor_L() {
    double Factor_L = abs(GetFactor_l_1());
    return Factor_L;
}

/* L' */
double CalculateAngle::GetFactor_L_1() {
    double Factor_L_1 = PI - GetFactor_L();
    return Factor_L_1;
}

/* Δ */
double CalculateAngle::GetFactor_DELTA() {
    double l_1 = GetFactor_l_1();
    double Factor_DELTA;

    if (l_1 >= 0) {
        Factor_DELTA = LatG_rad - LatMe_rad;
    } else {
        Factor_DELTA = LatMe_rad - LatG_rad;
    }

    return Factor_DELTA;
}

/* Σ */
double CalculateAngle::GetFactor_SIGMA() {
    double Factor_SIGMA = LatMe_rad + LatG_rad;
    return Factor_SIGMA;
}

/* u1 */
double CalculateAngle::GetFactor_u1() {
    double Factor_u1;
    double l_1 = GetFactor_l_1();

    if (l_1 >= 0) {
        double parameter = (1 - f_A) * tan(LatMe_rad);
        Factor_u1 = atan(parameter);

    } else {
        double parameter = (1 - f_A) * tan(LatG_rad);
        Factor_u1 = atan(parameter);
    }

    return Factor_u1;
}

/* u2 */
double CalculateAngle::GetFactor_u2() {
    double Factor_u2;

    if (GetFactor_l_1() >= 0) {
        double parameter = (1 - f_A) * tan(LatG_rad);
        Factor_u2 = atan(parameter);
    } else {
        double parameter = (1 - f_A) * tan(LatMe_rad);
        Factor_u2 = atan(parameter);
    }

    return Factor_u2;
}

/* Σ' */
double CalculateAngle::GetFactor_SIGMA_1() {
    double u1 = GetFactor_u1();
    double u2 = GetFactor_u2();

    double Factor_SIGMA_1 = u1 + u2;
    return Factor_SIGMA_1;
}

/* Δ' */
double CalculateAngle::GetFactor_DELTA_1() {
    double u1 = GetFactor_u1();
    double u2 = GetFactor_u2();

    double Factor_DELTA_1 = u2 - u1;
    return Factor_DELTA_1;
}

/* ξ */
double CalculateAngle::GetFactor_Xi() {
    double SIGMA_1 = GetFactor_SIGMA_1();

    double Factor_Xi = cos(SIGMA_1 / 2);
    return Factor_Xi;
}

/* ξ' */
double CalculateAngle::GetFactor_Xi_1() {
    double SIGMA_1 = GetFactor_SIGMA_1();

    double Factor_Xi_1 = sin(SIGMA_1 / 2);
    return Factor_Xi_1;
}

/* η */
double CalculateAngle::GetFactor_Eta() {
    double DELTA_1 = GetFactor_DELTA_1();

    double Factor_Eta = sin(DELTA_1 / 2);
    return Factor_Eta;
}

/* η' */
double CalculateAngle::GetFactor_Eta_1() {
    double DELTA_1 = GetFactor_DELTA_1();

    double Factor_Eta_1 = cos(DELTA_1 / 2);
    return Factor_Eta_1;
}

/* x */
double CalculateAngle::GetFactor_x() {
    double u1 = GetFactor_u1();
    double u2 = GetFactor_u2();

    double Factor_x = sin(u1) * sin(u2);
    return Factor_x;
}

/* y */
double CalculateAngle::GetFactor_y() {
    double u1 = GetFactor_u1();
    double u2 = GetFactor_u2();

    double Factor_y = cos(u1) * cos(u2);
    return Factor_y;
}

/* c */
double CalculateAngle::GetFactor_c() {
    double x = GetFactor_x();
    double y = GetFactor_y();
    double L = GetFactor_L();

    double Factor_c = y * cos(L) + x;
    return Factor_c;
}



/* g */
double CalculateAngle::GetFactor_g() {
    double c = GetFactor_c();
    double Eta = GetFactor_Eta();
    double Xi = GetFactor_Xi();
    double Factor_g;

    if (c >= 0) {
        double part1 = pow(Eta, 2) * pow(cos(PreTheta / 2), 2);
        double part2 = pow(Xi, 2) * pow(sin(PreTheta / 2), 2);

        Factor_g = sqrt(part1 + part2);
    } else {
        double part1 = pow(Eta, 2) * pow(sin(PreTheta / 2), 2);
        double part2 = pow(Xi, 2) * pow(cos(PreTheta / 2), 2);

        Factor_g = sqrt(part1 + part2);
    }

    return Factor_g;
}

/* h */
double CalculateAngle::GetFactor_h() {
    double c = GetFactor_c();
    double Eta_1 = GetFactor_Eta_1();
    double Xi_1 = GetFactor_Xi_1();
    double Factor_h;

    if (c >= 0) {
        double part1 = pow(Eta_1, 2) * pow(cos(PreTheta / 2), 2);
        double part2 = pow(Xi_1, 2) * pow(sin(PreTheta / 2), 2);

        Factor_h = sqrt(part1 + part2);
    } else {
        double part1 = pow(Eta_1, 2) * pow(sin(PreTheta / 2), 2);
        double part2 = pow(Xi_1, 2) * pow(cos(PreTheta / 2), 2);

        Factor_h = sqrt(part1 + part2);
    } 

    return Factor_h;
}

/* σ */
double CalculateAngle::GetFactor_sigma() {
    double g = GetFactor_g();
    double h = GetFactor_h();

    double Factor_sigma = 2 * atan(g / h);
    return Factor_sigma;
}

/* J */
double CalculateAngle::GetFactor_J() {
    double g = GetFactor_g();
    double h = GetFactor_h();

    double Factor_J = 2 * g * h;
    return Factor_J;
}

/* K */
double CalculateAngle::GetFactor_K() {
    double g = GetFactor_g();
    double h = GetFactor_h();

    double Factor_K = pow(h, 2) - pow(g, 2);
    return Factor_K;
}

/* γ */
double CalculateAngle::GetFactor_gamma() {
    double y = GetFactor_y();
    double J = GetFactor_J();

    double Factor_gamma = (y * sin(PreTheta)) / J;
    return Factor_gamma;
}

/* Γ　*/
double CalculateAngle::GetFactor_GAMMA() {
    double gamma = GetFactor_gamma();

    double Factor_GAMMA = 1 - pow(gamma, 2);
    return Factor_GAMMA;
}


/* ζ　 */
double CalculateAngle::GetFactor_zeta() {
    double x = GetFactor_x();
    double K = GetFactor_K();
    double GAMMA = GetFactor_GAMMA();

    double Factor_zeta = GAMMA * K - 2 * x;
    return Factor_zeta;
}

/* ζ' */
double CalculateAngle::GetFactor_zeta_1() {
    double x = GetFactor_x();
    double zeta = GetFactor_zeta();

    double Factor_zeta_1 = zeta + x;
    return Factor_zeta_1;
}

/* R */
double CalculateAngle::GetFactor_R() {
    double u1 = GetFactor_u1();
    double part_left = f_A * PI * pow(cos(u1), 2);
    double part_right_1 = (1/4) * f_A * (1 + f_A) * pow(sin(u1), 2);
    double part_right_2 = (3/16) * pow(f_A, 2) * pow(sin(u1), 4);

    double Factor_R = part_left * (1 - part_right_1 + part_right_2);
    return Factor_R;
}

/* q */
double CalculateAngle::GetFactor_q() {
    double L_1 = GetFactor_L_1();
    
    double Factor_q = L_1 / (f_A * PI);
    return Factor_q;
}

/* f1 */
double CalculateAngle::GetFactor_f1() {
    double Factor_f1 = (1/4) * f_A * (1 + (1/2) * f_A);
    return Factor_f1;
}


/* γ0 */
double CalculateAngle::GetFactor_GammaZero() {
    double q = GetFactor_q();
    double f1 = GetFactor_f1();

    double Factor_Gamma0 = q + (f1 * q) - (f1 * pow(q, 3));
    return Factor_Gamma0;
}

/* j */
double CalculateAngle::GetFactor_j() {
    double GammaZero = GetFactor_GammaZero();
    double u1 = GetFactor_u1();  

    double Factor_j = GammaZero / cos(u1);
    return Factor_j;
}

/* k */
double CalculateAngle::GetFactor_k() {
    double SIGMA_1 = GetFactor_SIGMA_1();
    double y = GetFactor_y();
    double f1 = GetFactor_f1();
    double part1 = 1 + f1;
    double part2 = abs(SIGMA_1);
    double part3 = (1 - f_A * y) / (f_A * PI * y);

    double Factor_k = part1 * part2 * part3;
    return Factor_k;
}

/* d1 */
double CalculateAngle::GetFactor_d1() {
    double u1 = GetFactor_u1();
    double R = GetFactor_R();
    double L_1 = GetFactor_L_1();

    double Factor_d1 = (L_1 * cos(u1)) - R;
    return Factor_d1;
}

/* d2 */
double CalculateAngle::GetFactor_d2() {
    double SIGMA_1 = GetFactor_SIGMA_1();
    double R = GetFactor_R();

    double Factor_d2 = abs(SIGMA_1) + R;
    return Factor_d2;
}

/* A0 */
double CalculateAngle::GetFactor_A0() {
    double d1 = GetFactor_d1();
    double d2 = GetFactor_d2();
    double Factor_A0;

    if ((-PI / 2) <= atan(d1 / d2) &&  atan(d1 / d2) <= (PI / 2)) {
        Factor_A0 = atan(d1 / d2);
    } else {
        Serial.print("Angle A0 Error: The A0 is not the specified value. A0 must be between -90° and 90°.");
        return NAN;
    }
    
    return Factor_A0;
}

/* B0 */
double CalculateAngle::GetFactor_B0() {
    double R = GetFactor_R();
    double d1 = GetFactor_d1();
    double d2 = GetFactor_d2();
    double Denominator = sqrt(pow(d1, 2) + pow(d2, 2));
    double Factor_B0;

    if (0 <= asin(R / Denominator) && asin(R / Denominator) <= (PI / 2)) 
        {
        Factor_B0 = asin(R / Denominator);
        } 
    else 
        {
        Serial.print("Angle B0 Error: The B0 is not the specified value. B0 must be between 0° and 90°.");
        return NAN;
        }

    return Factor_B0;
}

/* ψ */
double CalculateAngle::GetFactor_Psi() {
    double A0 = GetFactor_A0();
    double B0 = GetFactor_B0();

    double Factor_Psi = A0 + B0;
    return Factor_Psi;
}

/* j1 */
double CalculateAngle::GetFactor_j1() {
    double j = GetFactor_j();
    double k = GetFactor_k();
    double Psi = GetFactor_Psi();
    double Denominator = 1 + k * (1 / cos(Psi));

    double Factor_j1 = j / Denominator;
    return Factor_j1;
}


/* ψ' */
double CalculateAngle::GetFactor_Psi_1() {
    double j1 = GetFactor_j1();
    double Factor_Psi_1;

    if (0 <= asin(j1) && asin(j1) <= (PI / 2)) {
        Factor_Psi_1 = asin(j1);
    } else {
        Serial.print("Angle ψ' Error: The ψ' is not the specified value. ψ' must be between 0° to 90°.");
        return NAN;
    }

    return Factor_Psi_1;
}

/* ψ'' */
double CalculateAngle::GetFactor_Psi_2() {
    double Factor_Psi_2;
    double j1 = GetFactor_j1();
    double u1 = GetFactor_u1();
    double u2 = GetFactor_u2();

    if (0 <= asin((j1 * cos(u1)) / cos(u2)) && asin((j1 * cos(u1)) / cos(u2)) <= (PI / 2)) {
        Factor_Psi_2 = asin(j1 * cos(u1) / cos(u2));
    } else {
        Serial.print("Angle ψ'' Error: The ψ'' is not the specified value. ψ'' must be between 0° to 90°.");
        return NAN;
    }

    return Factor_Psi_2;
}

/* D */
double CalculateAngle::GetFactor_D() {
    double GAMMA = GetFactor_GAMMA();

    double Factor_D = (1/4) * f_A * (1 + f_A) - (3/16) * pow(f_A, 2) * GAMMA;
    return Factor_D;
}

/* E */
double CalculateAngle::GetFactor_E() {
    double D = GetFactor_D();
    double sigma = GetFactor_sigma();
    double gamma = GetFactor_gamma();
    double GAMMA = GetFactor_GAMMA();
    double zeta = GetFactor_zeta();
    double J = GetFactor_J();   
    double K = GetFactor_K();

    double part_1 = 1 - D * GAMMA;
    double part_2 = f_A * gamma;
    double part_3 = sigma + D * J * (zeta + D * K * (2 * pow(zeta, 2) - pow(GAMMA, 2)));

    double Factor_E = part_1 * part_2 * part_3;
    return Factor_E;
}

/* F */
double CalculateAngle::GetFactor_F() {
    double c = GetFactor_c();
    double L = GetFactor_L();
    double L_1 = GetFactor_L_1();
    double E = GetFactor_E();
    double Factor_F;

    if (c >= 0) {
        Factor_F = PreTheta - L - E;
    } else {
        Factor_F = PreTheta - L_1 + E;
    }

    return Factor_F;
}

/* G */
double CalculateAngle::GetFactor_G() {
    double gamma = GetFactor_gamma();
    double GAMMA = GetFactor_GAMMA();
    double sigma = GetFactor_sigma();
    double zeta = GetFactor_zeta();
    double zeta_1 = GetFactor_zeta_1();
    double D = GetFactor_D();
    double J = GetFactor_J();   

    double term_1 = f_A * pow(gamma, 2) * (1 - 2 * D * GAMMA);
    double term_2 = f_A * zeta_1 * (sigma / J) * (1 - D * GAMMA + (1/2) * f_A * pow(gamma, 2));
    double term_3 = (1/4) * pow(f_A, 2) * zeta * zeta_1;

    double Factor_G = term_1 + term_2 + term_3;
    return Factor_G;
}

/* θ0 */
double CalculateAngle::SetThetaZero() {
    double ThetaZero;
    double c = GetFactor_c();
    double L = GetFactor_L();
    double y = GetFactor_y();
    double u1 = GetFactor_u1();
    double d1 = GetFactor_d1();
    double c_limit_parameter = 3 * cos(d1) * DEG_TO_RAD;
    double c_limit = -cos(c_limit_parameter);
    double L_1 = GetFactor_L_1();
    double Psi_1 = GetFactor_Psi_1();
    double Psi_2 = GetFactor_Psi_2();
    double SIGMA = GetFactor_SIGMA();
    double SIGMA_1 = GetFactor_SIGMA_1();
    double DELTA_1 = GetFactor_DELTA_1();

    if (c >= 0) {
        ThetaZero = L * (1 + f_A * y);
    } else if (c_limit <= c && c < 0) {
        ThetaZero = L_1;
    } else {
        if (SIGMA == 0) {
            if (d1 > 0) {
                ThetaZero = L_1;
            } 
        } else {
            double Numerator_1 = tan((Psi_1 + Psi_2) / 2);
            double Numerator_2 = sin(abs(SIGMA_1) / 2);
            double Numerator = Numerator_1 * Numerator_2;
            double Denominator = cos(DELTA_1 / 2);

            ThetaZero = 2 * atan(Numerator / Denominator);
        }
    }

    return ThetaZero;
}


/* θ */
double CalculateAngle::CalculateTheta() {
    double Theta = SetThetaZero();
    double F = GetFactor_F();
    double G = GetFactor_G();
    const int MAX_ITER = 5;
    const double TOL = pow(10, -15);

    for (int i = 0; i < MAX_ITER; i++) {

        if (abs(F) < TOL) {
            return Theta;
        }

        Theta = Theta - (F / (1 - G));
        PreTheta = Theta;
    }

    return NAN;
}


/* α */
double CalculateAngle::GetFactor_Alpha() {

    double part2 = tan(PreTheta / 2);
    double c = GetFactor_c();
    double Factor_Alpha;

    if (c >= 0) {
        double Xi = GetFactor_Xi();
        double Eta = GetFactor_Eta();
        double part1 = Xi / Eta;

        Factor_Alpha = atan(part1 * part2);

    } else {
        double Xi_1 = GetFactor_Xi_1();
        double Eta_1 = GetFactor_Eta_1();
        double part1 = Eta_1 / Xi_1;

        Factor_Alpha = atan(part1 * part2);
    }

    return Factor_Alpha;
}

/* α/2 */
double CalculateAngle::GetFactor_AlphaHalf() {

    double part2 = tan(PreTheta / 2);
    double c = GetFactor_c();
    double Factor_AlphaHalf;

    if (c >= 0) {
        double Xi_1 = GetFactor_Xi_1();
        double Eta_1 = GetFactor_Eta_1();
        double part1 = Xi_1 / Eta_1;

        Factor_AlphaHalf = atan(part1 * part2);
    } else {
        double Xi = GetFactor_Xi();
        double Eta = GetFactor_Eta();
        double part1 = Eta / Xi;

        Factor_AlphaHalf = atan(part1 * part2);
    }

    return Factor_AlphaHalf;
}

/* α' */
double CalculateAngle::GetFactor_Alpha_1() {

    double Alpha = GetFactor_Alpha();
    double L = GetFactor_L();
    double Factor_Alpha_1;

    if (Alpha < 0 && L > 0) {
        Factor_Alpha_1 = Alpha + PI;
    } else {
        Factor_Alpha_1 = Alpha;
    }

    return Factor_Alpha_1;

}

/* α1' */
double CalculateAngle::GetFactor_Alpha1_1() {

    double Alpha_1 = GetFactor_Alpha_1();
    double AlphaHalf = GetFactor_AlphaHalf();
    double Factor_Alpha1_1 = Alpha_1 - AlphaHalf;

    return Factor_Alpha1_1;
}

/* α2 */
double CalculateAngle::GetFactor_Alpha_2() {

    double Alpha_1 = GetFactor_Alpha_1();
    double AlphaHalf = GetFactor_AlphaHalf();
    double c = GetFactor_c();
    double Factor_Alpha_2;

    if (c >= 0 ) {
        Factor_Alpha_2 = Alpha_1 + AlphaHalf;
    } else {
        Factor_Alpha_2 = PI - Alpha_1 - AlphaHalf;
    }

    return Factor_Alpha_2;
}

/* α21' */
double CalculateAngle::GetFactor_Alpha21_1() {
    double Factor_Alpha21_1 = PI - GetFactor_AlphaHalf();
    return Factor_Alpha21_1;
}

/* α1 */
double CalculateAngle::GetFactor_Alpha1() {
    double L = GetFactor_L();
    double DELTA = GetFactor_DELTA();
    double SIGMA = GetFactor_SIGMA();
    double l_1 = GetFactor_l_1();
    double Alpha1_1 = GetFactor_Alpha1_1();
    double Alpha21_1 = GetFactor_Alpha21_1();
    double Factor_Alpha1;

    if (L == 0 && DELTA >= 0) {
        Factor_Alpha1 = 0;
    } else if (abs(L) == PI && SIGMA >= 0) {
        Factor_Alpha1 = 0;       
    } else if (L == 0 && DELTA < 0) {
        Factor_Alpha1 = PI;       
    } else if (abs(L) == PI && SIGMA < 0) {
        Factor_Alpha1 = PI;              
    } else if (l_1 >= 0) {
        Factor_Alpha1 = Alpha1_1;
    } else {
        Factor_Alpha1 = Alpha21_1;
    }

    while (Factor_Alpha1 < 0) {
        Factor_Alpha1 += 2 * PI;
    }

    while (Factor_Alpha1 > 2 * PI) {
        Factor_Alpha1 -= 2 * PI;
    }

    return Factor_Alpha1;
}
