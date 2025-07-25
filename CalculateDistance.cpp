/* Created by 鳥井太智 in June 2025*/

#include "CalculateDistance.h"
#include <Arduino.h>
#include <cmath>
#include <math.h>

#ifdef ESP32

CalculateDistance::CalculateDistance(double LatMe_deg, double LatG_deg, double LongMe_deg, double LongG_deg) {
    LatG_rad = ToRad(LatG_deg);
    LongG_rad = ToRad(LongG_deg);
    LatMe_rad = ToRad(LatMe_deg);
    LongMe_rad = ToRad(LongMe_deg);

    PreTheta = SetThetaZero();

}

#else
#endif

double CalculateDistance::ToRad(double degree) {
    return degree * DEG_TO_RAD;
}

/* l */
double CalculateDistance::GetFactor_l(){
    double Factor_l = LongG_rad - LongMe_rad;
    return Factor_l;
}

/* l' */
double CalculateDistance::GetFactor_l_1() {
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
double CalculateDistance::GetFactor_L() {
    double Factor_L = abs(GetFactor_l_1());
    return Factor_L;
}

/* L' */
double CalculateDistance::GetFactor_L_1() {
    double Factor_L_1 = PI - GetFactor_L();
    return Factor_L_1;
}

/* Σ */
double CalculateDistance::GetFactor_SIGMA() {
    double Factor_SIGMA = LatMe_rad + LatG_rad;
    return Factor_SIGMA;
}

/* u1 */
double CalculateDistance::GetFactor_u1() {
    double Factor_u1;
    double l_1 = GetFactor_l_1();

    if (l_1 >= 0) {
        double parameter = (1 - f_D) * tan(LatMe_rad);
        Factor_u1 = atan(parameter);

    } else {
        double parameter = (1 - f_D) * tan(LatG_rad);
        Factor_u1 = atan(parameter);

    }

    return Factor_u1;
}

/* u2 */
double CalculateDistance::GetFactor_u2() {
    double Factor_u2;

    if (GetFactor_l_1() >= 0) {
        double parameter = (1 - f_D) * tan(LatG_rad);
        Factor_u2 = atan(parameter);
    } else {
        double parameter = (1 - f_D) * tan(LatMe_rad);
        Factor_u2 = atan(parameter);
    }

    return Factor_u2;
}

/* Σ' */
double CalculateDistance::GetFactor_SIGMA_1() {
    double u1 = GetFactor_u1();
    double u2 = GetFactor_u2();

    double Factor_SIGMA_1 = u1 + u2;
    return Factor_SIGMA_1;
}

/* Δ' */
double CalculateDistance::GetFactor_DELTA_1() {
    double u1 = GetFactor_u1();
    double u2 = GetFactor_u2();

    double Factor_DELTA_1 = u2 - u1;
    return Factor_DELTA_1;
}

/* ξ */
double CalculateDistance::GetFactor_Xi() {
    double SIGMA_1 = GetFactor_SIGMA_1();

    double Factor_Xi = cos(SIGMA_1 / 2);
    return Factor_Xi;
}

/* ξ' */
double CalculateDistance::GetFactor_Xi_1() {
    double SIGMA_1 = GetFactor_SIGMA_1();

    double Factor_Xi_1 = sin(SIGMA_1 / 2);
    return Factor_Xi_1;
}

/* η */
double CalculateDistance::GetFactor_Eta() {
    double DELTA_1 = GetFactor_DELTA_1();

    double Factor_Eta = sin(DELTA_1 / 2);
    return Factor_Eta;
}

/* η' */
double CalculateDistance::GetFactor_Eta_1() {
    double DELTA_1 = GetFactor_DELTA_1();

    double Factor_Eta_1 = cos(DELTA_1 / 2);
    return Factor_Eta_1;
}

/* x */
double CalculateDistance::GetFactor_x() {
    double u1 = GetFactor_u1();
    double u2 = GetFactor_u2();

    double Factor_x = sin(u1) * sin(u2);
    return Factor_x;
}

/* y */
double CalculateDistance::GetFactor_y() {
    double u1 = GetFactor_u1();
    double u2 = GetFactor_u2();

    double Factor_y = cos(u1) * cos(u2);
    return Factor_y;
}

/* c */
double CalculateDistance::GetFactor_c() {
    double x = GetFactor_x();
    double y = GetFactor_y();
    double L = GetFactor_L();

    double Factor_c = y * cos(L) + x;
    return Factor_c;
}

/* ε */
double CalculateDistance::GetFactor_Epsilon() {
    double Factor_Epsilon = (f_D * (2 - f_D)) / pow((1 - f_D), 2);
    return Factor_Epsilon;
}


/* g */
double CalculateDistance::GetFactor_g() {
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
double CalculateDistance::GetFactor_h() {
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
double CalculateDistance::GetFactor_sigma() {
    double g = GetFactor_g();
    double h = GetFactor_h();

    double Factor_sigma = 2 * atan(g / h);
    return Factor_sigma;
}

/* J */
double CalculateDistance::GetFactor_J() {
    double g = GetFactor_g();
    double h = GetFactor_h();

    double Factor_J = 2 * g * h;
    return Factor_J;
}

/* K */
double CalculateDistance::GetFactor_K() {
    double g = GetFactor_g();
    double h = GetFactor_h();

    double Factor_K = pow(h, 2) - pow(g, 2);
    return Factor_K;
}

/* γ */
double CalculateDistance::GetFactor_gamma() {
    double y = GetFactor_y();
    double J = GetFactor_J();

    double Factor_gamma = (y * sin(PreTheta)) / J;
    return Factor_gamma;
}

/* Γ　*/
double CalculateDistance::GetFactor_GAMMA() {
    double gamma = GetFactor_gamma();

    double Factor_GAMMA = 1 - pow(gamma, 2);
    return Factor_GAMMA;
}

/* ζ　 */
double CalculateDistance::GetFactor_zeta() {
    double x = GetFactor_x();
    double K = GetFactor_K();
    double GAMMA = GetFactor_GAMMA();

    double Factor_zeta = GAMMA * K - 2 * x;
    return Factor_zeta;
}

/* ζ' */
double CalculateDistance::GetFactor_zeta_1() {
    double x = GetFactor_x();
    double zeta = GetFactor_zeta();

    double Factor_zeta_1 = zeta + x;
    return Factor_zeta_1;
}

/* D */
double CalculateDistance::GetFactor_D() {
    double GAMMA = GetFactor_GAMMA();

    double Factor_D = (1/4) * f_D * (1 + f_D) - (3/16) * pow(f_D, 2) * GAMMA;
    return Factor_D;
}

/* E */
double CalculateDistance::GetFactor_E() {
    double D = GetFactor_D();
    double sigma = GetFactor_sigma();
    double gamma = GetFactor_gamma();
    double GAMMA = GetFactor_GAMMA();
    double zeta = GetFactor_zeta();
    double J = GetFactor_J();   
    double K = GetFactor_K();

    double part_1 = 1 - D * GAMMA;
    double part_2 = f_D * gamma;
    double part_3 = sigma + D * J * (zeta + D * K * (2 * pow(zeta, 2) - pow(GAMMA, 2)));

    double Factor_E = part_1 * part_2 * part_3;
    return Factor_E;
}

/* F */
double CalculateDistance::GetFactor_F() {
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
double CalculateDistance::GetFactor_G() {
    double gamma = GetFactor_gamma();
    double GAMMA = GetFactor_GAMMA();
    double sigma = GetFactor_sigma();
    double zeta = GetFactor_zeta();
    double zeta_1 = GetFactor_zeta_1();
    double D = GetFactor_D();
    double J = GetFactor_J();   

    double term_1 = f_D * pow(gamma, 2) * (1 - 2 * D * GAMMA);
    double term_2 = f_D * zeta_1 * (sigma / J) * (1 - D * GAMMA + (1/2) * f_D * pow(gamma, 2));
    double term_3 = (1/4) * pow(f_D, 2) * zeta * zeta_1;

    double Factor_G = term_1 + term_2 + term_3;
    return Factor_G;
}


/* n0 */
double CalculateDistance::GetFactor_n0() {
    double Epsilon = GetFactor_Epsilon();
    double GAMMA = GetFactor_GAMMA();

    double Denominator = pow(sqrt(1 + Epsilon * GAMMA) + 1, 2);

    double Factor_n0 = Epsilon * GAMMA / Denominator;

    return Factor_n0;
}

/* A */
double CalculateDistance::GetFactor_A() {
    double n0 = GetFactor_n0();

    double Factor_A = (1 + n0) * (1 + (5/4) * pow(n0, 2));

    return Factor_A;
}

/* B */
double CalculateDistance::GetFactor_B() {
    double Epsilon = GetFactor_Epsilon();
    double n0 = GetFactor_n0();
    double GAMMA = GetFactor_GAMMA();

    double Numerator = Epsilon * (1 - (3/8) * pow(n0, 2));
    double Denominator = pow(sqrt(1 + Epsilon * GAMMA) + 1, 2);

    double Factor_B = Numerator / Denominator;

    return Factor_B;
}


/* R */
double CalculateDistance::GetFactor_R() {
    double u1 = GetFactor_u1();
    double part_left = f_D * PI * pow(cos(u1), 2);
    double part_right_1 = (1/4) * f_D * (1 + f_D) * pow(sin(u1), 2);
    double part_right_2 = (3/16) * pow(f_D, 2) * pow(sin(u1), 4);

    double Factor_R = part_left * (1 - part_right_1 + part_right_2);
    return Factor_R;
}

/* d1 */
double CalculateDistance::GetFactor_d1() {
    double u1 = GetFactor_u1();
    double R = GetFactor_R();
    double L_1 = GetFactor_L_1();

    double Factor_d1 = (L_1 * cos(u1)) - R;
    return Factor_d1;
}

/* d2 */
double CalculateDistance::GetFactor_d2() {
    double SIGMA_1 = GetFactor_SIGMA_1();
    double R = GetFactor_R();

    double Factor_d2 = abs(SIGMA_1) + R;
    return Factor_d2;
}

/* q */
double CalculateDistance::GetFactor_q() {
    double L_1 = GetFactor_L_1();
    
    double Factor_q = L_1 / (f_D * PI);
    return Factor_q;
}

/* f1 */
double CalculateDistance::GetFactor_f1() {
    double Factor_f1 = (1/4) * f_D * (1 + (1/2) * f_D);
    return Factor_f1;
}

/* γ0 */
double CalculateDistance::GetFactor_GammaZero() {
    double q = GetFactor_q();
    double f1 = GetFactor_f1();

    double Factor_Gamma0 = q + (f1 * q) - (f1 * pow(q, 3));
    return Factor_Gamma0;
}

/* A0 */
double CalculateDistance::GetFactor_A0() {
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
double CalculateDistance::GetFactor_B0() {
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
double CalculateDistance::GetFactor_Psi() {
    double A0 = GetFactor_A0();
    double B0 = GetFactor_B0();

    double Factor_Psi = A0 + B0;
    return Factor_Psi;
}

/* j */
double CalculateDistance::GetFactor_j() {
    double GammaZero = GetFactor_GammaZero();
    double u1 = GetFactor_u1();  

    double Factor_j = GammaZero / cos(u1);
    return Factor_j;
}

/* k */
double CalculateDistance::GetFactor_k() {
    double SIGMA_1 = GetFactor_SIGMA_1();
    double y = GetFactor_y();
    double f1 = GetFactor_f1();
    double part1 = 1 + f1;
    double part2 = abs(SIGMA_1);
    double part3 = (1 - f_D * y) / (f_D * PI * y);

    double Factor_k = part1 * part2 * part3;
    return Factor_k;
}

/* j1 */
double CalculateDistance::GetFactor_j1() {
    double j = GetFactor_j();
    double k = GetFactor_k();
    double Psi = GetFactor_Psi();
    double Denominator = 1 + k * (1 / cos(Psi));

    double Factor_j1 = j / Denominator;
    return Factor_j1;
}

/* ψ' */
double CalculateDistance::GetFactor_Psi_1() {
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
double CalculateDistance::GetFactor_Psi_2() {
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

/* θ0 */
double CalculateDistance::SetThetaZero() {
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
        ThetaZero = L * (1 + f_D * y);
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
double CalculateDistance::CalculateTheta() {
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


/* s */
double CalculateDistance::GetDistance(double LatMe_deg, double LatG_deg_deg, double LongMe_deg, double LongG_deg) {
    CalculateTheta();

    double Distance;
    double A = GetFactor_A();
    double sigma = GetFactor_sigma();
    double B = GetFactor_B();
    double J = GetFactor_J();
    double K = GetFactor_K();
    double GAMMA = GetFactor_GAMMA();
    double zeta = GetFactor_zeta();
    double d1 = GetFactor_d1();

    if (d1 <= 0) {
        Distance = (1 - f_D) * EQUATORIAL_RADIUS_D * A * PI;
    } else {
        double r1 = pow(GAMMA, 2) - 2 * pow(zeta, 2);
        double r2 = 1 - 4 * pow(K,2);
        double r3 = 3 * pow(GAMMA, 2) - 4 * pow(zeta, 2);
        double r4 = K * r1;
        double r5 = (1/6) * B * zeta * r2 * r3;
        double r6 = r4 - r5;
        double r7 = (1/4) * B * r6;
        double r8 = zeta - r7;
        double part1 = (1 - f_D) * EQUATORIAL_RADIUS_D * A;
        double part2 = sigma - B * J * r8;

        Distance = part1 * part2;
    }

    PreTheta = SetThetaZero();


    return Distance;
}
