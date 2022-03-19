#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include "./traitement_message.h"

float* Vitesse_relative(float* X, float puissance_vent, float* Vect_vent);
float* normalisation(float* X);

theta_constant(int nb_i_delais, float* new_answer, int i0, float*quaternion_desire, int debatement, char*message){
    //Traitement du message
        int i;
        float* X;
        float ub = X[3];
        float vb = X[4];
        float wb = X[5];
        float Q = normalisation(X);
        float q0 = Q[0];
        float q1 = Q[1];
        float q2 = Q[2];
        float q3 = Q[3];
        float p = X[10];
        float q = X[11];
        float r = X[12];
        dm;
        Poussee;
        Cx;
        Cn_alpha;
        Coeff;
        Distance;
        float l_d = Distance[0];
        float l_ref= Distance[1];
        float l_p= Distance[2];
        float l_o_xcg= Distance[3];
        Surface;
        rho;
        Inertie;
        puissance_vent;
        Vect_vent;
        deltr;
        deltq;

    //Normalisatio

}

float* normalisation(float* X){
    float q0 = X[0];
    float q1 = X[1];
    float q2 = X[2];
    float q3 = X[3];

    float N = q0**2 + q1**2 + q2**2 + q3**2;
    q0 = q0 / N;
    q1 = q1 / N;
    q2 = q2 / N;
    q3 = q3 / N;
    return [q0,q1,q2,q3]
}

float* Vitesse_relative(float X, float puissance_vent, float* Vect_vent){

}