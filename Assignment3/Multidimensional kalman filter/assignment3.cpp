#include <iostream>
#include "kalman.hpp"


int main(void)
{

    MatrixXf x0(2, 1);// Initial state (location and velocity) 
    x0 << 0.0,
    	  0.0; 
    MatrixXf P0(2, 2);//Initial Uncertainty
    P0 << 100.0, 0.0, 
    	  0.0,   100.0; 
    MatrixXf F(2, 2);//Next State Function
    F << 1.0, 1.0,
    	 0.0, 1.0; 
    MatrixXf H(1, 2);//Measurement Function
    H << 1.0, 0.0; 
    MatrixXf Q(2, 2);//Motion Uncertainity
    Q << 0.0, 0.0,
    	 0.0, 0.0; 
    MatrixXf R(1, 1); //Measurement Uncertainty
    R << 1.0;
    MatrixXf I(2, 2);// Identity Matrix
    I << 1.0, 0.0,
    	 0.0, 1.0; 
    uint8_t n = 2;
    uint8_t m = 1;

    // MatrixXf u(2, 1);// External Motion
    // u << 0,
    // 	 0; 

    KalmanConfig_t k_config = 
    {
        .x0 = x0,
        .P0 = P0,
        .F = F,
        .H = H,
        .Q = Q,
        .R = R,
        .I = I,
        .n = n,
        .m = m
    };

    Kalman k_filter{k_config};

    k_filter.predict();


    cout << "HELLO WORLD." << endl;

    return 0;
}


/*

 
State Transition:
posterior = state_transition * prior,
x' = F*x
initial state x, 
state transition matrix F,

P represents state covariance in localization,
P' = F*P*F^T

Measurement Update:
Z represents how to map the state to the observation,
measurement matrix H, 
measurement residual y, (ie. we are comparing where the measurement tells us we are vs. where we think we are)
measurement noise(measurement uncertainty), denoted R

K = Kalman gain

*/