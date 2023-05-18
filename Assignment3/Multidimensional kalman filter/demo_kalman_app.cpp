#include <iostream>
#include "kalman.hpp"

using namespace std;

#define DT 0.1

#define F_MATRIX    1, DT, \
                    0, 1

#define Q_MATRIX    2, 0, \
                    0, 2

#define H_MATRIX    1, 0

#define R_MATRIX    0

int main(void)
{
    uint8_t n = 2;      // NUMBER OF STATES     - REF. KALMAN CLASS
    uint8_t m = 1;      // NUMBER OF SENSORS    - REF. KALMAN CLASS

    MatrixXd x0(n, 1);
    // x0.setZero();
    x0 << 0.0,          // pos-x
          1.0;          // vel-x

    MatrixXd P0(n, n);
    P0.setZero();

    Kalman k_filter(n, m, x0, P0);

    uint32_t k = 0;
    MatrixXd x;
    MatrixXd P;
    
    k_filter.get_state_mean_at(k, x);
    k_filter.get_state_covar_at(k, P);

    cout << "Mean at k=" << k << ":\n" << x << endl;
    cout << "Covariance at k=" << k << ":\n" << P << endl;

    MatrixXd F(n, n);
    F << F_MATRIX;
    
    MatrixXd Q(n, n);
    Q << Q_MATRIX;

    k_filter.predict(F, Q);

    k=1;
    k_filter.get_state_mean_at(k, x);
    k_filter.get_state_covar_at(k, P);
    
    cout << "Prediction Mean at k=" << k << ":\n" << x << endl;
    cout << "Prediction Covariance at k=" << k << ":\n" << P << endl;


    MatrixXd H(m, n);
    H << H_MATRIX;

    MatrixXd R(m, m); 
    R << R_MATRIX;

    MatrixXd z(m, 1);
    z << 0.3;

    k_filter.update(H, R, z);

    k_filter.get_state_mean_at(k, x);
    k_filter.get_state_covar_at(k, P);
    
    cout << "After Measurement Mean at k=" << k << ":\n" << x << endl;
    cout << "After Measurement Covariance at k=" << k << ":\n" << P << endl;

    z << 0.4;
    k_filter.step(F, Q, H, R, z);

    k = k_filter.get_current_step();
    k_filter.get_state_mean_at(k, x);
    k_filter.get_state_covar_at(k, P);
    
    cout << "After a step prediction + measurement Mean at k=" << k << ":\n" << x << endl;
    cout << "After a step prediction + measurement Covariance at k=" << k << ":\n" << P << endl;


    cout << "HELLO WORLD" << endl;
    return 0;
}

