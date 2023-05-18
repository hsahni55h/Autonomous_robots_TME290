#ifndef __KALMAN_HPP
#define __KALMAN_HPP

#include "Core" // Eigen Library
#include "LU"   // Eigen Library

using namespace std;
using namespace Eigen;


// typedef Matrix<double, Dynamic, Dynamic> MatrixXd;

typedef struct KalmanConfig 
{
    MatrixXd x0;    // prior state mean
    MatrixXd P0;    // prior state covariance
    MatrixXd F;     // motion model
    MatrixXd H;     // measurement model
    MatrixXd Q;     // motion noise
    MatrixXd R;     // measurement noise
    MatrixXd I;     // identity matrix of nxn
    uint8_t n;      // number of states
    uint8_t m;      // number of measurements
    uint32_t kmax;  // capcity or k_max
} KalmanConfig_t;

/**
 * @brief Multi-dimensional Kalman filter. Functions that return bool return true if function was executed without any error. 
 * 
 */
class Kalman
{
    private:
        // MatrixXd F;                     // motion model (from user input)
        // MatrixXd H;                     // measurement model (from user input)
        // MatrixXd Q;                     // motion noise (from user input)
        // MatrixXd R;                     // measurement noise (from user input)
        // MatrixXd I;                     // identity matrix of nxn

        // uint8_t n;                      // number of states
        // uint8_t m;                      // number of measurements
        // uint32_t k;                     // discrete time instance, k = 1, 2, 3, ...
        // uint32_t kmax;                  // capcity or k_max

        // vector<MatrixXd> x;             // state mean - x[0] is the prior mean config.x0, posterior starts from index 1 i.e. k=1
        // vector<MatrixXd> P;             // state covariance - P[0] is the prior covariance config.P0, posterior starts from index 1 i.e. k=1
        // vector<MatrixXd> z;             // measurement - z[0] is invalid, measurements starts from index 1 i.e. k=1

    public:
        Kalman(const KalmanConfig_t& config);       // ctor
        ~Kalman();                                  // dtor

        bool predict();                     // performs prediction step for Kalman filter
        bool update(MatrixXd z_in);            // performs update step for Kalman filter
        // bool step(MatrixXd z_in);              // performs both prediction and update for one time instance
        // bool run(vector<MatrixXd> z_arr);       // performs both prediction and update step over given set of measurements from current k for all given measurements z.

        // MatrixXd get_current_state_mean() const;         // returns current state mean i.e state mean at time instance Kalman.k
        // MatrixXd get_current_state_covar() const;       // returns current state covariance i.e state covariance at time instance Kalman.k

        // MatrixXd get_state_mean_at(const uint32_t k_in) const;     // returns state mean at input time instance k_in 
        // MatrixXd get_state_covar_at(const uint32_t k_in) const;    // returns state covariance at input time instance k_in

        // const vector<MatrixXd>& get_all_state_means(uint32_t* k_curr) const;      // outputs a pointer to state mean vector
        // const vector<MatrixXd>& get_all_state_covars(uint32_t* k_curr) const;     // outputs a pointer to state covariance vector
};

#endif // __KALMAN_HPP


