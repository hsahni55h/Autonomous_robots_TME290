#include "kalman.hpp"


using namespace std;
using namespace Eigen;

Kalman::Kalman(const KalmanConfig_t& config) //: 
    // F(config.F), 
    // H(config.H), 
    // Q(config.Q), 
    // R(config.R), 
    // I(config.I),
    // n(config.n),
    // m(config.m),
    // k(0),
    // kmax(config.kmax)
{    

//     x.push_back(config.x0);
//     P.push_back(config.P0);
    
//     MatrixXd z0(m, 1);
//     z0.setZero();
//     z.push_back(z0);

}

// Kalman::Kalman(const KalmanConfig_t& config)
// {
// }

Kalman::~Kalman()
{
}


bool Kalman::predict()
{
    // MatrixXd xp(n, 1);
    // xp << F * x.at(k);
    // x.push_back(xp);                        // prediction state mean 

    // MatrixXd Pp(n, n);
    // Pp = F * P.at(k) * F.transpose() + Q;
    // P.push_back(Pp);                          // prediction state covariance

    // k++;
    return true;
}

bool Kalman::update(MatrixXd z_in)
{
    // MatrixXd y(m, 1);
    // MatrixXd S(m, m);
    // MatrixXd K(n, m);

    // y << z_in - (H * x.at(k));                         // residual
    // S << H * P.at(k) * H.transpose() + R;
    // K << P.at(k) * H.transpose() * S.inverse();     // Kalman gain
    
    // x.at(k) = x.at(k) + K * y;                      // updated state mean
    // P.at(k) = (I - (K * H)) * P.at(k);              // updated state covariance

    return true;
}

// bool Kalman::step(MatrixXd z_in)
// {
//     predict();
//     update(z_in);
//     return true;
// }


// bool Kalman::run(vector<MatrixXd> z_arr)
// {
//     for(auto zk : z_arr) 
//     {
//         predict();
//         update(zk);
//     }
//     return true;
// }


// MatrixXd Kalman::get_current_state_mean() const
// {
//     return x.at(k);
// }

// MatrixXd Kalman::get_current_state_covar() const
// {
//     return P.at(k);
// }            


// MatrixXd Kalman::get_state_mean_at(const uint32_t k_in) const
// {
//     return x.at(k_in);
// }

// MatrixXd Kalman::get_state_covar_at(const uint32_t k_in) const
// {
//     return P.at(k_in);
// }


// const vector<MatrixXd>& Kalman::get_all_state_means(uint32_t* k_curr) const
// {
//     *k_curr = k;
//     return x;
// }

// const vector<MatrixXd>& Kalman::get_all_state_covars(uint32_t* k_curr) const
// {
//     *k_curr = k;
//     return P;
// }

