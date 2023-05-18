#include "kalman.hpp"

Kalman::Kalman(const uint8_t n, const uint8_t m, MatrixXd& x0, MatrixXd& P0) : 
    n(n), 
    m(m),
    k(0)
{
    x.push_back(x0);
    P.push_back(P0);
}

Kalman::~Kalman()
{

}

bool Kalman::predict(MatrixXd& F, MatrixXd& Q)
{
    MatrixXd xp(n, 1);
    MatrixXd Pp(n, n);

    xp << F * x.at(k);
    Pp << (F * P.at(k) * F.transpose()) + Q;
    k++;

    x.push_back(xp);
    P.push_back(Pp);
    
    return true;
}

bool Kalman::update(MatrixXd& H, MatrixXd& R, MatrixXd& z)
{
    MatrixXd xm(n, 1);
    MatrixXd Pm(n, n);

    MatrixXd y(m, 1);
    MatrixXd S(m, m);
    MatrixXd K(n, m);

    y << z - (H * x.at(k));
    S << (H * P.at(k) * H.transpose()) + R;
    K << P.at(k) * H.transpose() * S.inverse();
    
    xm << x.at(k) + (K * y);
    Pm << P.at(k) - (K * H * P.at(k));

    x.at(k) << xm;
    P.at(k) << Pm;
    return true;
}

bool Kalman::step(MatrixXd& F, MatrixXd& Q, MatrixXd& H, MatrixXd& R, MatrixXd& z)
{
    return predict(F, Q) && update(H, R, z);
}


bool Kalman::get_state_mean_at(const uint32_t k_in, MatrixXd& x_out) const
{
    if(k_in > k)
    {
        return false;
    }
    x_out.resize(n, 1);
    x_out << x.at(k_in);
    return true;
}

bool Kalman::get_state_covar_at(const uint32_t k_in, MatrixXd& P_out) const
{
    if(k_in > k)
    {
        return false;
    }
    P_out.resize(n, n);
    P_out << P.at(k_in);
    return true;
}

