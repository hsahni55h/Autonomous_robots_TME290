#include <vector>
#include "Core" // Eigen Library
#include "LU"   // Eigen Library

using namespace std;
using namespace Eigen;

class Kalman
{
    private:
        uint8_t n;              // number of states
        uint8_t m;              // number of sensors
        uint32_t k;             // current time step
        vector<MatrixXd> x;     // state mean
        vector<MatrixXd> P;     // state covariance
    
    public:
        Kalman(const uint8_t n, const uint8_t m, MatrixXd& x0, MatrixXd& P0);
        ~Kalman();
        bool predict(MatrixXd& F, MatrixXd& Q);
        bool update(MatrixXd& H, MatrixXd& R, MatrixXd& z);
        bool step(MatrixXd& F, MatrixXd& Q, MatrixXd& H, MatrixXd& R, MatrixXd& z);

        bool get_state_mean_at(const uint32_t k_in, MatrixXd& x_out) const;
        bool get_state_covar_at(const uint32_t k_in, MatrixXd& P_out) const;
        inline uint32_t get_current_step() const {return k;}
};

