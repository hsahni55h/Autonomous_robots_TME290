#include <iostream>
#include "kalman.hpp"
#include "csv.hpp"

using namespace std;

#define DT 0.1

#define F_MATRIX    1, DT, \
                    0, 1

#define Q_MATRIX    0, 0, \
                    0, 0

#define H_MATRIX    1, 0

#define R_MATRIX    2


int main(void)
{
    // read the csv files and load the data
    string filename;
    filename = "groundtruth-1.csv";
    vector<vector<double>> groundtruth = csv_read(filename);
    filename = "gnss-1.csv";
    vector<vector<double>> gnss = csv_read(filename);
    filename = "wheelspeeds-1.csv";
    vector<vector<double>> wheelspeeds = csv_read(filename);

    // state - [V phi X Y]
    // measurement - [V X Y]

    // convert the 2d vectors for gnss and wheelspeeds into MatrixXd
    

    return 0;
}



/**
 * 
 * ----- state - [V phi X Y] -----
 * V = (ws.r + ws.l) / 2 = K1
 * phi = phi + [ (ws.r - ws.l) / 2 ] * dt = phi + dphi
 * X = X + V * cos(phi) * dt
 * Y = Y + V * sin(phi) * dt
 * 
 * F = 
 *      K1           0               0       0
 *      0            1+dphi/phi      0       0  
 *      c(phi)*dt    0               1       0
 *      s(phi)*dt    0               0       1
 * 
 * 
 * 
 * ----- measurement - [V X Y] -----
 * H = 
 *      1    0   0   0
 *      0    0   1   0
 *      0    0   0   1
 * 
 */

