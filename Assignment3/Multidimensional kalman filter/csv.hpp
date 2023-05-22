#ifndef CSV_HPP
#define CSV_HPP

#include "Eigen/Dense"
#include <vector>

std::vector<std::vector<double>> csv_read(const std::string& filename);
double csv_getElementAt(const std::vector<std::vector<double>>& vec2D, uint32_t row, uint32_t col);

// helper
std::vector<Eigen::MatrixXd> convertToMatrixXd(const std::vector<std::vector<double>>& input);

#endif // CSV_HPP

