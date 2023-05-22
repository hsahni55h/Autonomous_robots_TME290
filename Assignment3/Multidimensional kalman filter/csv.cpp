#include "csv.hpp"

#include <iostream>
#include <fstream> // Include the necessary header file
#include <string>
#include <vector>
#include <sstream>

// Function to read a CSV file and return the data as a 2D vector of doubles

std::vector<std::vector<double>> csv_read(const std::string& filename) {
    std::ifstream file(filename);

    if (!file.is_open()) {
        std::cout << "Error opening the file." << std::endl;
        return {}; // Return an empty vector indicating an error
    }

    std::vector<std::vector<double>> csvData;

    std::string line;
    bool firstRow = true; // Flag to skip conversion for the first row (header row)
    while (std::getline(file, line)) {
        std::vector<double> row;
        std::string cell;
        std::stringstream lineStream(line);

        while (std::getline(lineStream, cell, ',')) {
            // Skip conversion for the first row (header row)
            if (!firstRow) {
                try {
                    row.push_back(std::stod(cell)); // Convert cell value to double and add to the row
                } catch (const std::exception& e) {
                    // Handle conversion errors if necessary
                    std::cout << "Error converting to double: " << e.what() << std::endl;
                }
            }
        }

        if (!firstRow) {
            csvData.push_back(row); // Add the row to the CSV data
        }

        firstRow = false;
    }

    file.close();

    return csvData;
}

// Function to access an element in a 2D vector
double csv_getElementAt(const std::vector<std::vector<double>>& vec2D, uint32_t row, uint32_t col) {
    if (row < vec2D.size() && col < vec2D[row].size()) {
        return vec2D[row][col];
    } else {
        std::cout << "Invalid row or column index." << std::endl;
        return 0.0; // Return 0.0 if the indices are out of range
    }
}

std::vector<Eigen::MatrixXd> convertToMatrixXd(const std::vector<std::vector<double>>& input) {
    std::vector<Eigen::MatrixXd> output;

    // Iterate over the input vector
    for (const auto& row : input) {
        // Convert each row into an Eigen MatrixXd object
        Eigen::MatrixXd matrix = Eigen::Map<const Eigen::MatrixXd>(row.data(), 1, row.size());
        output.push_back(matrix);
    }

    return output;
}

