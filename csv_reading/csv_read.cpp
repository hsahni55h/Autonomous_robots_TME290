#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>

// Function to read a CSV file and return the data as a 2D vector of doubles
std::vector<std::vector<double>> readCSV(const std::string& filename) {
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
double getElementAt(const std::vector<std::vector<double>>& vec2D, uint32_t row, uint32_t col) {
    if (row < vec2D.size() && col < vec2D[row].size()) {
        return vec2D[row][col];
    } else {
        std::cout << "Invalid row or column index." << std::endl;
        return 0.0; // Return 0.0 if the indices are out of range
    }
}

int main() {
    std::string filename = "gnss-1.csv";
    std::vector<std::vector<double>> data = readCSV(filename);

    // Print the contents of the CSV file
    for (std::size_t row = 0; row < data.size(); ++row) {
        for (std::size_t col = 0; col < data[row].size(); ++col) {
            std::cout << getElementAt(data, row, col) << "\t";
        }
        std::cout << std::endl;
    }
    return 0;
}
