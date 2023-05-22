#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>

int main() {
    std::ifstream file("gnss-1.csv");

    if (!file.is_open()) {
        std::cout << "Error opening the file." << std::endl;
        return 1; // Exit the program indicating an error
    }

    std::vector<std::vector<std::string>> csvData;

    std::string line;
    while (std::getline(file, line)) {
        std::vector<std::string> row;
        std::string cell;
        std::stringstream lineStream(line);

        while (std::getline(lineStream, cell, ',')) {
            row.push_back(cell);
        }

        csvData.push_back(row);
    }

    file.close();

    for (const auto& row : csvData) {
        for (const auto& cell : row) {
            std::cout << cell << "\t";
        }
        std::cout << std::endl;
    }

    return 0;
}
