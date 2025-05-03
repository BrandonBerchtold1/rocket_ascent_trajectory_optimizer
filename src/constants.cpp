#include "../headers/constants.h"
#include "../include/json.hpp"
#include <fstream>
#include <iostream>

using json = nlohmann::json;
using namespace std;

Constants CONSTANTS;

void Constants::loadFromJson(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error opening " << filename << std::endl;
        return;
    }

    json simulation_inputs;
    file >> simulation_inputs;

    G = 6.67430e-11;//[N*m^2/kg^2]
    dt = simulation_inputs["dt"];//[sec]
    R_earth = simulation_inputs["R_earth"];//[m]
    M_earth = simulation_inputs["M_earth"];//[kg]
}

// Read-only accessors
double Constants::get_G() const { return G; }
double Constants::get_dt() const { return dt; }
double Constants::get_R_earth() const { return R_earth; }
double Constants::get_M_earth() const { return M_earth; }