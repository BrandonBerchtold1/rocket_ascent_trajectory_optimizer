#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <string>

class Constants {
public:
    void loadFromJson(const std::string& filename);

    // Read-only accessors
    double get_G() const;
    double get_dt() const;
    double get_R_earth() const;
    double get_M_earth() const;

private:
    double G = 0.0;
    double dt = 0.0;
    double R_earth = 0.0;
    double M_earth = 0.0;
};

extern Constants CONSTANTS; // globally accessible instance

#endif //CONSTANTS_H