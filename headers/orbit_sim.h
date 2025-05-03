#ifndef ORBITSIM_H
#define ORBITSIM_H

#include <SFML/Graphics.hpp>
#include <fstream>
#include <iostream>
#include "vehicle.h"

//Yes I know these variables should probably all be private... Will update soon.

class orbit_sim {
public:
    orbit_sim();
    sf::VertexArray draw_orbit_ellipse(double earth_origin_x, double earth_origin_y, double earth_rad,
        double zoom, double a, double b, double e, double omega);
    void run_sim();
    void run();
    void read_sim_data_file();
    void clear_csv(const std::string& filename);
    void load_settings_from_JSON();
    void update_text(double pitch_over_angle, double apogee, double perigee, double propellant_remaining, 
        bool converged, double earth_rotation_velocity_savings);

    float zoom = 1.0;
    float zoom_delta = 0.25;
    int earth_rad = 500;
    float earth_origin_x = 3840 / 2 - earth_rad;
    float earth_origin_y = 2160 / 2 - earth_rad;

    sf::CircleShape earth;
    sf::CircleShape target_orbit;

    sf::CircleShape seco;
    sf::CircleShape meco;
    sf::CircleShape ship_point;
 
    //STRAFE VARIABLES
    bool isDragging = false;
    sf::Vector2i initial_pos = { 0, 0 };
    double original_earth_origin_x = 0;
    double original_earth_origin_y = 0;

    double x1 = 0;
    double y1 = 0;
    double x2 = 0;
    double y2 = 0;
    double ship_point_x = 0;
    double ship_point_y = 0;

    std::vector<sf::Vector2f> points;//Vector to store flight path points
    int previous_flight_point_count = 0;
    int current_flight_point_count = 0;
    double num1 = 0;
    double num2 = 0;

    double a = 0;
    double b = 0;
    double e = 0;
    double omega = 0;
    double apogee = 0;
    double perigee = 0;
    double propellant_remaining = 0;

    float pitch_over_angle = 4.5;
    float sim_time = 0;
    float pitch_over_start_time = 4.5;
    float pitch_over_duration = 5;
    float pitch_over_hold_time = 10;
    double Q_max_throttle_threshold = 40000;
    double Max_Q_throttle_setting = 0.7;
    double dt = 0.005;
    double earth_rotation_velocity_savings = 0;
    float target_circular_orbit = 0;
    bool converged = false;
    bool is_propellant_left_over = true;
    bool itteration_complete = false;

private:
    void handle_events();
    void render();
    sf::RenderWindow window;
    sf::Font font;

    double max_q = 0;
};

#endif //ORBITSIM_H