#define _USE_MATH_DEFINES

#include "../headers/orbit_sim.h"
#include <SFML/Graphics.hpp>
#include <windows.h>
#include <fstream>
#include "../headers/vehicle.h"
#include <iostream>
#include <iomanip>
#include "../headers/constants.h"
#include "../include/json.hpp"
#include <sstream>
#include <math.h>

using json = nlohmann::json;

using namespace std;

orbit_sim::orbit_sim()
    : window(sf::VideoMode({ 3840, 2160 }), "My window", sf::State::Fullscreen) {

    load_settings_from_JSON();

    //Create initial earth
    earth.setRadius(earth_rad);
    earth.setPosition({ (float)earth_origin_x, (float)earth_origin_y });
    earth.setFillColor(sf::Color(0, 0, 0));
    earth.setOutlineColor(sf::Color(255, 255, 255));
    earth.setOutlineThickness(-1);
    earth.setPointCount(1000);

    //Create target earth ORBIT
    target_orbit.setRadius((earth_rad + target_circular_orbit / 12756) * zoom);
    target_orbit.setPosition({ earth_origin_x - target_circular_orbit / 12756 * zoom, earth_origin_y - target_circular_orbit / 12756 * zoom });
    target_orbit.setFillColor(sf::Color(0, 0, 0));
    target_orbit.setOutlineColor(sf::Color(0, 255, 0));
    target_orbit.setOutlineThickness(1);
    target_orbit.setPointCount(1000);

    if (!font.openFromFile("include/arial.ttf")) {
        throw std::runtime_error("Failed to load font.");
    }
}

sf::VertexArray orbit_sim::draw_orbit_ellipse(double earth_origin_x, double earth_origin_y, double earth_rad,
    double zoom, double a, double b, double e, double omega)
{
    int points = 1000;

    sf::VertexArray ellipse(sf::PrimitiveType::LineStrip, points + 1); // +1 to close the loop

    for (int i = 0; i <= points; ++i) {
        float angle = (2 * M_PI * i) / points; // Angle for the ellipse point

        // Parametric equation of an ellipse
        float x = a * cos(angle);
        float y = b * sin(angle);

        // Rotate the point using a 2D rotation matrix
        float rotatedX = x * cos(omega) - y * sin(omega);
        float rotatedY = x * sin(omega) + y * cos(omega);

        // Store in the vertex array
        ellipse[i].position = sf::Vector2f((float)(earth_origin_x + earth_rad * zoom +
            (rotatedX + a * e * cos(omega)) * zoom / 12756), (float)(earth_origin_y +
                earth_rad * zoom - (rotatedY + a * e * sin(omega)) * zoom / 12756));
        ellipse[i].color = sf::Color::Magenta;
    }

    return ellipse;
}

void orbit_sim::handle_events() {
    // check all the window's events that were triggered since the last iteration of the loop
    while (const std::optional event = window.pollEvent())
    {
        if (event->is<sf::Event::Closed>())
        {
            window.close();
        }
        else if (const auto* keyPressed = event->getIf<sf::Event::KeyPressed>())
        {
            if (keyPressed->scancode == sf::Keyboard::Scancode::Escape)
                window.close();
        }

        //update zoom if mouse wheel scrolled
        if (const auto* mouseWheelScrolled = event->getIf<sf::Event::MouseWheelScrolled>())
        {
            if (mouseWheelScrolled->delta > 0)
                zoom = zoom * (1 + zoom_delta);

            if (mouseWheelScrolled->delta < 0)
                zoom = zoom / (1 + zoom_delta);

            if (mouseWheelScrolled->delta > 0)//zoom in
            {
                earth_origin_x = mouseWheelScrolled->position.x - (mouseWheelScrolled->position.x - (earth_origin_x)) * (1 + zoom_delta);
                earth_origin_y = mouseWheelScrolled->position.y - (mouseWheelScrolled->position.y - (earth_origin_y)) * (1 + zoom_delta);
            }
            else//zoom out
            {
                earth_origin_x = mouseWheelScrolled->position.x - (mouseWheelScrolled->position.x - (earth_origin_x)) / (1 + zoom_delta);
                earth_origin_y = mouseWheelScrolled->position.y - (mouseWheelScrolled->position.y - (earth_origin_y)) / (1 + zoom_delta);
            }

            earth.setRadius(earth_rad * zoom);
            earth.setPosition({ earth_origin_x, earth_origin_y });

            target_orbit.setRadius((earth_rad + target_circular_orbit / 12756) * zoom);
            target_orbit.setPosition({ earth_origin_x - target_circular_orbit / 12756 * zoom, earth_origin_y - target_circular_orbit / 12756 * zoom });
        }

        //Click and drag screen
        if (sf::Mouse::isButtonPressed(sf::Mouse::Button::Left))
        {
            if (isDragging == false)
            {
                isDragging = true;

                initial_pos = sf::Mouse::getPosition();
                original_earth_origin_x = earth_origin_x;
                original_earth_origin_y = earth_origin_y;
            }

            earth_origin_x = original_earth_origin_x + sf::Mouse::getPosition().x - initial_pos.x;
            earth_origin_y = original_earth_origin_y + sf::Mouse::getPosition().y - initial_pos.y;

            earth.setPosition({ earth_origin_x, earth_origin_y });
            target_orbit.setPosition({ earth_origin_x - target_circular_orbit / 12756 * zoom, earth_origin_y - target_circular_orbit / 12756 * zoom });
        }
        else
        {
            isDragging = false;
        }
    }
}

void orbit_sim::clear_csv(const std::string & filename) {
    std::ofstream file(filename, std::ios::trunc); // Open in truncation mode
    if (!file) {
        std::cerr << "Error: Could not open " << filename << std::endl;
    }
    file.close(); // File is now empty
}

void orbit_sim::run_sim() {

    std::ofstream fileX("outputs/xy_positions.csv");
    std::ofstream fileY("outputs/stage_coords.csv");

    clear_csv("outputs/xy_positions.csv");
    clear_csv("outputs/stage_coords.csv");

    //Reset sim variables
    sim_time = 0;
    Vehicle ship;
    ship.set_throttle(1.0); //ship.throttle = 1;//Set throttle to 100%
    bool meco_complete = false;
    bool seco_complete = false;
    int i = 0;//Needed for live draw

    //Clears all but previous flight data.
    previous_flight_point_count = current_flight_point_count;
    current_flight_point_count = 0;
    if (points.size() > previous_flight_point_count) {
        points.erase(points.begin(), points.begin() + (points.size() - previous_flight_point_count) + 1);
    }

    earth_rotation_velocity_savings = ship.get_earth_rotation_velocity_savings();

    //LOOP UNITL APOGEE IS 500KM
    //sim_time < 40 is to prevent 90° initial flight creating an eccentricity of e=1 which results in apogee going to infinity 
    // and ending this loop, waiting for sim_time = 40 sec lets some horizontal velocity to accumulate and create an e < 1.000000
    while ((sim_time < 40 || ship.calc_apogee() <= (target_circular_orbit + CONSTANTS.get_R_earth())) && (sqrt(ship.get_pos_x()* ship.get_pos_x()+ ship.get_pos_y()* ship.get_pos_y())) >= (CONSTANTS.get_R_earth() - 100) && (ship.get_prop_mass() > 0 || ship.get_stage() == 1))
    {
        ship.execute_time_step(sim_time, pitch_over_start_time, pitch_over_duration, pitch_over_angle, pitch_over_hold_time);

        //Save MECO point
        if (ship.get_stage() == 2 && meco_complete == false)
        {
            fileY << std::setprecision(15) << ship.get_pos_x() << "\t";
            fileY << std::setprecision(15) << ship.get_pos_y() << "\n";
            fileY.flush();
            meco_complete = true;
        }

        //Save every 200 timestep position to file (1 per sec)
        if (i % 200 == 0) {
            fileX << std::setprecision(15) << ship.get_pos_x() << "\t";
            fileX << std::setprecision(15) << ship.get_pos_y() << "\n";
            fileX.flush();
        }

        ship_point_x = ship.get_pos_x();
        ship_point_y = ship.get_pos_y();
        points.push_back(sf::Vector2f(ship_point_x, ship_point_y));//might be an issue with float since 6378000+0.01 = 6378000
        current_flight_point_count++;

        if (i % 200000 == 0) {//The number after the % affects sim speed. Bigger is faster since rendering is bottleneck!
            ship.calc_orbit_parameters();

            std::ifstream fileZ("outputs/ellipse_parameters.csv");  // Open the CSV file
            if (!fileZ.is_open()) {
                std::cerr << "Failed to open file!" << std::endl;
            }

            //Read in ellipse data
            fileZ >> a;
            fileZ >> b;
            fileZ >> e;
            fileZ >> omega;
                
            render();
            handle_events();
        }
        i++;
    }

    is_propellant_left_over = ship.do_hohmann();//executed hohmann transfer

    max_q = ship.get_max_q();
    
    //IF FLIGHT ENDS WITH STAGE 1, SAVE MECO HERE
    if (ship.get_stage() == 1)
    {
        fileY << ship.get_pos_x() << "\t";
        fileY << ship.get_pos_y() << "\t";
    }

    //save SECO point
    fileY << ship.get_pos_x() << "\t";
    fileY << ship.get_pos_y() << "\t";
    fileY << ship.calc_apogee() - CONSTANTS.get_R_earth() << "\t";
    fileY << ship.calc_perigee() - CONSTANTS.get_R_earth() << "\t";
    fileY << ship.get_prop_mass() << "\n";
    fileY.flush();

    ship.set_throttle(0.0);

    ship.calc_orbit_parameters();

    fileX.close();
    fileY.close();
}

void orbit_sim::render() {
    window.clear();
    window.draw(target_orbit);
    window.draw(earth);

    sf::VertexArray dataPoint(sf::PrimitiveType::Points, points.size());

    int k = 0;

    //updates point positions for orbit
    double var1 = earth_origin_x + earth_rad * zoom;
    double var2 = zoom / 12756;
    double var3 = earth_origin_y + earth_rad * zoom;

    for (const auto& point : points) {

        dataPoint[k].color = sf::Color::Red;  // Set circle color
        dataPoint[k].position = { (float)(var1 + point.x * var2), (float)(var3 - point.y * var2) };// Set circle position to the point
        k++;
    }

    //Updates meco and seco point locations for drag and zoom changes
    meco.setPosition({ (float)(var1 + x1 * var2 - 3), (float)(var3 - y1 * var2 - 3) });
    seco.setPosition({ (float)(var1 + x2 * var2 - 3), (float)(var3 - y2 * var2 - 3) });
    ship_point.setPosition({ (float)(var1 + ship_point_x * var2 - 3), (float)(var3 - ship_point_y * var2 - 3) });

    window.draw(seco);
    window.draw(meco);
    window.draw(ship_point);
    window.draw(draw_orbit_ellipse(earth_origin_x, earth_origin_y, earth_rad, zoom, a, b, e, omega));
    window.draw(dataPoint);
    update_text(pitch_over_angle, apogee, perigee, propellant_remaining, converged, earth_rotation_velocity_savings);
    window.display();
}

void orbit_sim::read_sim_data_file() {
    //Open data files
     std::ifstream fileY("outputs/stage_coords.csv");  // Open the CSV file
    if (!fileY.is_open()) {
        std::cerr << "Failed to open file!" << std::endl;
    }

    std::ifstream fileZ("outputs/ellipse_parameters.csv");  // Open the CSV file
    if (!fileZ.is_open()) {
        std::cerr << "Failed to open file!" << std::endl;
    }

    //READ IN MECO AND SECO COORDINATES
    fileY >> x1;
    fileY >> y1;

    //Read in ellipse data
    fileZ >> a;
    fileZ >> b;
    fileZ >> e;
    fileZ >> omega;

    //Create MECO POINT
    meco.setRadius(3);
    meco.setFillColor(sf::Color(0, 255, 0));

    fileY >> x2;
    fileY >> y2;

    //Create SECO POINT
    seco.setRadius(3);
    seco.setFillColor(sf::Color(255, 255, 0));

    //Create ship_point
    ship_point.setRadius(3);
    ship_point.setFillColor(sf::Color(255, 0, 0));

    fileY >> apogee;
    fileY >> perigee;
    fileY >> propellant_remaining;

    fileY.close();
    fileZ.close();
}

void orbit_sim::run() {
    CONSTANTS.loadFromJson("inputs/SIMULATION_INPUTS.JSON");
    
    run_sim();//Run initial default simulation
    read_sim_data_file();//Read datafile generated for initial default simulation
    
    std::cout << "Pitch over Angle: " << std::setprecision(5) << pitch_over_angle << std::endl;
    std::cout << "Apogee: " << apogee << std::endl;
    std::cout << "Perigee: " << perigee << std::endl;
    std::cout << "Propellant Remaining (Kg): " << propellant_remaining << std::endl;
    std::cout << "**************************************************" << std::endl << std::endl;

    double delta_pitch_scale = 0.050;
    double perigee1 = perigee;
    
    while (window.isOpen()) {
        while (delta_pitch_scale > 0.00001)
        {
            double pitch_over_angle_previous = pitch_over_angle;
            pitch_over_angle = pitch_over_angle + delta_pitch_scale;
            
            handle_events();
            run_sim();//Run initial default simulation
            read_sim_data_file();//Read datafile generated for initial default simulation
            render();
            
            std::cout << "Pitch over Angle: " << std::setprecision(5) << pitch_over_angle << std::endl;
            std::cout << "Apogee: " << apogee << std::endl;
            std::cout << "Perigee: " << perigee << std::endl;
            std::cout << "Propellant Remaining (Kg): " << propellant_remaining << std::endl;
            std::cout << "**************************************************" << std::endl << std::endl;

            if (perigee > perigee1)
            {
                perigee1 = perigee;
            }

            if (perigee < perigee1)
            {
                pitch_over_angle = pitch_over_angle_previous;
                delta_pitch_scale = delta_pitch_scale * 0.5;
            }
        }

        if (perigee > 0)
            converged = true;
        else
            converged = false;

        handle_events();
        render();
    }
}

void orbit_sim::load_settings_from_JSON()
{
    std::ifstream file("inputs/SIMULATION_INPUTS.JSON");
    json simulation_inputs;
    file >> simulation_inputs;

    pitch_over_angle = simulation_inputs["pitch_over_angle"];
    pitch_over_start_time = simulation_inputs["pitch_over_start_time"];
    pitch_over_duration = simulation_inputs["pitch_over_duration"];
    pitch_over_hold_time = simulation_inputs["pitch_over_hold_time"];
    target_circular_orbit = simulation_inputs["circular_orbit_altitude"];
}

void orbit_sim::update_text(double pitch_over_angle, double apogee, double perigee, double propellant_remaining, 
    bool converged, double earth_rotation_velocity_savings)
{
    std::ostringstream oss;

    oss << std::fixed << std::setprecision(5) << pitch_over_angle;
    sf::Text text1(font, "Pitch over angle [deg]: " + oss.str());

    text1.setFont(font);
    text1.setCharacterSize(30);  // Set default size
    text1.setFillColor(sf::Color::White); // Default text color
    text1.setPosition({ 100, 100 }); // Default position

    oss.str("");// Clear the string buffer
    oss.clear();
    oss << std::fixed << std::setprecision(0) << apogee;
    sf::Text text2(font, "Transfer orbit apogee [m]: " + oss.str());

    text2.setFont(font);
    text2.setCharacterSize(30);  // Set default size
    text2.setFillColor(sf::Color::White); // Default text color
    text2.setPosition({ 100, 140 }); // Default position

    oss.str("");// Clear the string buffer
    oss.clear();
    oss << std::fixed << std::setprecision(0) << perigee;
    sf::Text text3(font, "Transfer orbit perigee [m]: " + oss.str());

    text3.setFont(font);
    text3.setCharacterSize(30);  // Set default size
    text3.setFillColor(sf::Color::White); // Default text color
    text3.setPosition({ 100, 180 }); // Default position

    oss.str("");// Clear the string buffer
    oss.clear();
    oss << std::fixed << std::setprecision(1) << propellant_remaining;
    sf::Text text4(font, "Propellant Remaining [kg]: " + oss.str());

    text4.setFont(font);
    text4.setCharacterSize(30);  // Set default size
    text4.setFillColor(sf::Color::White); // Default text color
    text4.setPosition({ 100, 220 }); // Default position

    if (converged == true && is_propellant_left_over == true)
    {
        oss.str("");// Clear the string buffer
        oss.clear();
        oss << std::fixed << std::setprecision(5) << pitch_over_angle;
        sf::Text text5(font, "SIMULATION CONVERGED!! Optimal pitch over angle is: " + oss.str());

        text5.setFont(font);
        text5.setCharacterSize(30);  // Set default size
        text5.setFillColor(sf::Color::Green); // Default text color
        text5.setPosition({ 100, 300 }); // Default position
        window.draw(text5);

        oss.str("");// Clear the string buffer
        oss.clear();
        oss << std::fixed << std::setprecision(0) << target_circular_orbit / 1000 << " X ";
        oss << std::fixed << std::setprecision(0) << target_circular_orbit / 1000 << " [km], Eccentricity: ";
        oss << std::fixed << std::setprecision(3) << 0.000;
        sf::Text text8(font, "Final orbit: " + oss.str());

        text8.setFont(font);
        text8.setCharacterSize(30);  // Set default size
        text8.setFillColor(sf::Color::Green); // Default text color
        text8.setPosition({ 100, 340 }); // Default position
        window.draw(text8);
    }

    if (converged == true && is_propellant_left_over == false)
    {
        sf::Text text9(font, "Propellant ran out before target orbit was achieved!!!");

        text9.setFont(font);
        text9.setCharacterSize(30);  // Set default size
        text9.setFillColor(sf::Color::Red); // Default text color
        text9.setPosition({ 100, 340 }); // Default position
        window.draw(text9);
    }

    if (converged == false)
    {
        //failed to achieve orbit
    }

    oss.str("");// Clear the string buffer
    oss.clear();
    oss << std::fixed << std::setprecision(1) << max_q;
    sf::Text text6(font, "Max Q [Pa]: " + oss.str());

    text6.setFont(font);
    text6.setCharacterSize(30);  // Set default size
    text6.setFillColor(sf::Color::White); // Default text color
    text6.setPosition({ 100, 260 }); // Default position

    oss.str("");// Clear the string buffer
    oss.clear();
    oss << std::fixed << std::setprecision(1) << earth_rotation_velocity_savings;
    sf::Text text7(font, "Earth rotation delta V savings [m/s]: " + oss.str());

    text7.setFont(font);
    text7.setCharacterSize(30);  // Set default size
    text7.setFillColor(sf::Color::White); // Default text color
    text7.setPosition({ 100, 60}); // Default position

    window.draw(text1);
    window.draw(text2);
    window.draw(text3);
    window.draw(text4);
    window.draw(text6);
    window.draw(text7);
}