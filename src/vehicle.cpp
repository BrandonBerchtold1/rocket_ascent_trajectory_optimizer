#define _USE_MATH_DEFINES

#include "../headers/vehicle.h"
#include <math.h>
#include <utility>
#include <iostream>
#include <iomanip>
#include <stdio.h>
#include <SFML/Graphics.hpp>
#include <fstream>
#include "../headers/orbit_sim.h"
#include "../headers/constants.h"
#include "../include/json.hpp"

using json = nlohmann::json;

using namespace std;

Vehicle::Vehicle()
{
	std::ifstream file("inputs/SIMULATION_INPUTS.JSON");
	json simulation_inputs;
	file >> simulation_inputs;

	heading = simulation_inputs["heading"];
	pos[0] = simulation_inputs["initial_position_x"];
	pos[1] = simulation_inputs["initial_position_y"];
	vel[0] = simulation_inputs["initial_velocity_x"];
	vel[1] = simulation_inputs["initial_velocity_y"];
    acc[0] = simulation_inputs["initial_acceleration_x"];
	acc[1] = simulation_inputs["initial_acceleration_y"];
	throttle = simulation_inputs["throttle"];
	num_nozzles = simulation_inputs["stage_1_engine_count"];
	num_nozzles_stage_2 = simulation_inputs["stage_2_engine_count"];
	frontal_area = simulation_inputs["frontal_area"];
	frontal_area_stage_2 = simulation_inputs["frontal_area_stage_2"];
	m_dot = simulation_inputs["m_dot"];
	ve = simulation_inputs["ve"];
	ae = simulation_inputs["ae"];
	pe = simulation_inputs["pe"];
	m_dot_stage_2 = simulation_inputs["m_dot_stage_2"];
	ve_stage_2 = simulation_inputs["ve_stage_2"];
	ae_stage_2 = simulation_inputs["ae_stage_2"];
	pe_stage_2 = simulation_inputs["pe_stage_2"];
	Q_max_throttle_threshold = simulation_inputs["Q_max_throttle_threshold"];
	Max_Q_throttle_setting = simulation_inputs["Max_Q_throttle_setting"];
	double stage_1_wet_mass = simulation_inputs["stage_1_wet_mass"];
	double stage_1_dry_mass = simulation_inputs["stage_1_dry_mass"];
	double stage_2_wet_mass = simulation_inputs["stage_2_wet_mass"];
	double stage_2_dry_mass = simulation_inputs["stage_2_dry_mass"];
	double payload_mass = simulation_inputs["payload_mass"];

	prop_mass = stage_1_wet_mass - stage_1_dry_mass;
	dry_mass = stage_1_dry_mass + stage_2_wet_mass + stage_2_dry_mass + payload_mass;
	prop_mass_stage_2 = stage_2_wet_mass - stage_2_dry_mass;
	dry_mass_stage_2 = stage_2_dry_mass + payload_mass;

	latitude = simulation_inputs["latitude"];
	inclination = simulation_inputs["inclination"];
	circular_orbit_altitude = simulation_inputs["circular_orbit_altitude"];

	set_launch_azimuth();//Accounts for earth rotation, corrects initial velocity
}

//NEW POSITION CALC
void Vehicle::calc_next_position()
{
	double px_new = 0;
	double vx_new = 0;
	double ax_new = 0;

	double py_new = 0;
	double vy_new = 0;
	double ay_new = 0;

	vx_new = vel[0] + 0.5 * acc[0] * CONSTANTS.get_dt();
	vy_new = vel[1] + 0.5 * acc[1] * CONSTANTS.get_dt();

	px_new = pos[0] + vx_new * CONSTANTS.get_dt();
	py_new = pos[1] + vy_new * CONSTANTS.get_dt();

	std::pair<double, double> totalForce = calc_total_force(px_new, py_new, vx_new, vy_new);

	ax_new = totalForce.first / (prop_mass + dry_mass);
	ay_new = totalForce.second / (prop_mass + dry_mass);

	vx_new = vx_new + 0.5 * ax_new * CONSTANTS.get_dt();
	vy_new = vy_new + 0.5 * ay_new * CONSTANTS.get_dt();

	//Update rocket position, vel, and accel values.
	pos[0] = px_new;
	pos[1] = py_new;
	vel[0] = vx_new;
	vel[1] = vy_new;
	acc[0] = ax_new;
	acc[1] = ay_new;

	update_prop_mass();
}

//DRAG FORCE CALC
std::pair<double, double> Vehicle::calc_drag_force(double px, double py, double vx, double vy)
{
	//Subtract rotating atmopsphere (for earth rotation boost to initial velocity)
	double height = get_vector_magnitude(px, py);//From earth center

	double local_atmosphere_vel = initial_velocity_x * height / CONSTANTS.get_R_earth();

	vx = vx + local_atmosphere_vel;//Effectively rotates atmosphere to simulate rotating earth

	double v_magnitude = get_vector_magnitude(vx, vy);

	double unitVectorVx = 0;
	double unitVectorVy = 0;

	if (get_vector_magnitude(vx, vy) == 0)
	{
		unitVectorVx = 0;
		unitVectorVy = 0;
	}
	else
	{
		unitVectorVx = vx / v_magnitude;
		unitVectorVy = vy / v_magnitude;
	}

	double dragForceMag = 0.5 * get_rho(px, py) * v_magnitude * v_magnitude * get_cd(v_magnitude, px, py) * frontal_area;

	if ((0.5 * get_rho(px, py) * v_magnitude * v_magnitude) > Q_max_throttle_threshold)
		throttle = Max_Q_throttle_setting;
	else
		throttle = 1.0;

	double dragForceX = dragForceMag * (-1) * unitVectorVx;
	double dragForceY = dragForceMag * (-1) * unitVectorVy;

	double old_max_q = max_q;
	max_q = 0.5 * get_rho(px, py) * v_magnitude * v_magnitude;
	if (max_q < old_max_q)
		max_q = old_max_q;

	return std::make_pair(dragForceX, dragForceY);
}

//GRAVITY FORCE CALC
std::pair<double, double> Vehicle::calc_gravity_force(double px, double py)
{
	double gravityForceMag = CONSTANTS.get_G() * CONSTANTS.get_M_earth() * (dry_mass + prop_mass) / (pos[0] * pos[0] + pos[1] * pos[1]);

	//cout << "mass" << dry_mass + prop_mass << endl;

	double unitVectorPx = px / get_vector_magnitude(px, py);
	double unitVectorPy = py / get_vector_magnitude(px, py);

	double gravityForceX = gravityForceMag * (-1) * unitVectorPx;
	double gravityForceY = gravityForceMag * (-1) * unitVectorPy;

	//cout << "Gravity: " << sqrt(gravityForceX* gravityForceX + gravityForceY * gravityForceY) << endl;

	return std::make_pair(gravityForceX, gravityForceY);
}

//THRUST FORCE CALC
std::pair<double, double> Vehicle::calc_thrust_force(double px, double py, double throttle, double heading)
{
	//This is an approximation. Would need to include isentropic flow equations to properly do.
	double thrust_per_nozzle = (m_dot * ve + ae * (pe - get_P(px, py))) * throttle;
	
	double total_thrust = thrust_per_nozzle * num_nozzles;

	//cout << "Thrust: " << total_thrust << endl;

	double thrust_x = total_thrust * cos(heading / 180 * M_PI);
	double thrust_y = total_thrust * sin(heading / 180 * M_PI);

	return std::make_pair(thrust_x, thrust_y);//Update to account for atmosphere vacuum thrust
}

//TOTAL FORCE CALC
std::pair<double, double> Vehicle::calc_total_force(double px, double py, double vx, double vy)
{
	std::pair<double, double> dragForce = calc_drag_force(px, py, vx, vy);
	std::pair<double, double> gravityForce = calc_gravity_force(px, py);
	std::pair<double, double> thrustForce = calc_thrust_force(px, py, throttle, heading);

	double total_force_x = dragForce.first + gravityForce.first + thrustForce.first;
	double total_force_y = dragForce.second + gravityForce.second + thrustForce.second;

	return  std::make_pair(total_force_x, total_force_y);
}

//RHO CALC
double Vehicle::get_rho(double px, double py)//sort out non dimensional stuff
{
	double stratosphere_height = 25000;
	double troposphere_height = 11000;

	double T = 0;
	double P = 0;

	double h = get_vector_magnitude(px, py) - CONSTANTS.get_R_earth();

	if (h > stratosphere_height) {
		T = -131.21 + 0.00299 * h;
		P = 2.488 * pow(((T + 273.1) / 216.6), -11.388);
	}
	else if (h > troposphere_height && h < stratosphere_height) {
		T = -56.46;
		P = 22.65 * exp(1.73 - 0.000157 * h);
	}
	else {
		T = 15.04 - 0.00649 * h;
		P = 101.29 * pow(((T + 273.1) / 288.08), 5.256);
	}
	return P / (0.2869 * (T + 273.1));
}

//P_AMBIENT CALC
double Vehicle::get_P(double px, double py)
{
	double stratosphere_height = 25000;
	double troposphere_height = 11000;

	double T = 0;
	double P = 0;

	double h = get_vector_magnitude(px, py) - CONSTANTS.get_R_earth();

	if (h > stratosphere_height) {
		T = -131.21 + 0.00299 * h;
		P = 2.488 * pow(((T + 273.1) / 216.6), -11.388);
	}
	else if (h > troposphere_height && h < stratosphere_height) {
		T = -56.46;
		P = 22.65 * exp(1.73 - 0.000157 * h);
	}
	else {
		T = 15.04 - 0.00649 * h;
		P = 101.29 * pow(((T + 273.1) / 288.08), 5.256);
	}
	return P;
}

//CD CALC (UPDATE WITH OPENFOAM DATA FOR ACTUAL VEHICLE!!!, also factor in angle of attack and add lift function too)
double Vehicle::get_cd(double v_magnitude, double px, double py)
{
	double cd = 0;

	double stratosphere_height = 25000;
	double troposphere_height = 11000;

	double T = 0;
	double P = 0;

	double h = get_vector_magnitude(px, py) - CONSTANTS.get_R_earth();

	if (h > stratosphere_height) {
		T = -131.21 + 0.00299 * h;
		P = 2.488 * pow(((T + 273.1) / 216.6), -11.388);
	}
	else if (h > troposphere_height && h < stratosphere_height) {
		T = -56.46;
		P = 22.65 * exp(1.73 - 0.000157 * h);
	}
	else {
		T = 15.04 - 0.00649 * h;
		P = 101.29 * pow(((T + 273.1) / 288.08), 5.256);
	}
	
	double sonic_vel = 331 + (0.61 * T);

	double mach_number = v_magnitude / sonic_vel;

	if (mach_number < 0.6)
		return 0.15;
	else if (mach_number < 1.2)
		return 0.5 * mach_number - 0.15;
	else if (mach_number < 1.8)
		return -0.33333 * mach_number + 0.85;
	else if (mach_number < 5.0)
		return -0.03125 * mach_number + .30625;
	else
		return 0.15;
}

//VECTOR MAGNITUDE CALC
double Vehicle::get_vector_magnitude(double x, double y)
{
	return sqrt(x * x + y * y);
}

//CALC HEADING FROM VELOCITY
double Vehicle::calc_heading()
{
	return 180.0 / M_PI * atan2(vel[1], (vel[0] + initial_velocity_x));//note function takes (Y, X) unlike rest of code
}

//UPDATE MASS
void Vehicle::update_prop_mass()
{
	prop_mass = prop_mass - m_dot * num_nozzles * throttle * CONSTANTS.get_dt();
}

//Controls pitch over manouver and heading for entire simulation
void Vehicle::set_heading(float sim_time, float pitch_over_start_time, float pitch_over_duration, 
	float pitch_over_angle, float pitch_over_hold_time)
{
	if (sim_time < pitch_over_start_time)
		heading = 90;

	if (sim_time >= pitch_over_start_time && sim_time < (pitch_over_duration + pitch_over_start_time)) {
		heading = heading + pitch_over_angle / pitch_over_duration * CONSTANTS.get_dt();
	}

	if (sim_time >= (pitch_over_duration + pitch_over_start_time + pitch_over_hold_time))
		heading = calc_heading();//lock heading to velocity vector for remainder of flight
}

//Calculates current orbit apogee
double Vehicle::calc_apogee()
{
	double r_mag = get_vector_magnitude(pos[0], pos[1]);
	double v_mag = get_vector_magnitude(vel[0], vel[1]);
	double h = pos[0] * vel[1] - pos[1] * vel[0];
	double ex = vel[1] * h / (CONSTANTS.get_G() * CONSTANTS.get_M_earth()) - pos[0] / r_mag;
	double ey = -vel[0] * h / (CONSTANTS.get_G() * CONSTANTS.get_M_earth()) - pos[1] / r_mag;
	double e = sqrt(ex * ex + ey * ey);
	double perigee = h * h / (CONSTANTS.get_G() * CONSTANTS.get_M_earth()) * (1 / (1 + e));
	
	double a = h * h / (CONSTANTS.get_G() * CONSTANTS.get_M_earth()) * (1 / (1 - e * e));
	double apogee = abs(2 * a - perigee);

	return apogee;
}

//Calculates current orbit perigee
double Vehicle::calc_perigee()
{
	double r_mag = get_vector_magnitude(pos[0], pos[1]);
	double h = pos[0] * vel[1] - pos[1] * vel[0];
	double ex = vel[1] * h / (CONSTANTS.get_G() * CONSTANTS.get_M_earth()) - pos[0] / r_mag;
	double ey = -vel[0] * h / (CONSTANTS.get_G() * CONSTANTS.get_M_earth()) - pos[1] / r_mag;
	double e = sqrt(ex * ex + ey * ey);
	double p = h * h / (CONSTANTS.get_G() * CONSTANTS.get_M_earth()) * (1 / (1 + e));
	double perigee = abs(p);

	return perigee;
}

void Vehicle::clear_csv(const std::string& filename) {
	std::ofstream file(filename, std::ios::trunc); // Open in truncation mode
	if (!file) {
		std::cerr << "Error: Could not open " << filename << std::endl;
	}
	file.close(); // File is now empty
}

void Vehicle::calc_orbit_parameters()
{
	std::ofstream fileZ("outputs/ellipse_parameters.csv");

	clear_csv("outputs/ellipse_parameters.csv");

	double r_mag = get_vector_magnitude(pos[0], pos[1]);
	double v_mag = get_vector_magnitude(vel[0], vel[1]);
	double h = pos[0] * vel[1] - pos[1] * vel[0];
	double ex = vel[1] * h / (CONSTANTS.get_G() * CONSTANTS.get_M_earth()) - pos[0] / r_mag;
	double ey = -vel[0] * h / (CONSTANTS.get_G() * CONSTANTS.get_M_earth()) - pos[1] / r_mag;
	double e = sqrt(ex * ex + ey * ey);
	double eta = v_mag * v_mag / 2 - (CONSTANTS.get_G() * CONSTANTS.get_M_earth()) / r_mag;
	double a = -(CONSTANTS.get_G() * CONSTANTS.get_M_earth()) / (2 * eta);
	double b = a * sqrt(1 - e * e);
	double omega = atan2(ey, ex) - M_PI;

	fileZ << a << "\n";
	fileZ << b << "\n";
	fileZ << e << "\n";
	fileZ << omega << "\n";
	fileZ << h << "\n";
	fileZ << CONSTANTS.get_G() * CONSTANTS.get_M_earth() << "\n";
	fileZ.flush();

	fileZ.close();
}

//Performs hot staging manouver if propellant mass is zero.
void Vehicle::if_no_propellant_then_stage()
{
	if (prop_mass <= 0 && stage == 1)
	{
		prop_mass = prop_mass_stage_2;
		dry_mass = dry_mass_stage_2;
		num_nozzles = num_nozzles_stage_2;
		frontal_area = frontal_area_stage_2;
		m_dot = m_dot_stage_2;
		ve = ve_stage_2;
		ae = ae_stage_2;
		pe = pe_stage_2;
		stage = 2;
	}
}

void Vehicle::execute_time_step(float &sim_time, float pitch_over_start_time, 
	float pitch_over_duration, float pitch_over_angle, float pitch_over_hold_time)
{
	calc_next_position();
	set_heading(sim_time, pitch_over_start_time, pitch_over_duration, pitch_over_angle, pitch_over_hold_time);
	if_no_propellant_then_stage();

	sim_time = sim_time + CONSTANTS.get_dt();
}

void Vehicle::set_launch_azimuth()
{
	if (abs(inclination) < latitude)
	{
		cout << "Error, inclination is too low for this latitude!"
			"Inclination must be greater or equal to latitude. Inclination has been set to latitude." << endl;

		inclination = latitude;
	}

	//Convert to radians
	inclination = inclination / 180 * M_PI;
	latitude = latitude / 180 * M_PI;

	double azimuth_inertial = asin(cos(inclination) / cos(latitude));
	double orbit_velocity = sqrt(CONSTANTS.get_G() * CONSTANTS.get_M_earth() / (CONSTANTS.get_R_earth() + circular_orbit_altitude));
	double equator_earth_rotation_velocity = 465.101;
	double v_rotating_x = orbit_velocity * sin(azimuth_inertial) - equator_earth_rotation_velocity * cos(latitude);
	double v_rotating_y = orbit_velocity * cos(azimuth_inertial);
	double azimuth_rotating = atan(v_rotating_x / v_rotating_y);

	double v_rotating = sqrt(v_rotating_x * v_rotating_x + v_rotating_y * v_rotating_y);

	initial_velocity_x = orbit_velocity - v_rotating;//Velocity to be added to initial vehicle starting state
	
	earth_rotation_velocity_savings = initial_velocity_x;

	vel[0] = -initial_velocity_x;//Negative to account for coordinate frame of sim
}

double Vehicle::get_throttle()
{
	return throttle;
}

void Vehicle::set_throttle(double throttle_setting)
{
	throttle = throttle_setting;
}

int Vehicle::get_stage()
{
	return stage;
}

double Vehicle::get_pos_x()
{
	return pos[0];
}

double Vehicle::get_pos_y()
{
	return pos[1];
}

double Vehicle::get_prop_mass()
{
	return prop_mass;
}

double Vehicle::get_max_q()
{
	return max_q;
}

double Vehicle::get_circular_orbit_altitude()
{
	return circular_orbit_altitude;
}

bool Vehicle::do_hohmann()
{
	double target_h = sqrt(CONSTANTS.get_G() * CONSTANTS.get_M_earth() * (CONSTANTS.get_R_earth() + circular_orbit_altitude));
	double current_h = pos[0] * vel[1] - pos[1] * vel[0];
	double transfer_orbit_vel = current_h / (CONSTANTS.get_R_earth() + circular_orbit_altitude);//vehicle velocity at apogee
	double target_orbit_vel = target_h / (CONSTANTS.get_R_earth() + circular_orbit_altitude);//Vehicle velocity at orbit transfer point
	double delta_v = target_orbit_vel - transfer_orbit_vel;
	double v_eff = ve + (pe - get_P(pos[0], pos[1])) * ae / m_dot;

	if (stage == 1)
	{
		double ship_init_mass = dry_mass_stage_2 + prop_mass_stage_2 + dry_mass + prop_mass;
		double final_ship_mass = ship_init_mass / exp(delta_v / v_eff);
		prop_mass = prop_mass - (ship_init_mass - final_ship_mass);
		
		if (prop_mass < 0)
			return false;
		else
			return true;
	}
	else
	{
		double ship_init_mass = dry_mass + prop_mass;
		double final_ship_mass = ship_init_mass / exp(delta_v / v_eff);
		prop_mass = prop_mass - (ship_init_mass - final_ship_mass);
		if (prop_mass < 0)
			return false;
		else
			return true;
	}
}

double Vehicle::get_earth_rotation_velocity_savings()
{
	return earth_rotation_velocity_savings;
}