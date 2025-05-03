#ifndef VEHICLECLASS_H
#define VEHICLECLASS_H

#include <utility>
#include <SFML/Graphics.hpp>

class Vehicle {
public:
	Vehicle();

	void calc_next_position();

	//DRAG FORCE CALC
	std::pair<double, double> calc_drag_force(double px, double py, double vx, double vy);

	//GRAVITY FORCE CALC
	std::pair<double, double> calc_gravity_force(double px, double py);

	//THRUST FORCE CALC
	std::pair<double, double> calc_thrust_force(double px, double py, double throttle, double heading);

	//TOTAL FORCE CALC
	std::pair<double, double> calc_total_force(double px, double py, double vx, double vy);

	//RHO CALC
	double get_rho(double px, double py);

	//P_AMBIENT CALC
	double get_P(double px, double py);

	//CD CALC
	double get_cd(double v_magnitude, double px, double py);

	//VECTOR MAGNITUDE CALC
	double get_vector_magnitude(double x, double y);

	//CALC HEADING FROM VELOCITY
	double calc_heading();

	//UPDATE MASS
	void update_prop_mass();

	//Controls pitch over manouver and heading for entire simulation
	void set_heading(float sim_time, float pitch_over_start_time, float pitch_over_duration, 
		float pitch_over_angle, float pitch_over_hold_time);

	//Calculates current orbit apogee
	double calc_apogee();

	//Calculates current orbit perigee
	double calc_perigee();

	//CALCULATE ORBIT PARAMETERS
	void calc_orbit_parameters();

	//Performs hot staging manouver
	void if_no_propellant_then_stage();

	//Calculates next position, increments sim_time, sets heading, does staging if no propellant, logs data
	void execute_time_step(float &sim_time, float pitch_over_start_time, float pitch_over_duration, 
		float pitch_over_angle, float pitch_over_hold_time);

	void clear_csv(const std::string& filename);

	double get_throttle();

	void set_launch_azimuth();
	
	void set_throttle(double throttle_setting);

	int get_stage();

	double get_pos_x();

	double get_pos_y();

	double get_prop_mass();

	double get_max_q();

	double get_circular_orbit_altitude();

	bool do_hohmann();

	double get_earth_rotation_velocity_savings();

private:
	double heading = 0;//deg
	double pos[2] = { 0 };//m
	double vel[2] = { 0 };//m/s
	double acc[2] = { 0 };//m/s^2
	double prop_mass = 0;//kg
	double dry_mass = 10000;//kg
	double prop_mass_stage_2 = 0;//kg
	double dry_mass_stage_2 = 10000;//kg
	double throttle = 0;//0-1.0
	int num_nozzles = 1;
	int num_nozzles_stage_2 = 1;
	double frontal_area = 3;//m^2
	double frontal_area_stage_2 = 3;//m^2
	double m_dot = 305;// 30.7;//kg/s
	double ve = 2951;// 3535.85;//m/s
	double ae = 0.9;// 0.14083;//m^2
	double pe = 40000;// 40530;//Pa
	double m_dot_stage_2 = 305;// 30.7;//kg/s
	double ve_stage_2 = 2951;// 3535.85;//m/s
	double ae_stage_2 = 0.9;// 0.14083;//m^2
	double pe_stage_2 = 40000;// 40530;//Pa
	int stage = 1;
	double Q_max_throttle_threshold = 0;
	double Max_Q_throttle_setting = 0;
	double latitude = 0;
	double inclination = 0;
	double circular_orbit_altitude = 0;
	double initial_velocity_x = 0;
	double max_q = 0;
	double earth_rotation_velocity_savings = 0;
};

#endif // VEHICLECLASS_H