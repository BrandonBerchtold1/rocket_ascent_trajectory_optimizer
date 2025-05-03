# rocket_ascent_trajectory_optimizer
Optimization code and visualizer to plan optimal ascent trajectory for arbitrary 2 stage rocket to arbitrary circular earth orbit.

# To use this rocket flight optimizer, do the following steps:

TLDR:
1) Navigate to Rocket "rocket_ascent_trajectory_optimizer\x64\Debug"
2) Copy all contents and paste to "rocket_ascent_trajectory_optimizer"
3) Navigate to "rocket_ascent_trajectory_optimizer\inputs" and edit "SIMULATION_INPUTS.JSON" with desired launch parameters.
4) Navigate to "rocket_ascent_trajectory_optimizer" and run "Rocket Flight Simulator.exe"
5) Simulation will open a window and display the live results. Press esc to exit.

Alternatively:
1) Navigate to "rocket_ascent_trajectory_optimizer\inputs" and edit "SIMULATION_INPUTS.JSON" with desired launch parameters.
2) Navigate to "rocket_ascent_trajectory_optimizer" and open "Rocket Flight Simulator.sln" and run from within Visual Studio.

# Detailed Instructions:

1) Navigate to "rocket_ascent_trajectory_optimizer\inputs" and edit "SIMULATION_INPUTS.JSON" with desired launch parameters.

The file is setup by default for a Falcon 9 block 5 launch to a 500km orbit with 20000 kg of payload. The circularization burn is not 
yet included with this tool. The program will open a window and display each orbit itteration and the current pitch over angle being tested, 
orbit apogee, perigee, and remaining propellant for the active stage at the end of the flight. The program optimizes the flight profile to 
maximize the perigee for a given set of flight parameters. It will output the optimal pitch_over_angle for the given flight parameters.

If the propellant load is insufficient for the payload and vehicle configuration, the simulation will fail to converge to a perigee above the 
surface of the earth.

If target inclination is too small for the given latitude, the simulation will send an error to the console and will set inclination to equal latitude.

If "circular_orbit_altitude" is set too low, the simulation will fail to produce an orbit that does not go below the earth's surface. This is because
the vehicle does not have enough acceleration to achieve orbital velocity while staying below the target altitude during launch. To achieve this orbit,
the flight profile would need to overshoot the target orbit altitude which is not allowed in this simulation.

IMPORTANT! AVOID MODIFYING THE BELOW VARIABLES in SIMULATION_INPUTS.JSON AS THEY MAY BREAK THE OPTIMIZER:
"dt": 0.005,
"pitch_over_angle": 0.5,
"heading": 90,
"initial_position_x": 0,
"initial_position_y": 6378000,
"initial_velocity_x": 0.0,
"initial_velocity_y": 0.0,
"initial_acceleration_x": 0.0,
"initial_acceleration_y": 0.0,
"throttle": 1.0,
"R_earth": 6378000,
"M_earth": 5.972e24

2) Navigate to "rocket_ascent_trajectory_optimizer" and open "Rocket Flight Simulator.sln" and run from within Visual Studio.

3) ESC closes the window.

4) Assumptions:
	a) The simulation accounts for drag and gravity.
	
	b) The earth standard atmosphere model is used for calculating drag.
	
	c) The thrust change as a function of altitude is modeled. It assumes engine thrust and mass flow rate vary linearly with throttle lever 
		(which is a poor assumption and should be corrected to use the isentropic gas expansion equations).
	
	d) Drag coefficient is assumed to be:
		0.15 for mach_number < 0.6
		0.5 * mach_number - 0.15 for mach_number < 1.2
		-0.33333 * mach_number + 0.85 for mach_number < 1.8
		-0.03125 * mach_number + .30625 for mach_number < 5.0
		0.15 for mach_number >= 5.0
	
	e) Thrust throttle changes are instantaneous currently. Ramp function should be used for better accuracy.
	
	f) Max Q throttling is allowed and is set by using Q_max_throttle_threshold and Max_Q_throttle_setting. If Q exceeds Q_max_throttle_threshold,
		throttle is set to Max_Q_throttle_setting until Q returns below Q_max_throttle_threshold.
	
	g) Earth is spherical and has radius of 6378000 m
	
	h) Simulation is shown in inertial reference frame, not rotational frame of earth. Therefore the initial simulated flight plot tracks horizontally 
		at high speed. This speed is the boost that Earth's rotation gives the vehicle at launch.




