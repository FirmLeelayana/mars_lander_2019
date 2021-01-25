// Mars lander simulator
// Version 1.10
// Mechanical simulation functions
// Gabor Csanyi and Andrew Gee, August 2017

// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation, to make use of it
// for non-commercial purposes, provided that (a) its original authorship
// is acknowledged and (b) no modified versions of the source code are
// published. Restriction (b) is designed to protect the integrity of the
// exercise for future generations of students. The authors would be happy
// to receive any suggested modifications by private correspondence to
// ahg@eng.cam.ac.uk and gc121@eng.cam.ac.uk.

#include "lander.h"
#include <math.h>
#include <fstream>
void autopilot (void)
  // Autopilot to adjust the engine throttle, parachute and attitude control
{
  // INSERT YOUR CODE HERE
	parachute_status = NOT_DEPLOYED;
	stabilized_attitude = 1;
	double target_altitude = 500;
	double P_out, kp, kh, delta, e, altitude;
	kh = 0.018;
	kp = 2;
	delta = 0.5;
	altitude = position.abs() - MARS_RADIUS;
	e = -(0.5 + kh * altitude + velocity * position.norm());
	P_out = kp * e;
	/*if (P_out <= - delta) {
		throttle = 0;
	}
	else if(-delta < P_out && P_out < (1 - delta)) {
		throttle = delta + P_out;
	}
	else {
		throttle = 1;
	}
	*/
	throttle = 747.1 / MAX_THRUST;
	/*ofstream fout;
	fout.open("trajectories2.txt", std::ios_base::app);
	if (fout) {
		fout << altitude << ' ' << velocity * position.norm() << endl;
	}
	else {
		cout << "Could not open trajectory file for writing" << endl;
	}
	*/

}

void numerical_dynamics (void)
  // This is the function that performs the numerical integration to update the
  // lander's pose. The time step is delta_t (global variable).
{
  // INSERT YOUR CODE HERE
  // Euler
  /*
  vector3d acceleration;
  vector3d thrust_vector, thrust, drag;
  double area = pow(LANDER_SIZE, 2) * M_PI ;
  double parachute_area = pow(2 * LANDER_SIZE, 2) * 5;
  double d = atmospheric_density(position);
  double mass = UNLOADED_LANDER_MASS + FUEL_CAPACITY * FUEL_DENSITY * fuel;
  thrust_vector = thrust_wrt_world();
  thrust = thrust_vector/mass;
  if (parachute_status == DEPLOYED) {
	  drag = (-0.5 * d * DRAG_COEF_LANDER* area *velocity.abs2()*velocity.norm() / mass) - (0.5 * d * DRAG_COEF_CHUTE * parachute_area*velocity.abs2()*velocity.norm() / mass);
  }
  else {
	  drag = -0.5 * d * DRAG_COEF_LANDER* area *velocity.abs2()*velocity.norm() / mass;
  }
  acceleration = -((GRAVITY * MARS_MASS) / (pow(position.abs(), 3)))* position + thrust + drag;
  position = position + delta_t * velocity;
  velocity = velocity + delta_t * acceleration;
  */

  // Verlet
	static vector3d previous_position;
	vector3d new_position;
	vector3d acceleration;
	vector3d thrust_vector, thrust, drag;
	double area = pow(LANDER_SIZE, 2) * M_PI;
	double parachute_area = pow(2 * LANDER_SIZE, 2) * 5;
	double d = atmospheric_density(position);
	double mass = UNLOADED_LANDER_MASS + FUEL_CAPACITY * FUEL_DENSITY * fuel;
	thrust_vector = thrust_wrt_world();
	thrust = thrust_vector / mass;
	if (parachute_status == DEPLOYED) {
		drag = (-0.5 * d * DRAG_COEF_LANDER* area *velocity.abs2()*velocity.norm() / mass) - (0.5 * d * DRAG_COEF_CHUTE * parachute_area*velocity.abs2()*velocity.norm() / mass);
	}
	else {
		drag = -0.5 * d * DRAG_COEF_LANDER* area *velocity.abs2()*velocity.norm() / mass;
	}
	if (simulation_time == 0.0) {
		acceleration = -((GRAVITY * MARS_MASS) / (pow(position.abs(), 3)))* position + thrust + drag;
		new_position = position + delta_t * velocity;
		velocity = velocity + delta_t * acceleration;
	}
	else {
		acceleration = -((GRAVITY * MARS_MASS) / (pow(position.abs(), 3)))* position + thrust + drag;
		new_position = 2 * position - previous_position + pow(delta_t, 2) * acceleration;
		velocity = (1 / delta_t) * (new_position - position);
	}
	previous_position = position;
	position = new_position;



  // Here we can apply an autopilot to adjust the thrust, parachute and attitude
  if (autopilot_enabled) autopilot();

  // Here we can apply 3-axis stabilization to ensure the base is always pointing downwards
  if (stabilized_attitude) attitude_stabilization();
}

void initialize_simulation (void)
  // Lander pose initialization - selects one of 10 possible scenarios
{
  // The parameters to set are:
  // position - in Cartesian planetary coordinate system (m)
  // velocity - in Cartesian planetary coordinate system (m/s)
  // orientation - in lander coordinate system (xyz Euler angles, degrees)
  // delta_t - the simulation time step
  // boolean state variables - parachute_status, stabilized_attitude, autopilot_enabled
  // scenario_description - a descriptive string for the help screen

  scenario_description[0] = "circular orbit";
  scenario_description[1] = "descent from 10km";
  scenario_description[2] = "elliptical orbit, thrust changes orbital plane";
  scenario_description[3] = "polar launch at escape velocity (but drag prevents escape)";
  scenario_description[4] = "elliptical orbit that clips the atmosphere and decays";
  scenario_description[5] = "descent from 200km";
  scenario_description[6] = "1";
  scenario_description[7] = "2";
  scenario_description[8] = "3";
  scenario_description[9] = "";

  switch (scenario) {

  case 0:
    // a circular equatorial orbit
    position = vector3d(1.2*MARS_RADIUS, 0.0, 0.0);
    velocity = vector3d(0.0, -3247.087385863725, 0.0);
    orientation = vector3d(0.0, 90.0, 0.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 1:
    // a descent from rest at 10km altitude
    position = vector3d(0.0, -(MARS_RADIUS + 10000.0), 0.0);
    velocity = vector3d(0.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = true;
    autopilot_enabled = false;
    break;

  case 2:
    // an elliptical polar orbit
    position = vector3d(0.0, 0.0, 1.2*MARS_RADIUS);
    velocity = vector3d(3500.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 3:
    // polar surface launch at escape velocity (but drag prevents escape)
    position = vector3d(0.0, 0.0, MARS_RADIUS + LANDER_SIZE/2.0);
    velocity = vector3d(0.0, 0.0, 5027.0);
    orientation = vector3d(0.0, 0.0, 0.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 4:
    // an elliptical orbit that clips the atmosphere each time round, losing energy
    position = vector3d(0.0, 0.0, MARS_RADIUS + 100000.0);
    velocity = vector3d(4000.0, 0.0, 0.0);
    orientation = vector3d(0.0, 90.0, 0.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 5:
    // a descent from rest at the edge of the exosphere
    position = vector3d(0.0, -(MARS_RADIUS + EXOSPHERE), 0.0);
    velocity = vector3d(0.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = true;
    autopilot_enabled = false;
    break;

  case 6:
	  position = vector3d(0.0, -(MARS_RADIUS + 500), 0.0);
	  velocity = vector3d(0.0, 0.0, 0.0);
	  orientation = vector3d(0.0, 0.0, 90.0);
	  delta_t = 0.01;
	  parachute_status = NOT_DEPLOYED;
	  stabilized_attitude = true;
	  autopilot_enabled = true;
	  break;

  case 7:
	  position = vector3d(0.0, -(MARS_RADIUS + 510), 0.0);
	  velocity = vector3d(0.0, 0.0, 0.0);
	  orientation = vector3d(0.0, 0.0, 90.0);
	  delta_t = 0.01;
	  parachute_status = NOT_DEPLOYED;
	  stabilized_attitude = true;
	  autopilot_enabled = true;
	  break;

  case 8:
	  position = vector3d(0.0, -(MARS_RADIUS + 700), 0.0);
	  velocity = vector3d(0.0, 0.0, 0.0);
	  orientation = vector3d(0.0, 0.0, 90.0);
	  delta_t = 0.01;
	  parachute_status = NOT_DEPLOYED;
	  stabilized_attitude = true;
	  autopilot_enabled = true;
	  break;

  case 9:
    break;

  }
}
