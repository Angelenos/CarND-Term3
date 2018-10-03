#ifndef VEHICLE_H
#define VEHICLE_H
#include <iostream>
#include <random>
#include <vector>
#include <map>
#include <string>
#include "spline.h"

using namespace std;
using namespace tk;

const double ref_v = 49.0 / 2.23694; // Reference speed to maintain (m/s)
const double max_acc = 8; // Maximal acceleration allowed (m/s^2)
const double max_jerk = 8; // Maximal jerk allowed (m/s^3)
const double dt = 0.02; // Time resolution (s)
const double lane_width = 4.0; // Physical width of the lane (m)
const int lane_desire = 1; // Desire lane to stay
const int PATH_SIZE = 65; // Size of the vector next_path
const double cost_weight[2] = {0.95, 0.05}; // Cost weighs of inefficient and lane-swapping costs

class Vehicle {
  /* Vehicle class which defines the basic properties of vehicles on the highway
  as well as functions related to complete highway driving */
public:

  map<string, int> lane_direction = {{"PLCL", -1}, {"LCL", -1}, {"LCR", 1}, {"PLCR", 1}};

  spline s_gen; // Spline object to generate poly_fit

  double preferred_buffer = 35.0; // Front and rear buffer area when maintaining space

  int lane; // Current lane of the vehicle
  int lane_tag; // Target lane of the vehicle

  // Occupied flag used under lane change
  // 0 means vehicle allow all states
  // 1 means vehicle busy under current state (not interruptable by other state)
  bool occupied_flg;

  double d; // Vehicle d value in Frenet coordinate
  double s; // Vehicle s value in Frenet coordinate
  double x; // Vehicle x value in global coordinate
  double y; // Vehicle y value in global coordinate
  double v; // Vehicle speed value (m/s)
  double yaw; // Vehcle yaw angle (rad)

  double target_speed; // Target speed used for next_path generation

  vector<double> next_path_x; // Copy of the next_path_x vector
  vector<double> next_path_y; // Copy of the next_path_y vector

  vector<double> prev_path_x; // Copy of the prev_path_x vector
  vector<double> prev_path_y; // Copy of the prev_path_y vector

  vector<double> map_waypoints_x; // Copy of the map_waypoints_x vector
  vector<double> map_waypoints_y; // Copy of the map_waypoints_y vector
  vector<double> map_waypoints_s; // Copy of the map_waypoints_s vector

  string state; // Vehicle current state

  /**
  * Constructor
  */
  Vehicle();
  Vehicle(double d, double s, double v, string state="CS");

  /**
  * Destructor
  */
  virtual ~Vehicle();

  void choose_next_state(const vector<vector<double>> &sensor_fusion);

  vector<string> successor_states();

  vector<vector<double>> generate_trajectory(string state_in, const vector<vector<double>> &sensor_fusion);

  vector<vector<double>> keep_lane_trajectory(const vector<vector<double>> &sensor_fusion);

  vector<vector<double>> lane_change_trajectory(string state_in, const vector<vector<double>> &sensor_fusion);

  vector<vector<double>> prep_lane_change_trajectory(string state_in, const vector<vector<double>> &sensor_fusion);

  bool get_vehicle_behind(const vector<vector<double>> &sensor_fusion, int target_lane, int &ind);

  bool get_vehicle_ahead(const vector<vector<double>> &sensor_fusion, int target_lane, int &ind);

  vector<double> get_polyfit_path(int target_lane, int num_pts, double resolution);

  vector<vector<double>> gen_smooth_path(vector<double> ref);
};

#endif