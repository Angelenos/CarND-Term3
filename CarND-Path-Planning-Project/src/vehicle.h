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

const double ref_v = 49.0 * 2.23694;
const double max_acc = 9;
const double max_jerk = 9;
const double dt = 0.02;
const double lane_width = 4.0;
const int path_size = 75;

class Vehicle {
public:

  map<string, int> lane_direction = {{"PLCL", 1}, {"LCL", 1}, {"LCR", -1}, {"PLCR", -1}};

  /*struct collider{

    bool collision ; // is there a collision?
    int  time; // time collision happens
  };*/

  spline s_gen;

  // double L = 4.5;

  double preferred_buffer = 30.0; // impacts "keep lane" behavior.

  int lane;

  // Vehicle state, 0 for free to accept instruction;
  // 1 for busy performing current action but interruptable;
  // 2 for busy performing current action and uninterruptable;
  int veh_stat;

  double d;
  double s;
  double x;
  double y;
  double v;
  double yaw;

  double target_speed;

  vector<double> next_path_x;
  vector<double> next_path_y;

  int N_pend;

  vector<double> prev_path_x;
  vector<double> prev_path_y;

  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // int goal_lane;

  // int goal_s;

  string state;

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

  vector<vector<double>> generate_trajectory(string state, const vector<vector<double>> &sensor_fusion);

  vector<vector<double>> keep_lane_trajectory(const vector<vector<double>> &sensor_fusion);

  vector<vector<double>> lane_change_trajectory(string state, const vector<vector<double>> &sensor_fusion);

  vector<vector<double>> prep_lane_change_trajectory(string state, const vector<vector<double>> &sensor_fusion);

  bool get_vehicle_behind(const vector<vector<double>> &sensor_fusion, int lane, int &ind);

  bool get_vehicle_ahead(const vector<vector<double>> &sensor_fusion, int lane, int &ind);

  void get_polyfit_path(int lane_tag);

  vector<vector<double>> gen_smooth_path(bool reset);
};

#endif