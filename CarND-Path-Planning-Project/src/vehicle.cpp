#include <algorithm>
#include <iostream>
#include "vehicle.h"
#include <cmath>
#include <map>
#include <string>
#include <iterator>
#include "spline.h"
#include "helper.h"

using namespace std;
using namespace tk;

/**
 * Initializes Vehicle
 */

Vehicle::Vehicle() {
    this->d = 0;
    this->s = 0;
    this->v = 0;
    this->lane = 0;
    this->lane_tag = 0;
    this->state = "KL";
    this->occupied_flg = false;
}

Vehicle::Vehicle(double d, double s, double v, string state) {

    this->d = d;
    this->s = s;
    this->v = v;
    this->state = state;
    this->occupied_flg = false;

}

Vehicle::~Vehicle() {}


void Vehicle::choose_next_state(const vector<vector<double>> &sensor_fusion) {
    /*
    
    ***Here you can implement the transition_function code from the Behavior Planning Pseudocode
    classroom concept.***
    
    INPUT: A predictions map. This is a map using vehicle id as keys with predicted
        vehicle trajectories as values. A trajectory is a vector of Vehicle objects. The first
        item in the trajectory represents the vehicle at the current timestep. The second item in
        the trajectory represents the vehicle one timestep in the future.
    OUTPUT: The the best (lowest cost) trajectory for the ego vehicle corresponding to the next ego vehicle state.

    Functions that will be useful:
    1. successor_states() - Uses the current state to return a vector of possible successor states for the finite 
       state machine.
    2. generate_trajectory(string state, const vector<vector<double>> &sensor_fusion) - Returns a vector of Vehicle objects 
       representing a vehicle trajectory, given a state and predictions. Note that trajectory vectors 
       might have size 0 if no possible trajectory exists for the state. 
    3. calculate_cost(Vehicle vehicle, const vector<vector<double>> &sensor_fusion, vector<Vehicle> trajectory) - Included from 
       cost.cpp, computes the cost for a trajectory.
    */

    // Determine possible successor state
    vector<string> states = successor_states();

    // Evaluate cost
    double cost;
    vector<double> costs;
    vector<string> final_states;
    vector<vector<vector<double>>> final_trajectories;

    for (vector<string>::iterator it = states.begin(); it != states.end(); ++it) {
        string state_in = *it;
        vector<vector<double>> trajectory = generate_trajectory(state_in, sensor_fusion);
        if (trajectory.size() != 0) {
            cost = trajectory[trajectory.size() - 1][1]; // calculate_cost(*this, vector<vector<double>> se, trajectory);
            costs.push_back(cost);
            trajectory.erase(trajectory.end() - 1);
            final_trajectories.push_back(trajectory);
        }
    }

    vector<double>::iterator best_cost = min_element(begin(costs), end(costs));
    int best_idx = distance(begin(costs), best_cost);
    this->state = states[best_idx];

    cout << "Best next state: " << states[best_idx] << "; cost: " << costs[best_idx] << endl;
    
    vector<vector<double>> final_trajectory = final_trajectories[best_idx];
    for(auto it = final_trajectory.begin(); it != final_trajectory.end(); it++) {
        next_path_x.push_back((*it)[0]);
        next_path_y.push_back((*it)[1]);
    }

    if(states[best_idx].compare("PLCR") == 0 || states[best_idx].compare("PLCL") == 0 ) {
        lane_tag = lane + lane_direction[states[best_idx]];
    }

    if(states[best_idx].compare("LCL") == 0 || states[best_idx].compare("LCR") == 0 ) {
        if(abs(d - ((0.5 + lane_tag) * lane_width)) / lane_width <= 0.025) {
            occupied_flg = false;
        } else {
            occupied_flg = true;
        }
    }
}

vector<string> Vehicle::successor_states() {
    /*
    Provides the possible next states given the current state for the FSM 
    discussed in the course, with the exception that lane changes happen 
    instantaneously, so LCL and LCR can only transition back to KL.
    */
    vector<string> states;
    string state_cur = this->state;
    cout << "Current lane: " << this->lane << "; Target lane: " << lane_tag << "; Current d: " << d << endl;
    if(occupied_flg) {
        states.push_back(state_cur);
    } else {
        states.push_back("KL");
        if(state_cur.compare("KL") == 0) {
            states.push_back("PLCL");
            states.push_back("PLCR");
        } else if (state_cur.compare("PLCL") == 0) {
            if (this->lane != 0) {
                states.push_back("PLCL");
                states.push_back("LCL");
            }
        } else if (state_cur.compare("PLCR") == 0) {
            if (this->lane != 2) {
                states.push_back("PLCR");
                states.push_back("LCR");
            }
        }
    }
    
    //If state is "LCL" or "LCR", then just return "KL"
    return states;
}

vector<vector<double>> Vehicle::generate_trajectory(string state_in, const vector<vector<double>> &sensor_fusion) {
    /*
    Given a possible next state, generate the appropriate trajectory to realize the next state.
    */
    vector<vector<double>> trajectory;
    if (state_in.compare("KL") == 0) {
        trajectory = keep_lane_trajectory(sensor_fusion);
    } else if (state_in.compare("LCL") == 0 || state_in.compare("LCR") == 0) {
        trajectory = lane_change_trajectory(state_in, sensor_fusion);
    } else if (state_in.compare("PLCL") == 0 || state_in.compare("PLCR") == 0) {
        trajectory = prep_lane_change_trajectory(state_in, sensor_fusion);
    }
    return trajectory;
}

vector<vector<double>> Vehicle::keep_lane_trajectory(const vector<vector<double>> &sensor_fusion) {
    /*
    Generate a keep lane trajectory.
    */
    vector<vector<double>> trajectory;
    int ind_ahead = -1;
    double cost = 0;

    vector<double> ref = get_polyfit_path(lane, 4, 35);

    if (get_vehicle_ahead(sensor_fusion, lane, ind_ahead)) {
        double vx_ahead = sensor_fusion[ind_ahead][3];
        double vy_ahead = sensor_fusion[ind_ahead][4];
        double target_speed_pend = 0.975 * sqrt((vx_ahead*vx_ahead) + (vy_ahead*vy_ahead));
        target_speed = target_speed_pend;
    } else {
        target_speed = ref_v;
    }

    trajectory = gen_smooth_path(ref);

    double cost_ineff = abs(ref_v - target_speed) / ref_v;
    double cost_lane = abs(lane - lane_desire);

    if(target_speed > ref_v) {
        cost = 1.0;
    }

    cost += (cost_weight[0] * cost_ineff) + (cost_weight[1] * cost_lane);
    trajectory.push_back({-1, cost});

    return trajectory;
}

vector<vector<double>> Vehicle::prep_lane_change_trajectory(string state_in, const vector<vector<double>> &sensor_fusion) {
    /*
    Generate a trajectory preparing for a lane change.
    */
    vector<vector<double>> trajectory;
    double target_speed_side;
    double cost = 0;
    int ind_ahead = -1;
    int ind_behind = -1;
    int ind_sideahead = -1;
    double target_speed_ahead = ref_v;
    double target_speed_behind = ref_v;
    double target_speed_sideahead = ref_v;
    int target_lane = lane + lane_direction[state_in];

    vector<double> ref = get_polyfit_path(lane, 4, 35);

    if (get_vehicle_ahead(sensor_fusion, lane, ind_ahead)) {
        double vx_ahead = sensor_fusion[ind_ahead][3];
        double vy_ahead = sensor_fusion[ind_ahead][4];
        // When estimating vehicle speed ahead, a safety factor 0.975 is added to avoid collision due to comm delay
        target_speed_ahead = 0.975 * sqrt((vx_ahead*vx_ahead) + (vy_ahead*vy_ahead));
    }

    if(get_vehicle_behind(sensor_fusion, target_lane, ind_behind)) {
        double vx_behind = sensor_fusion[ind_behind][3];
        double vy_behind = sensor_fusion[ind_behind][4];
        target_speed_behind = sqrt((vx_behind*vx_behind) + (vy_behind*vy_behind));
    }

    if(get_vehicle_ahead(sensor_fusion, target_lane, ind_sideahead)) {
        double vx_sideahead = sensor_fusion[ind_sideahead][3];
        double vy_sideahead = sensor_fusion[ind_sideahead][4];
        target_speed_sideahead = 0.975 * sqrt((vx_sideahead*vx_sideahead) + (vy_sideahead*vy_sideahead));
    }

    target_speed = target_speed_ahead;
    target_speed_side = (target_speed_sideahead > target_speed_behind?target_speed_sideahead:target_speed_behind);
    trajectory = gen_smooth_path(ref);

    double cost_ineff = (abs(ref_v - target_speed) + abs(ref_v - target_speed_side)) / (2.0 * ref_v);
    double cost_lane = abs(target_lane - lane_desire);

    cost = (cost_weight[0] * cost_ineff) + (cost_weight[1] * cost_lane) + 0.001;
    cout << "PLC state: " << state_in << " with cost: " << cost << endl;
    trajectory.push_back({-1, cost});

    return trajectory;
}

vector<vector<double>> Vehicle::lane_change_trajectory(string state_in, const vector<vector<double>> &sensor_fusion) {
    /*
    Generate a lane change trajectory.
    */
    vector<vector<double>> trajectory;
    double target_speed_side;
    double cost = 0;
    int ind_ahead = -1;
    int ind_behind = -1;
    int ind_sideahead = -1;
    double target_speed_ahead = ref_v;
    double target_speed_behind = ref_v;
    double target_speed_sideahead = ref_v;
    int target_lane = lane_tag;

    vector<double> ref = get_polyfit_path(target_lane, 3, 50);

    if (get_vehicle_ahead(sensor_fusion, lane, ind_ahead)) {
        double vx_ahead = sensor_fusion[ind_ahead][3];
        double vy_ahead = sensor_fusion[ind_ahead][4];
        target_speed_ahead = 0.975 * sqrt((vx_ahead*vx_ahead) + (vy_ahead*vy_ahead));
    }

    if(get_vehicle_behind(sensor_fusion, target_lane, ind_behind)) {
        double vx_behind = sensor_fusion[ind_behind][3];
        double vy_behind = sensor_fusion[ind_behind][4];
        double s_behind = sensor_fusion[ind_behind][5];
        target_speed_behind = sqrt((vx_behind*vx_behind) + (vy_behind*vy_behind));
        if(s - s_behind <= 0.7 * preferred_buffer) {cost += 1.0;}
    }

    if(get_vehicle_ahead(sensor_fusion, target_lane, ind_sideahead)) {
        double vx_sideahead = sensor_fusion[ind_sideahead][3];
        double vy_sideahead = sensor_fusion[ind_sideahead][4];
        double s_sideahead = sensor_fusion[ind_sideahead][5];
        target_speed_sideahead = 0.975 * sqrt((vx_sideahead*vx_sideahead) + (vy_sideahead*vy_sideahead));
        if(s_sideahead - s <= preferred_buffer) {cost += 1.0;}
    }

    target_speed_side = (target_speed_sideahead > target_speed_behind?target_speed_sideahead:target_speed_behind);
    target_speed = target_speed_side;
    trajectory = gen_smooth_path(ref);

    double cost_ineff = abs(ref_v - target_speed_side) / ref_v;
    double cost_lane = abs(target_lane - lane_desire);

    cost += (cost_weight[0] * cost_ineff) + (cost_weight[1] * cost_lane);
    cout << "LC state: " << state_in << " with cost: " << cost << endl;
    trajectory.push_back({-1, cost});

    cout << "Lane change trajectory cost: " << cost << endl;
    
    return trajectory;
}

bool Vehicle::get_vehicle_behind(const vector<vector<double>> &sensor_fusion, int target_lane, int &ind) {
    /*
    Returns a true if a vehicle is found behind the current vehicle, false otherwise. The passed reference
    rVehicle is updated if a vehicle is found.
    */
    double min_s = preferred_buffer;
    double dist_min = preferred_buffer;
    bool found_vehicle = false;
    for (auto it = sensor_fusion.begin(); it != sensor_fusion.end(); ++it) {
        vector<double> vehicle_targ = *it;
        double d_behind = vehicle_targ[6] < 0? -12:vehicle_targ[6]; 
        int lane_behind = (int)(d_behind / lane_width);
        double s_behind = vehicle_targ[5];
        if (lane_behind == target_lane && s_behind < this->s && this->s - s_behind < min_s) {
            found_vehicle = true;
            // Here only the nearest vehicle will be considered
            if(this->s - s_behind < dist_min) {
                ind = (int)(it - sensor_fusion.begin());
                dist_min = this->s - s_behind;
            }
        }
    }
    return found_vehicle;
}

bool Vehicle::get_vehicle_ahead(const vector<vector<double>> &sensor_fusion, int target_lane, int &ind) {
    /*
    Returns a true if a vehicle is found ahead of the current vehicle, false otherwise. The passed reference
    rVehicle is updated if a vehicle is found.
    */
    double min_s = preferred_buffer;
    double dist_min = preferred_buffer;
    bool found_vehicle = false;
    for (auto it = sensor_fusion.begin(); it != sensor_fusion.end(); ++it) {
        vector<double> vehicle_targ = *it;
        double d_ahead = (*it)[6] < 0? -12:vehicle_targ[6]; 
        int lane_ahead = (int)(d_ahead / lane_width);
        double s_ahead = (*it)[5];
        if (lane_ahead == target_lane && s_ahead > this->s && s_ahead - this->s < min_s) {
            found_vehicle = true;
            // Here only the nearest vehicle will be considered
            if(s_ahead - this->s < dist_min) {
                ind = (int)(it - sensor_fusion.begin());
                dist_min = s_ahead - this->s;
            }
        }
    }
    return found_vehicle;
}

vector<double> Vehicle::get_polyfit_path(int target_lane, int num_pts, double resolution) {
    /* Generate path under polynomial fit based on the given target lane, number of points
    given and resolution in fitting*/
    vector<double> ptsx;
    vector<double> ptsy;
    double ref_x = x;
    double ref_y = y;
    double ref_yaw = yaw;

    unsigned prev_path_size = prev_path_x.size();
    if(prev_path_size < 2) {
        ptsx.push_back(x - cos(yaw));
        ptsy.push_back(y - sin(yaw));
    } else {
        ref_x = prev_path_x[prev_path_size - 1];
        ref_y = prev_path_y[prev_path_size - 1];
        double ref_x_prev = prev_path_x[prev_path_size - 2];
        double ref_y_prev = prev_path_y[prev_path_size - 2];

        ptsx.push_back(ref_x_prev);
        ptsy.push_back(ref_y_prev);
        ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
    }
    ptsx.push_back(ref_x);
    ptsy.push_back(ref_y);

    // Number of points to generate fitting is provided as a function parameter
    for(int i = 1; i <= num_pts; i++) {
        vector<double> next_wp = getXY(s + (resolution*i), (2 + (4*target_lane)), map_waypoints_s, map_waypoints_x, map_waypoints_y);
        ptsx.push_back(next_wp[0]);
        ptsy.push_back(next_wp[1]);
    }

    // Transfer the points into vehicle local coordinate
    for(int i = 0; i < ptsx.size(); i++) {
        double shift_x = ptsx[i] - ref_x;
        double shift_y = ptsy[i] - ref_y;

        ptsx[i] = (shift_x * cos(0 - ref_yaw)) - (shift_y * sin(0 - ref_yaw));
        ptsy[i] = (shift_x * sin(0 - ref_yaw)) + (shift_y * cos(0 - ref_yaw));
    }

    // Feed points to the spline generator
    s_gen.set_points(ptsx, ptsy);

    // Return the reference points to transfer from global to vehicle local coordinates
    // These values will be used by "Vehicle::gen_smooth_path(vector<double> ref)"
    return {ref_x, ref_y, ref_yaw};
}

vector<vector<double>> Vehicle::gen_smooth_path(vector<double> ref) {
    /* Generate smooth path for vehicle motions in the future. Here a constant jerk (da/dt)
    trajectory is adopted to avoid both large jerk and large acceleration*/
    vector<vector<double>> next_path;

    // Parsing the input reference points
    double ref_x = ref[0];
    double ref_y = ref[1];
    double ref_yaw = ref[2];

    double x_add_on, y_add_on;
    double x_point, y_point, x_ref, y_ref;
    double next_v = v;
    double next_a = 0;
    
    // Calculate current velocity and acceleration from prev_path vector
    if(prev_path_x.size() > 2) {
        double shift_x = prev_path_x[prev_path_x.size() - 1] - prev_path_x[prev_path_x.size() - 2];
        double shift_y = prev_path_y[prev_path_y.size() - 1] - prev_path_y[prev_path_y.size() - 2];
        next_v = sqrt((shift_x*shift_x) + (shift_y*shift_y)) / dt;
        shift_x = prev_path_x[prev_path_x.size() - 2] - prev_path_x[prev_path_x.size() - 3];
        shift_y = prev_path_y[prev_path_y.size() - 2] - prev_path_y[prev_path_y.size() - 3];
        double prev_v = sqrt((shift_x*shift_x) + (shift_y*shift_y)) / dt;
        next_a = (next_v - prev_v) / dt;
    }

    double a_add_on = next_a * dt;
    double j_add_on = max_jerk * dt;
    x_add_on = 0;

    for(int i = prev_path_x.size(); i < PATH_SIZE; i++) {

        // Here vehicle is accelerating/deaccelerating with constant jerk specified in max_jerk parameters
        // The maximum absolute value of acceleration is limited by parameter max_acc as well
        double v_diff = target_speed - next_v;
        if(v_diff > 0.1) {
            // Acceleration
            if(abs(v_diff) / target_speed < 0.1) {
                next_a = (next_a >= j_add_on?(next_a - j_add_on):0);
            } else {
                next_a = (next_a >= max_acc - j_add_on?max_acc:(next_a + j_add_on));
            }
        } else if(v_diff < -0.1) {
            // Deacceleration
            if(abs(v_diff) / target_speed < 0.1) {
                next_a = (next_a <= -j_add_on?(next_a + j_add_on):0);
            } else {
                next_a = (next_a <= j_add_on - max_acc?-max_acc:(next_a - j_add_on));
            }
        }

        // Update velocity in the next loop step
        next_v += next_a * dt;
        next_v = next_v >= ref_v? ref_v:next_v;

        x_point = x_add_on + (next_v*dt);
        y_point = s_gen(x_point);
        x_add_on = x_point;

        // Adopt the reference points returned by poly_fit function to convert vehicle local
        // points into global coordinate
        x_ref = x_point;
        y_ref = y_point;

        x_point = (x_ref*cos(ref_yaw)) - (y_ref*sin(ref_yaw));
        y_point = (x_ref*sin(ref_yaw)) + (y_ref*cos(ref_yaw));
        x_point += ref_x;
        y_point += ref_y;

        next_path.push_back({x_point, y_point});
    }

    return next_path;
}