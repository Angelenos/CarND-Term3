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

Vehicle::Vehicle(){}

Vehicle::Vehicle(double d, double s, double v, string state) {

    this->d = d;
    this->s = s;
    this->v = v;
    this->state = state;
    this->veh_stat = 0;

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
    
    //TODO: Your solution here.
    vector<string> states = successor_states();
    float cost;
    vector<float> costs;
    vector<string> final_states;
    vector<vector<vector<double>>> final_trajectories;

    for (vector<string>::iterator it = states.begin(); it != states.end(); ++it) {
        vector<vector<double>> trajectory = generate_trajectory(*it, sensor_fusion);
        if (trajectory.size() != 0) {
            cost = 1.0; // calculate_cost(*this, vector<vector<double>> se, trajectory);
            costs.push_back(cost);
            final_trajectories.push_back(trajectory);
        }
    }

    vector<float>::iterator best_cost = min_element(begin(costs), end(costs));
    int best_idx = distance(begin(costs), best_cost);
    
    vector<vector<double>> final_trajectory = final_trajectories[best_idx];
    for(auto it = final_trajectory.begin(); it != final_trajectory.end(); it++) {
        next_path_x.push_back((*it)[0]);
        next_path_y.push_back((*it)[0]);
    }
}

vector<string> Vehicle::successor_states() {
    /*
    Provides the possible next states given the current state for the FSM 
    discussed in the course, with the exception that lane changes happen 
    instantaneously, so LCL and LCR can only transition back to KL.
    */
    vector<string> states;
    string state = this->state;
    if(veh_stat > 0) {
        states.push_back(state);
    } else {
        states.push_back("KL");
        if(state.compare("KL") == 0) {
            // states.push_back("PLCL");
            // states.push_back("PLCR");
        } else if (state.compare("PLCL") == 0) {
            if (lane != 2) {
                states.push_back("PLCL");
                states.push_back("LCL");
            }
        } else if (state.compare("PLCR") == 0) {
            if (lane != 0) {
                states.push_back("PLCR");
                states.push_back("LCR");
            }
        }
    }
    
    //If state is "LCL" or "LCR", then just return "KL"
    return states;
}

vector<vector<double>> Vehicle::generate_trajectory(string state, const vector<vector<double>> &sensor_fusion) {
    /*
    Given a possible next state, generate the appropriate trajectory to realize the next state.
    */
    vector<vector<double>> trajectory;
    if (state.compare("KL") == 0) {
        trajectory = keep_lane_trajectory(sensor_fusion);
    } else if (state.compare("LCL") == 0 || state.compare("LCR") == 0) {
        trajectory = lane_change_trajectory(state, sensor_fusion);
    } else if (state.compare("PLCL") == 0 || state.compare("PLCR") == 0) {
        trajectory = prep_lane_change_trajectory(state, sensor_fusion);
    }
    return trajectory;
}

vector<vector<double>> Vehicle::keep_lane_trajectory(const vector<vector<double>> &sensor_fusion) {
    /*
    Generate a keep lane trajectory.
    */
    vector<vector<double>> trajectory;
    int ind_ahead = -1;

    get_polyfit_path(lane);

    if (get_vehicle_ahead(sensor_fusion, lane, ind_ahead)) {
        double vx_ahead = sensor_fusion[ind_ahead][3] * 2.23694; // m/s to mph
        double vy_ahead = sensor_fusion[ind_ahead][4] * 2.23694; // m/s to mph
        double target_speed_pend = 0.9 * sqrt((vx_ahead*vx_ahead) + (vy_ahead*vy_ahead));
        if(veh_stat == 0) {
            veh_stat = 1;
            target_speed = target_speed_pend;
            trajectory = gen_smooth_path(true);
        } else {
            if(target_speed_pend < target_speed && (target_speed - target_speed_pend)/target_speed > 0.1) {
                target_speed = 0.9 * target_speed_pend;
                trajectory = gen_smooth_path(false);
            }
        }
    } else {
        veh_stat = 0;
        target_speed = ref_v;
        trajectory = gen_smooth_path(false);
    }

    return trajectory;
}

vector<vector<double>> Vehicle::prep_lane_change_trajectory(string state, const vector<vector<double>> &sensor_fusion) {
    /*
    Generate a trajectory preparing for a lane change.
    */
}

vector<vector<double>> Vehicle::lane_change_trajectory(string state, const vector<vector<double>> &sensor_fusion) {
    /*
    Generate a lane change trajectory.
    */
}

bool Vehicle::get_vehicle_behind(const vector<vector<double>> &sensor_fusion, int lane, int &ind) {
    /*
    Returns a true if a vehicle is found behind the current vehicle, false otherwise. The passed reference
    rVehicle is updated if a vehicle is found.
    */
    double min_s = preferred_buffer;
    bool found_vehicle = false;
    for (auto it = sensor_fusion.begin(); it != sensor_fusion.end(); ++it) {
        int lane_behind = (int)((*it)[6] / 4.0);
        double s_behind = (*it)[5];
        if (lane_behind == this->lane && s_behind < this->s && this->s - s_behind < min_s) {
            ind = (int)(it - sensor_fusion.begin());
            found_vehicle = true;
        }
    }
    return found_vehicle;
}

bool Vehicle::get_vehicle_ahead(const vector<vector<double>> &sensor_fusion, int lane, int &ind) {
    /*
    Returns a true if a vehicle is found ahead of the current vehicle, false otherwise. The passed reference
    rVehicle is updated if a vehicle is found.
    */
    double min_s = preferred_buffer;
    bool found_vehicle = false;
    for (auto it = sensor_fusion.begin(); it != sensor_fusion.end(); ++it) {
        int lane_ahead = (int)((*it)[6] / 4.0);
        double s_ahead = (*it)[5];
        if (lane_ahead == this->lane && s_ahead > this->s && s_ahead - this->s < min_s) {
            ind = (int)(it - sensor_fusion.begin());
            found_vehicle = true;
        }
    }
    return found_vehicle;
}

void Vehicle::get_polyfit_path(int lane_tag) {
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

    vector<double> next_wp0 = getXY(s + 30, (2 + (4*lane_tag)), map_waypoints_s, map_waypoints_x, map_waypoints_y);
    vector<double> next_wp1 = getXY(s + 60, (2 + (4*lane_tag)), map_waypoints_s, map_waypoints_x, map_waypoints_y);
    vector<double> next_wp2 = getXY(s + 90, (2 + (4*lane_tag)), map_waypoints_s, map_waypoints_x, map_waypoints_y);

    ptsx.push_back(next_wp0[0]);
    ptsx.push_back(next_wp1[0]);
    ptsx.push_back(next_wp2[0]);

    ptsy.push_back(next_wp0[1]);
    ptsy.push_back(next_wp1[1]);
    ptsy.push_back(next_wp2[1]);

    for(int i = 0; i < ptsx.size(); i++) {
        double shift_x = ptsx[i] - ref_x;
        double shift_y = ptsx[i] - ref_y;

        ptsx[i] = (shift_x * cos(0 - ref_yaw)) - (shift_y * sin(0 - ref_yaw));
        ptsy[i] = (shift_x * sin(0 - ref_yaw)) + (shift_y * cos(0 - ref_yaw));
    }

    s_gen.set_points(ptsx, ptsy);
}

vector<vector<double>> Vehicle::gen_smooth_path(bool reset) {
    // Use const speed profile t
    // Use function atan(x) to smooth the velocity profile during acceleration/deacceleration
    // Formula for the function is v(t) = amp(atan(coeff*t -12) + Pi/2)
    // Here magic number 12 is chosen since atan(12) is 95% of its upper limit pi/2
    vector<vector<double>> next_path;
    double cur_speed = v * 2.23694; // m/s to mph
    double amp = (target_speed - cur_speed) / M_PI;
    double coeff = max_acc / amp;
    double integral_tot = 24 / coeff * amp * M_PI / 2; // Integral of v(t) over time, which is the distance travelled
    double next_v = 0;
    int N = (int)(24 / coeff / dt);

    double x_ref = 0;
    double y_ref = 0;

    if(reset) {
        N_pend = N;
        next_path_x = {};
        next_path_y = {};
    }

    for(int i = 0; i < path_size - prev_path_x.size(); i++) {
        if (veh_stat = 0) {
            next_v = target_speed;
        } else{
            if(N_pend == 0) {
                veh_stat = 0;
                break;
            }
            next_v = amp * (atan((coeff*dt*(i + N - N_pend)) - 12) + (M_PI/2));
            N_pend--;
        }

        x_ref += next_v * dt;
        y_ref = s_gen(x_ref);

        next_path.push_back({(x_ref*cos(yaw)) - (y_ref*sin(yaw)), (x_ref*sin(yaw)) + (y_ref*cos(yaw))});
    }
}