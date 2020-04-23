#ifndef PLANNER_H
#define PLANNER_H
#include <vector>
#include "json.hpp"

#define MAX_S 6945.554
#define SIM_PD 0.02
#define MPS_TO_MPH 2.237
#define LANE_WIDTH 4
#define LANE_CENTER_D(lane) (lane*1.5*LANE_WIDTH)
#define D_TO_LANE(d) (d / int(LANE_WIDTH))


using nlohmann::json;
using std::vector;

struct CarData {
    double car_x;
    double car_y;
    double car_s;
    double car_d;
    double car_yaw;
    double car_speed;

    double end_path_s;
    double end_path_d;

    json previous_path_x;
    json previous_path_y;
    // Sensor Fusion Data, a list of all other cars on the same side
    //   of the road.
    json sensor_fusion;
};

class Planner {
    public :
        int TRAJ_POINTS         = 50;
        double MAX_VEL_MS       = 49.5 / MPS_TO_MPH; // m/s
        double SAFE_ACC_INC     = 9.5;  // m/s2
        double SAFE_DISTANCE_M  = 30.0; // m
        Planner(vector<vector<double>> map_waypoints_info);
        ~Planner() {}

        vector<vector<double>> get_next_pos_vals(CarData &cd);
    private:
        vector<vector<double>> get_spline_points(CarData &cd, vector<vector<double>> ref_vec);
        void avoid_traffic(CarData &cd, vector<double>ref_pos);
        vector<vector<double>> get_ref_vec(CarData &cd);

        vector<double> map_waypoints_x;
        vector<double> map_waypoints_y;
        vector<double> map_waypoints_s;
        vector<double> map_waypoints_dx;
        vector<double> map_waypoints_dy;
        int curr_lane;
        double curr_vel_ms;
        int tgt_lane;
        double tgt_vel_ms;
};
#endif  // PLANNER_H

