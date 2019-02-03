#ifndef PLANNER_H
#define PLANNER_H
#include <vector>
#include "json.hpp"

#define MAX_S 6945.554
#define SIM_PD 0.02
#define MPS_TO_MPH 2.237
#define LANE_WIDTH 4
#define LANE_CENTER_D(lane) (lane*1.5*LANE_WIDTH)


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
        int TRAJ_POINTS = 50;
        double REF_VEL = 49.5; // m/s
        Planner(vector<vector<double>> map_waypoints_info);
        ~Planner() {}

        vector<vector<double>> get_next_pos_vals(CarData &cd);
    private:
        vector<double> map_waypoints_x;
        vector<double> map_waypoints_y;
        vector<double> map_waypoints_s;
        vector<double> map_waypoints_dx;
        vector<double> map_waypoints_dy;
        int curr_lane;
};
#endif  // PLANNER_H

