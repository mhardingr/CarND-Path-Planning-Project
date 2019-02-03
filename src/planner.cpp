#include <cmath>  // trig function
#include "helpers.h"  // getXY
#include "spline.h"
#include "planner.h"
using nlohmann::json;


Planner::Planner(vector<vector<double>> map_waypoints_info) {
            map_waypoints_x = map_waypoints_info[0];
            map_waypoints_y = map_waypoints_info[1];
            map_waypoints_s = map_waypoints_info[2];
            map_waypoints_dx = map_waypoints_info[3];
            map_waypoints_dy = map_waypoints_info[4];
            curr_lane = 1;
}

vector<vector<double>> Planner::get_next_pos_vals(CarData &cd) {
    vector<double> next_x_vals;
    vector<double> next_y_vals;

    // Create a set of 5 interpolation points from car's current position+heading
    int prev_size = cd.previous_path_x.size();
    vector<double> interp_pts_x;
    vector<double> interp_pts_y;
    double ref_x = cd.car_x;
    double ref_y = cd.car_y;
    double ref_yaw = deg2rad(cd.car_yaw);

    if (prev_size < 2) {
        // Just starting, so extrapolate a previous point
        double prev_x = ref_x - cos(cd.car_yaw);
        double prev_y = ref_y - sin(cd.car_yaw);

        interp_pts_x.push_back(prev_x);
        interp_pts_y.push_back(prev_y);
    } else {
        // Use the end of the prev trajectory as start
        ref_x = cd.previous_path_x[prev_size-1];
        ref_y = cd.previous_path_y[prev_size-1];

        double ref_x_prev = cd.previous_path_x[prev_size-2];
        double ref_y_prev = cd.previous_path_y[prev_size-2];

        interp_pts_x.push_back(ref_x_prev);
        interp_pts_y.push_back(ref_y_prev);
    }
    interp_pts_x.push_back(ref_x);
    interp_pts_y.push_back(ref_y);

    int interp_spacing = 30;
    vector<double> next_interp0 = getXY(cd.car_s+interp_spacing,
                                         LANE_CENTER_D(curr_lane),
                                         map_waypoints_s, map_waypoints_x,
                                         map_waypoints_y);
    vector<double> next_interp1 = getXY(cd.car_s+2*interp_spacing,
                                         LANE_CENTER_D(curr_lane),
                                         map_waypoints_s, map_waypoints_x,
                                         map_waypoints_y);
    vector<double> next_interp2 = getXY(cd.car_s+3*interp_spacing,
                                         LANE_CENTER_D(curr_lane),
                                         map_waypoints_s, map_waypoints_x,
                                         map_waypoints_y);
    interp_pts_x.push_back(next_interp0[0]);
    interp_pts_x.push_back(next_interp1[0]);
    interp_pts_x.push_back(next_interp2[0]);

    interp_pts_y.push_back(next_interp0[1]);
    interp_pts_y.push_back(next_interp1[1]);
    interp_pts_y.push_back(next_interp2[1]);

    // Shift interpolation points to be local frame (current pos+ref_yaw)
    for (int i=0; i<interp_pts_x.size(); i++) {
        double adj_x = interp_pts_x[i] - ref_x;
        double adj_y = interp_pts_y[i] - ref_y;

        interp_pts_x[i] = (adj_x*cos(-ref_yaw) - adj_y*sin(-ref_yaw));
        interp_pts_y[i] = (adj_x*sin(-ref_yaw) + adj_y*cos(-ref_yaw));
    }

    // Enqueue remaining prev path waypoints to new trajectory
    for (int i=0; i<prev_size; i++) {
        next_x_vals.push_back(cd.previous_path_x[i]);
        next_y_vals.push_back(cd.previous_path_y[i]);
    }

    // Prepare spline with interp points
    tk::spline s;
    s.set_points(interp_pts_x, interp_pts_y);
    // Break up spline points so reference velocity is maintained
    double horizon_x = 30.0; // 30m forward
    double horizon_y = s(horizon_x);
    double hzn_dist = sqrt((horizon_x*horizon_x) + (horizon_y*horizon_y));
    double last_x_l = 0.0;

    // Interpolate remaining waypoints using spline and controlling for speed
    for (int i=0; i<TRAJ_POINTS-prev_size; i++) {
        double N = hzn_dist / (SIM_PD*REF_VEL/MPS_TO_MPH);
        double x_l = last_x_l+(horizon_x)/N;
        double y_l = s(x_l);

        last_x_l = x_l;

        // Undo local transform (un-rotate, un-translate)
        double x_w = (x_l * cos(ref_yaw) - y_l*sin(ref_yaw));
        double y_w = (x_l* sin(ref_yaw) + y_l*cos(ref_yaw));

        // Translate by last position
        x_w += ref_x;
        y_w += ref_y;

        next_x_vals.push_back(x_w);
        next_y_vals.push_back(y_w);
    }
    return {next_x_vals, next_y_vals};
}
