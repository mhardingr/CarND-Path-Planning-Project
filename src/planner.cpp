#include <cmath>  // trig function
#include <algorithm> // min/max
#include <iostream>
#include "helpers.h"  // getXY
#include "spline.h"
#include "planner.h"
using nlohmann::json;
using std::min;
using std::max;
using std::cout;
using std::endl;

#define EPSILON 1e-5

Planner::Planner(vector<vector<double>> map_waypoints_info) {
    map_waypoints_x     = map_waypoints_info[0];
    map_waypoints_y     = map_waypoints_info[1];
    map_waypoints_s     = map_waypoints_info[2];
    map_waypoints_dx    = map_waypoints_info[3];
    map_waypoints_dy    = map_waypoints_info[4];
    curr_lane           = 1;
    curr_vel_ms         = 0.0;
    tgt_lane            = curr_lane;
    tgt_vel_ms          = MAX_VEL_MS;
}

vector<vector<double>> Planner::get_spline_points(CarData &cd, vector<vector<double>> ref_vec) {
    // This function returns spline points translated and rotated to the ref_pos
    vector<double> prev_ref_pos = ref_vec[0];
    vector<double> ref_pos      = ref_vec[1];
    double ref_x    = ref_pos[0];
    double ref_y    = ref_pos[1];
    double ref_yaw  = ref_pos[2];
    double prev_ref_x   = prev_ref_pos[0];
    double prev_ref_y   = prev_ref_pos[1];

    // Create a set of 5 interpolation points from car's current position+heading
    vector<double> spline_pts_x;
    vector<double> spline_pts_y;

    spline_pts_x.push_back(prev_ref_x);
    spline_pts_y.push_back(prev_ref_y);
    spline_pts_x.push_back(ref_x);
    spline_pts_y.push_back(ref_y);

    int interp_spacing = 30;
    // Using tgt_lane for next 3 waypoints
    vector<double> next_interp0 = getXY(cd.car_s+interp_spacing,
                                         LANE_CENTER_D(tgt_lane),
                                         map_waypoints_s, map_waypoints_x,
                                         map_waypoints_y);
    vector<double> next_interp1 = getXY(cd.car_s+2*interp_spacing,
                                         LANE_CENTER_D(tgt_lane),
                                         map_waypoints_s, map_waypoints_x,
                                         map_waypoints_y);
    vector<double> next_interp2 = getXY(cd.car_s+3*interp_spacing,
                                         LANE_CENTER_D(tgt_lane),
                                         map_waypoints_s, map_waypoints_x,
                                         map_waypoints_y);
    spline_pts_x.push_back(next_interp0[0]);
    spline_pts_x.push_back(next_interp1[0]);
    spline_pts_x.push_back(next_interp2[0]);

    spline_pts_y.push_back(next_interp0[1]);
    spline_pts_y.push_back(next_interp1[1]);
    spline_pts_y.push_back(next_interp2[1]);

    // Shift interpolation points to be local frame (current pos+ref_yaw)
    for (int i=0; i<spline_pts_x.size(); i++) {
        double adj_x = spline_pts_x[i] - ref_x;
        double adj_y = spline_pts_y[i] - ref_y;

        spline_pts_x[i] = (adj_x*cos(-ref_yaw) - adj_y*sin(-ref_yaw));
        spline_pts_y[i] = (adj_x*sin(-ref_yaw) + adj_y*cos(-ref_yaw));
    }
    return {spline_pts_x, spline_pts_y};
}

vector<vector<double>> Planner::get_ref_vec(CarData &cd) {
    double ref_x = cd.car_x;
    double ref_y = cd.car_y;
    double ref_yaw = deg2rad(cd.car_yaw);
    double prev_ref_x;
    double prev_ref_y;

    int prev_size = cd.previous_path_x.size();
    if (prev_size < 2) {
        // Just starting, so extrapolate a previous point
        ref_x   = cd.car_x;
        ref_y   = cd.car_y;
        ref_yaw = deg2rad(cd.car_yaw);
        prev_ref_x      = ref_x - cos(ref_yaw);
        prev_ref_y      = ref_y - sin(ref_yaw);
    } else {
        // Use the end of the prev trajectory as start
        ref_x   = cd.previous_path_x[prev_size-1];
        ref_y   = cd.previous_path_y[prev_size-1];
        prev_ref_x  = cd.previous_path_x[prev_size-2];
        prev_ref_y  = cd.previous_path_y[prev_size-2];
        ref_yaw = atan2(ref_y-prev_ref_y, ref_x-prev_ref_x);

        // Use the end of the prev trajectory as start
        if (fabs(prev_ref_x - ref_x) < 0.000001) {
            cout << "Last path's last x is too close to ref_x" << endl;
        }
    }
    vector<double> ref_pos = {ref_x, ref_y, ref_yaw};
    vector<double> prev_ref_pos = {prev_ref_x, prev_ref_y, -1.0};

    return {prev_ref_pos, ref_pos};
}

void Planner::avoid_traffic(CarData &cd, vector<double>ref_pos) {
    // Looks ahead in time and within lane to determine if collision is impending
    // Tunes tgt_vel and tgt_lane to avoid traffic
    double ref_x    = ref_pos[0];
    double ref_y    = ref_pos[1];
    double ref_yaw  = ref_pos[2];

    // How many waypoints remain on the last trajectory
    int prev_size = cd.previous_path_x.size();

    // Loop over all detected cars
    vector<vector<double>> sensor_fusion = cd.sensor_fusion;
    bool car_ahead = false;
    for (int i=0; i<sensor_fusion.size(); i++) {
        double det_car_vx           = sensor_fusion[i][3];
        double det_car_vy           = sensor_fusion[i][4];
        double det_car_speed        = sqrt(det_car_vx*det_car_vx + det_car_vy*det_car_vy);
        double det_car_s            = sensor_fusion[i][5];
        double det_car_d            = sensor_fusion[i][6];

        // Predict car's s-coord at the end of previous trajectory (i.e. rel. to ref_pos)
        double pred_s   = det_car_s + (prev_size * det_car_speed * SIM_PD);

        // Determine if any cars in same lane ahead within the radius
        int det_car_lane    = D_TO_LANE(det_car_d);
        int car_lane        = D_TO_LANE(cd.car_d);
        if (det_car_lane == car_lane && det_car_s > cd.car_s) {
            if (pred_s - cd.end_path_s < SAFE_DISTANCE_M) {
                // Need to slow down to match vel of car AHEAD
                // and potentially try to change lanes
                car_ahead = true;
                cout << "Car ahead! Need to match speed: " << det_car_speed << endl;
                tgt_vel_ms = det_car_speed;
            }
        }
    }
    if (!car_ahead) {
        // No cars ahead, can accelerate if not at MAX_VEL_MS
        cout << "No cars ahead..." << endl;
        if (tgt_vel_ms < MAX_VEL_MS) {
            tgt_vel_ms = MAX_VEL_MS;
        }
    }
}

vector<vector<double>> Planner::get_next_pos_vals(CarData &cd) {
    vector<double> next_x_vals;
    vector<double> next_y_vals;

    // Prepare a reference position: i.e. car's last position in last trajectory
    cout << "previos path has size: " << cd.previous_path_x.size() << endl;
    vector<vector<double>> ref_vec  = get_ref_vec(cd);
    vector<double> prev_ref_pos     = ref_vec[0];
    vector<double> ref_pos          = ref_vec[1];
    double ref_x    = ref_pos[0];
    double ref_y    = ref_pos[1];
    double ref_yaw  = ref_pos[2];

    // Check for traffic and incoming collisions
    avoid_traffic(cd, ref_pos);

    // Prepare spline points
    vector<vector<double>> spline_points = get_spline_points(cd, ref_vec);
    vector<double> spline_pts_x= spline_points[0];
    vector<double> spline_pts_y= spline_points[1];
    curr_lane       = tgt_lane;

    // Prepare spline with interp points
    tk::spline s;
    s.set_points(spline_pts_x, spline_pts_y);

    // Enqueue remaining prev path waypoints to new trajectory
    int prev_size = cd.previous_path_x.size();
    for (int i=0; i<prev_size; i++) {
        next_x_vals.push_back(cd.previous_path_x[i]);
        next_y_vals.push_back(cd.previous_path_y[i]);
    }

    // Break up spline points so reference velocity is maintained
    double horizon_x = 30.0; // 30m forward
    double horizon_y = s(horizon_x);
    double hzn_dist = sqrt((horizon_x*horizon_x) + (horizon_y*horizon_y));
    double last_x_l = 0.0;

    // Interpolate remaining waypoints using spline and controlling for speed
    for (int i=0; i<TRAJ_POINTS-prev_size; i++) {
        // Accelerate/decelerate realistically
        double delta_vel = SIM_PD * SAFE_ACC_INC;
        if (curr_vel_ms < tgt_vel_ms) {
            curr_vel_ms     += delta_vel;
            curr_vel_ms      = min(tgt_vel_ms, curr_vel_ms);
        }
        else if (curr_vel_ms > tgt_vel_ms) {
            curr_vel_ms     -= delta_vel;
            curr_vel_ms      = max(tgt_vel_ms, curr_vel_ms);
        }
        cout << "curr_vel_ms: " << curr_vel_ms << endl;
        if (fabs(curr_vel_ms) < EPSILON) {
            // Avoid divide by 0
            cout << "curr_vel_ms is too close to zero!" << endl;
            next_x_vals.push_back(ref_x);
            next_y_vals.push_back(ref_y);
            continue;
        }

        double N    = hzn_dist / (SIM_PD*curr_vel_ms);
        double x_l  = last_x_l+(horizon_x)/N;
        double y_l  = s(x_l);

        last_x_l    = x_l;

        // Undo local transform (un-rotate, un-translate)
        double x_w = (x_l * cos(ref_yaw) - y_l*sin(ref_yaw));
        double y_w = (x_l * sin(ref_yaw) + y_l*cos(ref_yaw));

        // Translate by last position
        x_w += ref_x;
        y_w += ref_y;

        next_x_vals.push_back(x_w);
        next_y_vals.push_back(y_w);
    } cout << endl;
    return {next_x_vals, next_y_vals};
}
