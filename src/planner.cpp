#include <cmath>  // trig function
#include <algorithm> // min/max
#include <iostream>
#include <set>
#include "helpers.h"  // getXY
#include "spline.h"
#include "planner.hpp"
using nlohmann::json;
using std::min;
using std::max;
using std::cout;
using std::endl;

#define EPSILON 1e-5

CarLane::CarLane(double _ego_end_path_s, double _lane_d) : ego_end_path_s(_ego_end_path_s), lane_d(_lane_d) {}

void CarLane::add_car_to_lane(double next_car_s, double next_car_speed) {
    // Next car_s is the car's predicted s-value at end of trajectory
    lane_cars.insert( LaneCar{next_car_s, lane_d, next_car_speed} );
}

void CarLane::process_nearest_cars() {
    // Sets nearest lead and follow cars
    bool done         = false;
    bool found_follow = false;
    bool found_lead   = false;
    double next_s;

    LaneCar follow_car, lead_car;
    std::set<LaneCar>::iterator next_it = lane_cars.begin();
    while (!done) {
        next_s = next_it->curr_s;
        if (SAFE_S_ADD(next_s, (-ego_end_path_s)) < LANE_HORIZON_M) {
            // Found lead car
            lead_car = *next_it;
            found_lead = true;
        } else if (SAFE_S_ADD(ego_end_path_s, (-next_s)) < LANE_HORIZON_M) {
            // Found follow car
            follow_car = *next_it;
            found_follow = true;
        }
        // Loop termination
        if ((found_lead && found_follow) || next_it == lane_cars.end()) {
            done = true;
        } else {
            next_it++;
        }
    }
    // Save tuple of (follow_car, lead_car)
    nearest_cars = std::pair<LaneCar,LaneCar>(follow_car, lead_car);
}

void CarLane::print_nearest_cars() {
    LaneCar follow_car, lead_car;
    follow_car = nearest_cars.first;
    lead_car = nearest_cars.second;
    cout << "Follow (is_init=" << follow_car.is_init << ", curr_s=" << follow_car.curr_s << ", speed_ms=" << follow_car.speed_ms \
        << ")\nSecond (is_init=" << lead_car.is_init << ", curr_s=" << lead_car.curr_s << ", speed_ms=" << lead_car.speed_ms << ")" << endl;
}

bool CarLane::has_nearest_lead_car() {
    // Returns whether there exists a car in this lane just ahead or alongside ego-car (in s-coordinates)
    return nearest_cars.second.is_init;
}

LaneCar CarLane::get_nearest_lead_car() {
    // Returns the first car ahead or right next to ego car
    return nearest_cars.second;
}

bool CarLane::has_nearest_follow_car() {
    // Returns whether there exists a car in this lane just behind (in s-coordinates)
    return nearest_cars.first.is_init;
}

LaneCar CarLane::get_nearest_follow_car() {
    // Returns the first car behind ego car
    return nearest_cars.first;
}

bool CarLane::canMerge() {

    if (D_TO_LANE(lane_d) < LEFTMOST_LANE || D_TO_LANE(lane_d) > RIGHTMOST_LANE) {
        return false;
    }

    bool has_lead = has_nearest_lead_car();
    bool has_follow = has_nearest_follow_car();
    if (has_lead && !has_follow) {
//        cout << "(canMerge) Lane (" << D_TO_LANE(lane_d) << " has only lead car." << endl;
        double lead_s = get_nearest_lead_car().curr_s;
        if ((CAR_S_TO_REAR(lead_s) - CAR_S_TO_FRONT(ego_end_path_s)) > LANE_CHANGE_MERGE_BUFFER_M) {
//            cout << "\t(canMerge) Lane (" << D_TO_LANE(lane_d) << " has room!" << endl;
            return true;
        }
    } else if (has_lead && has_follow) {
        // Gap exists to left, is it fast enough and large enough?
//        cout << "(canMerge) Lane (" << D_TO_LANE(lane_d) << " has lead and follow." << endl;
        LaneCar lead    = get_nearest_lead_car();
        LaneCar follow  = get_nearest_follow_car();
        if (CarLane::isValidGap(lead, follow)) {
//            cout << "\t(canMerge) Lane (" << D_TO_LANE(lane_d) << " has room!" << endl;
            return true;
        }
    } else if (!has_lead && has_follow) {
//        cout << "\t(canMerge) Lane (" << D_TO_LANE(lane_d) << " has only follow." << endl;
        double follow_s = get_nearest_follow_car().curr_s;
        if ((CAR_S_TO_REAR(ego_end_path_s) - CAR_S_TO_FRONT(follow_s)) > LANE_CHANGE_MERGE_BUFFER_M) {
//            cout << "\t(canMerge) Lane (" << D_TO_LANE(lane_d) << " has room!" << endl;
            return true;
        }
    } else {
        // No cars!
//        cout << "(canMerge) Lane (" << D_TO_LANE(lane_d) << " is empty!" << endl;
//        cout << "\t(canMerge) Lane (" << D_TO_LANE(lane_d) << " has room!" << endl;
        return true;
    }
    return false;

}

bool CarLane::canMergeFasterThan(double thresh_speed_ms) {
    if (D_TO_LANE(lane_d) < LEFTMOST_LANE || D_TO_LANE(lane_d) > RIGHTMOST_LANE) {
        return false;
    }
    // Check left lane first if gap is moving faster than car ahead
    bool has_lead = has_nearest_lead_car();
    bool has_follow = has_nearest_follow_car();
    if (has_lead && !has_follow) {
        if (get_nearest_lead_car().speed_ms > thresh_speed_ms) {
            // only lead-car, and it's moving faster than car_ahead
            cout << "Lane (" << D_TO_LANE(lane_d) << " has no gap, but fast enough lead car." << endl;
            return true;
        }
    } else if (has_lead && has_follow) {
        // Gap exists to left, is it fast enough and large enough?
        LaneCar lead    = get_nearest_lead_car();
        LaneCar follow  = get_nearest_follow_car();
        if (CarLane::isValidGap(lead, follow) &&
                CarLane::isFasterThan(lead, follow, thresh_speed_ms)) {
            cout << "Lane (" << D_TO_LANE(lane_d) << " has valid gap that's fast enough." << endl;
            return true;
        }
    } else if (!has_lead && has_follow) {
        LaneCar follow = get_nearest_follow_car();
        if (follow.speed_ms > thresh_speed_ms) {
            cout << "Lane (" << D_TO_LANE(lane_d) << " has only follow that's fast enough." << endl;
            return true;
        }
    } else {
        // No cars!
        cout << "Lane (" << D_TO_LANE(lane_d) << ") is empty!\t->PREP_CL" << endl;
        return true;
    }
    return false;
}

double CarLane::getMergeSpeed() {
    double merge_speed_ms;

    bool has_lead = has_nearest_lead_car();
    bool has_follow = has_nearest_follow_car();
    if (has_lead && !has_follow) {
        return get_nearest_lead_car().speed_ms;
    } else if (has_lead && has_follow) {
        // Gap exists, return average speed (i.e speed of midpoint)
        LaneCar lead    = get_nearest_lead_car();
        LaneCar follow  = get_nearest_follow_car();
        return (lead.speed_ms + follow.speed_ms) / 2.0;
    } else if (!has_lead && has_follow) {
        LaneCar follow = get_nearest_follow_car();
        return follow.speed_ms;
    } else {
        // No cars!
        return Planner::MAX_VEL_MS;
    }
}

Planner::Planner(vector<vector<double>> map_waypoints_info) {
    map_waypoints_x         = map_waypoints_info[0];
    map_waypoints_y         = map_waypoints_info[1];
    map_waypoints_s         = map_waypoints_info[2];
    map_waypoints_dx        = map_waypoints_info[3];
    map_waypoints_dy        = map_waypoints_info[4];
    curr_lane               = CENTER_LANE;
    curr_vel_ms             = 0.0;
    tgt_lane                = curr_lane;
    tgt_vel_ms              = MAX_VEL_MS;
    plan_state              = KEEP_LANE;
    lane_change_trigerred   = false;
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

    if (fabs(prev_ref_x - ref_x) < EPSILON) {
        double prev_spacing = 2.5;
        vector<double> prev_interp = getXY(SAFE_S_ADD(cd.car_s,(-prev_spacing)),
                                             LANE_CENTER_D(curr_lane),
                                             map_waypoints_s, map_waypoints_x,
                                             map_waypoints_y);
        spline_pts_x.push_back(prev_interp[0]);
        spline_pts_y.push_back(prev_interp[1]);
    } else {
        spline_pts_x.push_back(prev_ref_x);
        spline_pts_y.push_back(prev_ref_y);
    }
    spline_pts_x.push_back(ref_x);
    spline_pts_y.push_back(ref_y);

    int interp_spacing = 30;
    // Using tgt_lane for next 3 waypoints
    vector<double> next_interp0 = getXY(SAFE_S_ADD(cd.car_s,interp_spacing),
                                         LANE_CENTER_D(tgt_lane),
                                         map_waypoints_s, map_waypoints_x,
                                         map_waypoints_y);
    vector<double> next_interp1 = getXY(SAFE_S_ADD(cd.car_s,(2*interp_spacing)),
                                         LANE_CENTER_D(tgt_lane),
                                         map_waypoints_s, map_waypoints_x,
                                         map_waypoints_y);
    vector<double> next_interp2 = getXY(SAFE_S_ADD(cd.car_s,(3*interp_spacing)),
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
    if (prev_size < 2 ) {
        // Just starting or traj is vertical, so extrapolate a previous point
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

    }
    vector<double> ref_pos = {ref_x, ref_y, ref_yaw};
    vector<double> prev_ref_pos = {prev_ref_x, prev_ref_y, -1.0};

    return {prev_ref_pos, ref_pos};
}

bool Planner::_is_safe_to_change_lanes(CarLane &tgt_lane) {
    // Does desired new lane have a gap large enough to turn into?
    if (!tgt_lane.canMerge()) {
        return false;
    }
    return true;
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
    // Determine if there is a car ahead
    // Also find all viable gaps to fit into in left and right lanes
    vector<vector<double>> sensor_fusion= cd.sensor_fusion;
    int ego_lane_i  = D_TO_LANE(cd.car_d);
    double left_lane_d = LANE_CENTER_D(ego_lane_i - 1);// TODO
    double right_lane_d = LANE_CENTER_D(ego_lane_i + 1);
//    cout << "ego_lane_i:" << ego_lane_i << endl;
//    cout << "ego_lane_d " << cd.car_d << "\tleft_d:"<<left_lane_d << "\tright_d:"<<right_lane_d <<endl;
    CarLane left_lane {cd.end_path_s, left_lane_d};
    CarLane ego_lane  {cd.end_path_s, LANE_CENTER_D(ego_lane_i)};
    CarLane right_lane{cd.end_path_s, right_lane_d};
    for (int i=0; i<sensor_fusion.size(); i++) {
        double det_car_vx           = sensor_fusion[i][3];
        double det_car_vy           = sensor_fusion[i][4];
        double det_car_speed        = sqrt(det_car_vx*det_car_vx + det_car_vy*det_car_vy);
        double det_car_s            = sensor_fusion[i][5];
        double det_car_d            = sensor_fusion[i][6];

        // Predict car's s-coord at the end of previous trajectory (i.e. rel. to ref_pos)
        // Note: we keep the raw pred_s (no modulo) in order to maintain car order in
        // each lane
        double pred_s   = SAFE_S_ADD(det_car_s, (prev_size * det_car_speed * SIM_PD));

        // Determine if any cars in same lane ahead within the radius
        int det_car_lane    = D_TO_LANE(det_car_d);
        int car_lane        = D_TO_LANE(cd.car_d);
        if (det_car_lane == car_lane) {
            // Add car to ego lane
            ego_lane.add_car_to_lane(pred_s, det_car_speed);
        } else if (det_car_lane >= LEFTMOST_LANE && det_car_lane < car_lane) {
            // Other car is to our left in our way of traffic
            // update left lane gap
            left_lane.add_car_to_lane(pred_s, det_car_speed);

        } else if (det_car_lane <= RIGHTMOST_LANE && det_car_lane > car_lane) {
            // Other car is to our right in our way of traffic
            right_lane.add_car_to_lane(pred_s, det_car_speed);
        }
    }

    // Process the lanes' cars to find the nearest lead and follow cars
    left_lane.process_nearest_cars();
    ego_lane.process_nearest_cars();
    right_lane.process_nearest_cars();

    // Get car ahead
    bool car_ahead_exists = false;
    LaneCar car_ahead;
    if (ego_lane.has_nearest_lead_car()) {
        car_ahead_exists = true;
        car_ahead = ego_lane.get_nearest_lead_car();
    }

    // Examine options for all possible successor states
    switch (plan_state) {
        case KEEP_LANE:  // Next states: KL, PREP_CL, or PREP_CR
            tgt_lane    = curr_lane;
            if (car_ahead_exists) {
                // Need to at least match lead car's speed
                if ((car_ahead.curr_s - cd.car_s < SAFE_DISTANCE_M)
                   && car_ahead.speed_ms < MAX_VEL_MS) {
                    // We want to switch lanes if there's a car ahead (within some threshold)
                    tgt_vel_ms  = car_ahead.speed_ms;
                    if (tgt_lane != LEFTMOST_LANE && left_lane.canMergeFasterThan(car_ahead.speed_ms)) {
                        cout << "Can change into left lane ("<< ego_lane_i-1 << ") -> PREP_CL" << endl;
                        plan_state = PREP_CL;
                    } else if (tgt_lane != RIGHTMOST_LANE && right_lane.canMergeFasterThan(car_ahead.speed_ms)) {
                        cout << "Can change into right lane ("<< ego_lane_i+1 << ") -> PREP_CR" << endl;
                        plan_state = PREP_CR;
                    } else {
                        // Can't switch lanes because gaps aren't going to get us around car ahead. Need to wait
                        plan_state = KEEP_LANE;
                    }
                }
                else {
                    tgt_vel_ms = MAX_VEL_MS;
                }
            } else {
                // No cars ahead, can accelerate if not at MAX_VEL_MS
                tgt_vel_ms  = MAX_VEL_MS;
                tgt_lane    = curr_lane;
            }
            break;
        case PREP_CL:
            // Can remain in PREP, or can change left
            // Slow down until a safe distance away from car_ahead
            if (car_ahead_exists && (CAR_S_TO_REAR(car_ahead.curr_s) - CAR_S_TO_FRONT(cd.end_path_s)) < LANE_CHANGE_EGO_BUFFER_M) {
                tgt_vel_ms -= PREP_SPEED_STEP * (TRAJ_POINTS - prev_size);
                cout << "(PREP_CL) Slowing down to avoid hitting car ahead. Current speed: " << tgt_vel_ms << endl;
            } else {
                if (_is_safe_to_change_lanes(left_lane)) {
                    cout << "Trigerring lane change LEFT..." << endl;
                    plan_state = CHANGE_LEFT;
                } else {
                    // Should match the speed of car_ahead
                    tgt_vel_ms = car_ahead.speed_ms;
                    cout << "Matching car_ahead speed until safe to trigger lane change LEFT..." << endl;
                }
            }
            break;
        case PREP_CR:
            // Can remain in PREP, or can change left
            // Slow down until a safe distance away from car_ahead
            if (car_ahead_exists && (CAR_S_TO_REAR(car_ahead.curr_s) - CAR_S_TO_FRONT(cd.end_path_s)) < LANE_CHANGE_EGO_BUFFER_M) {
                tgt_vel_ms -= PREP_SPEED_STEP * (TRAJ_POINTS - prev_size);
                cout << "(PREP_CR) Slowing down to avoid hitting car ahead. Current speed: " << tgt_vel_ms << endl;
            } else {
                if (_is_safe_to_change_lanes(right_lane)) {
                    cout << "Trigerring lane change RIGHT..." << endl;
                    plan_state = CHANGE_RIGHT;
                } else {
                    // Should match the speed of car_ahead
                    tgt_vel_ms = car_ahead.speed_ms;
                    cout << "Matching car_ahead speed until safe to trigger lane change RIGHT..." << endl;
                }
            }
            break;
        case CHANGE_LEFT:
            if (!lane_change_trigerred) {
                if (_is_safe_to_change_lanes(left_lane)) {
                    lane_change_trigerred = true;
                    tgt_lane = D_TO_LANE(left_lane_d);
                    tgt_vel_ms= left_lane.getMergeSpeed();
                    cout << "Lane change trigerred!" << endl;
                } else {
                    cout << "Can't change lane safely! Keeping lane instead" << endl;
                    plan_state = KEEP_LANE;
                }
            } else {
                if (tgt_lane == ego_lane_i) {
                    plan_state           = KEEP_LANE;
                    lane_change_trigerred= false;
                    cout << "Lane change complete." << endl;
                } else {
                    cout << "Changing into the left lane ..." << endl;
                }
            }
            break;
        case CHANGE_RIGHT:
            if (!lane_change_trigerred) {
                if (_is_safe_to_change_lanes(right_lane)) {
                    lane_change_trigerred = true;
                    tgt_lane = D_TO_LANE(right_lane_d);
                    tgt_vel_ms= right_lane.getMergeSpeed();
                    cout << "Lane change trigerred!" << endl;
                } else {
                    cout << "Can't change lane safely! Keeping lane instead" << endl;
                    plan_state = KEEP_LANE;
                }
            } else {
                if (tgt_lane == ego_lane_i) {
                    plan_state           = KEEP_LANE;
                    lane_change_trigerred= false;
                    cout << "Lane change complete." << endl;
                } else {
                    cout << "Changing into the right lane ..." << endl;
                }
            }
            break;
    }
}

vector<vector<double>> Planner::get_next_pos_vals(CarData &cd) {
    vector<double> next_x_vals;
    vector<double> next_y_vals;

    //////////////////////////////
    // Trajectory preparation

    // Prepare a reference position: i.e. car's last position in last trajectory
//    cout << "previous path has size: " << cd.previous_path_x.size() << endl;
    vector<vector<double>> ref_vec  = get_ref_vec(cd);
    vector<double> prev_ref_pos     = ref_vec[0];
    vector<double> ref_pos          = ref_vec[1];
    double ref_x    = ref_pos[0];
    double ref_y    = ref_pos[1];
    double ref_yaw  = ref_pos[2];

    // Check for traffic and incoming collisions
    // Sets tgt_lane and tgt_vel for this trajectory
    avoid_traffic(cd, ref_pos);

    // Prepare spline points
    vector<vector<double>> spline_points= get_spline_points(cd, ref_vec);
    vector<double> spline_pts_x         = spline_points[0];
    vector<double> spline_pts_y         = spline_points[1];
    curr_lane                           = tgt_lane;

    // Prepare spline with interp points
    tk::spline s;
    s.set_points(spline_pts_x, spline_pts_y);

    // END Prepare trajectory
    ////////////////////////////

    // Enqueue remaining prev path waypoints to new trajectory
    int prev_size = cd.previous_path_x.size();
    for (int i=0; i<prev_size; i++) {
        next_x_vals.push_back(cd.previous_path_x[i]);
        next_y_vals.push_back(cd.previous_path_y[i]);
    }

    // Break up spline points so reference velocity is maintained
    double horizon_x = 40.0; // 40m forward
    double horizon_y = s(horizon_x);
    double hzn_dist = sqrt((horizon_x*horizon_x) + (horizon_y*horizon_y));
    double last_x_l = 0.0;

    // Interpolate remaining waypoints using spline and controlling for speed
    for (int i=0; i<TRAJ_POINTS-prev_size; i++) {
        // Accelerate/decelerate realistically
        double delta_vel;
        if (lane_change_trigerred) {
            // Lane change long. acceleration should be cut down
            delta_vel = SIM_PD * 0.5 * SAFE_ACC_INC;
        } else {
            delta_vel = SIM_PD * SAFE_ACC_INC;
        }
        if (curr_vel_ms < tgt_vel_ms) {
            curr_vel_ms     += delta_vel;
            curr_vel_ms      = min(tgt_vel_ms, curr_vel_ms);
        }
        else if (curr_vel_ms > tgt_vel_ms) {
            curr_vel_ms     -= delta_vel;
            curr_vel_ms      = max(tgt_vel_ms, curr_vel_ms);
        }
        if (fabs(curr_vel_ms) < EPSILON) {
            // Avoid divide by 0
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
    }
//    cout << "curr lane" << curr_lane << "\tplan_state: " << plan_state << endl;
    return {next_x_vals, next_y_vals};
}
