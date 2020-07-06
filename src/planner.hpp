#ifndef PLANNER_H
#define PLANNER_H

#include <cmath> // floor
#include <set>
#include <vector>
#include "json.hpp"

#define MAX_S 6945.554
#define SAFE_S_ADD(s1, s2) positive_modulo((s1) + (s2), MAX_S)
#define SIM_PD 0.02
#define MPS_TO_MPH 2.237
#define LANE_WIDTH 4.0
#define LANE_CENTER_D(lane) ((lane)*LANE_WIDTH + 0.5*LANE_WIDTH)
#define D_TO_LANE(d) ((int)floor(d / LANE_WIDTH))
#define CAR_LENGTH_M 5.0
#define LANE_HORIZON_M 25.0
#define VALID_CAR_GAP_M (CAR_LENGTH_M * 4)
#define CAR_S_TO_FRONT(s) (SAFE_S_ADD(s, CAR_LENGTH_M/2.0))
#define CAR_S_TO_REAR(s)  (s - CAR_LENGTH_M/2.0)
#define LANE_CHANGE_MERGE_BUFFER_M (10.)
#define LANE_CHANGE_EGO_BUFFER_M (15.0)
#define PREP_SPEED_STEP ((5.0 / MPS_TO_MPH) * SIM_PD) // 5mph step per SIM_PD

#define LEFTMOST_LANE 0
#define CENTER_LANE 1
#define RIGHTMOST_LANE 2


using nlohmann::json;
using std::vector;

double positive_modulo(double x, double y);

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

class LaneCar{
    public:
        bool is_init;   // Default: false
        double curr_s;
        double curr_d;
        double speed_ms;
        LaneCar ();
        LaneCar (double _curr_s, double _curr_d, double _speed_ms);
        ~LaneCar () = default;

        inline bool operator< (const LaneCar rhs) const { return is_init && curr_s < rhs.curr_s; }
};

struct LaneGap{
    double gap_follow_s;
    double gap_follow_speed_ms;
    double gap_lead_s;
    double gap_lead_speed_ms;
};

class CarLane {
    public:
        CarLane(double _ego_end_path_s, double _lane_d);
        ~CarLane() = default;
        void add_car_to_lane(double next_car_s, double next_car_speed);
        bool has_nearest_lead_car();
        bool has_nearest_follow_car();
        std::shared_ptr<LaneCar> get_nearest_lead_car();
        std::shared_ptr<LaneCar> get_nearest_follow_car();
        void process_nearest_cars();
        bool canMerge();
        bool canMergeFasterThan(double thresh_speed_ms);
        double getMergeSpeed();
        void print_nearest_cars();
        static bool isValidGap(const std::shared_ptr<LaneCar> lead_car, const std::shared_ptr<LaneCar> follow_car) {
            // Assumes both lead and follow cars are valid
            return (CAR_S_TO_REAR(lead_car->curr_s) - CAR_S_TO_FRONT(follow_car->curr_s)
                    > VALID_CAR_GAP_M);
        }
        static bool isFasterThan(const std::shared_ptr<LaneCar> lead_car, const std::shared_ptr<LaneCar> follow_car, double thresh_speed_ms) {
            return ((lead_car->speed_ms + follow_car->speed_ms) / 2.0 > thresh_speed_ms);
        }
    private:
        std::set<std::shared_ptr<LaneCar>> lane_cars;                               // Ordered set of cars
        std::pair<std::shared_ptr<LaneCar>, std::shared_ptr<LaneCar>> nearest_cars; // first-> follow, second->lead
        double ego_end_path_s;
        double lane_d;
};

enum PlannerState {KEEP_LANE=0, PREP_CL=1, PREP_CR=2, CHANGE_LEFT=3, CHANGE_RIGHT=4};

class Planner {
    public :
        static const int TRAJ_POINTS             = 50;
        static constexpr double MAX_VEL_MS       = 49.5 / MPS_TO_MPH;  // m/s
        static constexpr double MAX_SAFE_ACC_INC = 9.5;                // m/s2
        static constexpr double SAFE_DISTANCE_M  = 40.0;               // m
        Planner(vector<vector<double>> map_waypoints_info);
        ~Planner() {}

        vector<vector<double>> get_next_pos_vals(CarData &cd);
    private:

        PlannerState plan_state;
        bool lane_change_trigerred;

        bool _is_safe_to_change_lanes(CarLane &tgt_lane);
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
