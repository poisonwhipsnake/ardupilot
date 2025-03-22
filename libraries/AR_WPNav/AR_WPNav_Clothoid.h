#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <AP_Logger/AP_Logger.h>
#include "AR_WPNav.h"

// log structure definitions
#define LOG_CLOTHOID_MSG 230
struct PACKED log_Clothoid {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t state;
    float total_turn_angle;
    float entry_angle;
    float exit_angle;
    float target_curvature;
    float clothoid_rate;
    float turn_radius;
};

/*
  This class implements Clothoid curve navigation for Rover
  The path between waypoints is divided into:
  1. Entry clothoid (increasing curvature)
  2. Constant radius turn
  3. Exit clothoid (decreasing curvature)
  4. Straight segment
*/
class AR_WPNav_Clothoid : public AR_WPNav {
public:
    // clothoid navigation state
    enum class ClothoidState {
        STRAIGHT = 0,       // following straight path
        ENTRY_SPIRAL = 1,   // in entry clothoid (increasing curvature)
        CONSTANT_TURN = 2,  // in constant radius turn
        EXIT_SPIRAL = 3     // in exit clothoid (decreasing curvature)
    };

    // constructor
    AR_WPNav_Clothoid(AR_AttitudeControl& atc, AR_PosControl &pos_control);

    // update navigation
    void update(float dt) override;

    // set desired location and (optionally) next_destination
    bool set_desired_location(const Location &destination, Location next_destination = Location()) override WARN_IF_UNUSED;

    // true if vehicle has reached destination
    bool reached_destination() const override;

    // get target curvature in 1/meters
    float get_target_curvature() const { return _target_curvature; }

    // get clothoid navigation target curvature
    float get_clothoid_target_curvature() const { return _target_curvature; }




    // parameter var table
    static const struct AP_Param::GroupInfo var_info[];

    // log structure
    static const struct LogStructure log_structure[];

    // calculate clothoid parameters for the current path segment
    void calculate_clothoid_parameters(const Location& prev_wp, const Location& curr_wp, const Location& next_wp, bool reset_state = true);

    // get current navigation state
    ClothoidState get_clothoid_state() const { return _clothoid_state; }

    // debug logging
    void log_clothoid_debug() const;
    void log_state_info(const Location& current_loc, float current_heading) const;
    void log_path_points() const;

private:
    // calculate clothoid parameters for the current path segment
    void calculate_clothoid_parameters();

    float calc_crosstrack_error_strait(const Location& current_loc) const;

    // update distance and bearing from vehicle's current position to destination
    void update_clothoid_distance_and_bearing();

    // calculate position on clothoid given heading change from start
    void calc_clothoid_position(float heading_change, float& x, float& y) const;

    // calculate heading and curvature at a given distance along clothoid
    void calc_clothoid_properties(float distance, float& heading, float& curvature) const;

    

    // member variables
    ClothoidState _clothoid_state;    // current state of clothoid navigation
    struct Clothoid_Turn_Geometry {
        float entry_length;
        float exit_length;
        float total_turn_angle;
        float entry_angle;
        bool use_fixed_radius;
        float exit_angle;
        float straight_length;
        float entry_spiral_heading;
        float constant_turn_heading;
        float exit_spiral_heading;
        Vector2f entry_spiral_start_ned;
        Vector2f constant_turn_start_ned;
        Vector2f exit_spiral_start_ned;
        float target_curvature;
    } next_turn;

    Clothoid_Turn_Geometry current_turn; // geometry of current turn
  

    float _target_curvature;         // target path curvature in 1/meters (positive = turn right, negative = turn left)
    float turn_start_distance;
    float _current_track_heading;     // heading of current waypoint from previous waypoint


    Location _prev_wp;                // previous waypoint
    Location _curr_wp;                // current waypoint
    Location _next_wp;                // next waypoint

    // parameters
    AP_Float _clothoid_rate;          // rate of change of curvature with distance
    AP_Float _pos_error_gain;         // gain for converting lateral position error into a corrective curvature
    AP_Float _long_error_gain;        // gain for converting longitudinal position error into a corrective curvature
    AP_Float _look_ahead_dist;        // look ahead distance for straight segments
    AP_Float _turn_radius;            // minimum turn radius in meters
    AP_Float _angle_gain;             // gain for converting heading error into a corrective curvature
}; 