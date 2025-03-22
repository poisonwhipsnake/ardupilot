#pragma once

#include <AP_Common/AP_Common.h>
#include "AR_WPNav.h"

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

    // true if vehicle has reached desired location
    bool reached_destination() const override;

    // get target curvature in 1/meters
    float get_target_curvature() const { return _target_curvature; }

    // parameter var table
    static const struct AP_Param::GroupInfo var_info[];

    // calculate clothoid parameters for the current path segment
    void calculate_clothoid_parameters(const Location& prev_wp, const Location& curr_wp, const Location& next_wp);

    // get current navigation state
    ClothoidState get_clothoid_state() const { return _clothoid_state; }

    // debug logging
    void log_clothoid_debug() const;
    void log_state_info(const Location& current_loc, float current_heading) const;
    void log_path_points() const;

private:
    // calculate clothoid parameters for the current path segment
    void calculate_clothoid_parameters();

    // update distance and bearing from vehicle's current position to destination
    void update_clothoid_distance_and_bearing();

    // calculate position on clothoid given heading change from start
    void calc_clothoid_position(float heading_change, float& x, float& y) const;

    // calculate heading and curvature at a given distance along clothoid
    void calc_clothoid_properties(float distance, float& heading, float& curvature) const;

    // member variables
    ClothoidState _clothoid_state;    // current state of clothoid navigation
    float _clothoid_entry_length;     // length of entry spiral in meters
    float _clothoid_exit_length;      // length of exit spiral in meters
    float _total_turn_angle;          // total angle change for the turn in radians
    float _entry_angle;               // angle change for entry spiral in radians
    float _exit_angle;                // angle change for exit spiral in radians
    float _straight_length;           // length of straight segment in meters
    float _entry_spiral_heading;     // heading at start of entry spiral
    float _constant_turn_heading;    // heading at start of constant turn
    float _exit_spiral_heading;      // heading at start of exit spiral
    float _straight_heading;         // heading of straight segment
    Vector2f _entry_spiral_start_ned;  // NED position of entry spiral start
    Vector2f _constant_turn_start_ned; // NED position of constant turn start
    Vector2f _exit_spiral_start_ned;   // NED position of exit spiral start
    float _target_curvature;         // target path curvature in 1/meters (positive = turn right, negative = turn left)

    // parameters
    AP_Float _clothoid_rate;          // rate of change of curvature with distance
    AP_Float _pos_error_gain;         // gain for converting lateral position error into a corrective curvature
    AP_Float _long_error_gain;        // gain for converting longitudinal position error into a corrective curvature
    AP_Float _look_ahead_dist;        // look ahead distance for straight segments
    AP_Float _turn_radius;            // minimum turn radius in meters
}; 