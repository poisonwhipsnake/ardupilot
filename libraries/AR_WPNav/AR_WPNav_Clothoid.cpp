/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <AP_Math/AP_Math.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_Logger.h>
#include "AR_WPNav_Clothoid.h"
#include <GCS_MAVLink/GCS.h>
#include <AP_AHRS/AP_AHRS.h>

extern const AP_HAL::HAL& hal;

// parameters
const AP_Param::GroupInfo AR_WPNav_Clothoid::var_info[] = {
    // @Param: RATE
    // @DisplayName: Clothoid curvature rate
    // @Description: Rate of change of curvature with distance for clothoid curves
    // @Units: 1/m^3
    // @Range: 0.001 1.0
    // @Increment: 0.001
    AP_GROUPINFO("RATE", 1, AR_WPNav_Clothoid, _clothoid_rate, 0.05f),

    // @Param: POSGAIN
    // @DisplayName: Lateral position error gain
    // @Description: Gain for converting lateral position error into a corrective curvature. Higher values cause stronger corrections.
    // @Units: 1/m
    // @Range: 0.01 1.0
    // @Increment: 0.01
    AP_GROUPINFO("POSGAIN", 2, AR_WPNav_Clothoid, _pos_error_gain, 0.1f),

    // @Param: LONGGAIN
    // @DisplayName: Longitudinal position error gain
    // @Description: Gain for converting longitudinal position error into a corrective curvature. Positive when ahead of ideal point, negative when behind. Higher values cause stronger corrections.
    // @Units: 1/m
    // @Range: 0.01 1.0
    // @Increment: 0.01
    AP_GROUPINFO("LONGGAIN", 3, AR_WPNav_Clothoid, _long_error_gain, 0.05f),

    // @Param: LOOKAHD
    // @DisplayName: Look ahead distance
    // @Description: Distance ahead of the vehicle to look for path corrections in straight line mode
    // @Units: m
    // @Range: 1.0 20.0
    // @Increment: 0.5
    AP_GROUPINFO("LOOKAHD", 4, AR_WPNav_Clothoid, _look_ahead_dist, 5.0f),

    // @Param: TURNRAD
    // @DisplayName: Turn radius
    // @Description: Minimum radius for constant radius turns
    // @Units: m
    // @Range: 0.5 100.0
    // @Increment: 0.1
    AP_GROUPINFO("TURNRAD", 5, AR_WPNav_Clothoid, _turn_radius, 4.0f),

    AP_GROUPEND
};

// constructor
AR_WPNav_Clothoid::AR_WPNav_Clothoid(AR_AttitudeControl& atc, AR_PosControl &pos_control) :
    AR_WPNav(atc, pos_control),
    _clothoid_state(ClothoidState::STRAIGHT),
    _clothoid_entry_length(0.0f),
    _clothoid_exit_length(0.0f),
    _total_turn_angle(0.0f),
    _entry_angle(0.0f),
    _exit_angle(0.0f),
    _straight_length(0.0f),
    _entry_spiral_heading(0.0f),
    _constant_turn_heading(0.0f),
    _exit_spiral_heading(0.0f),
    _straight_heading(0.0f),
    _entry_spiral_start_ned(0.0f, 0.0f),
    _constant_turn_start_ned(0.0f, 0.0f),
    _exit_spiral_start_ned(0.0f, 0.0f),
    _target_curvature(0.0f)
{
    AP_Param::setup_object_defaults(this, var_info);
}

// update navigation
void AR_WPNav_Clothoid::update(float dt)
{
    // exit immediately if no current location, origin or destination
    Location current_loc;
    float speed;
    if (!hal.util->get_soft_armed() || !is_destination_valid() || !AP::ahrs().get_location(current_loc) || !_atc.get_forward_speed(speed)) {
        _desired_speed_limited = _atc.get_desired_speed_accel_limited(0.0f, dt);
        _desired_lat_accel = 0.0f;
        _desired_turn_rate_rads = 0.0f;
        _target_curvature = 0.0f;
        return;
    }

    // update distance and bearing to destination
    update_clothoid_distance_and_bearing();

    // get current vehicle heading
    float current_heading = AP::ahrs().get_yaw();
    Vector2f heading_vec(cosf(current_heading), sinf(current_heading));

    // log current state for debugging
    log_clothoid_debug();
    log_state_info(current_loc, current_heading);

    // determine which segment we're in and calculate desired speed and curvature
    float desired_speed = _reversed ? -_speed_max : _speed_max;
    float target_curvature = 0;

    switch (_clothoid_state) {
        case ClothoidState::ENTRY_SPIRAL: {
            // calculate heading change from start of entry spiral
            float heading_change = wrap_PI(current_heading - _entry_spiral_heading);
            
            // calculate ideal position on clothoid
            float ideal_x, ideal_y;
            calc_clothoid_position(heading_change, ideal_x, ideal_y);
            
            // convert current position to NED and calculate error
            Vector3f current_pos;
            if (!current_loc.get_vector_from_origin_NEU(current_pos)) {
                return;
            }
            Vector2f current_pos_ned(current_pos.x, current_pos.y);
            Vector2f error_vec = current_pos_ned - _entry_spiral_start_ned - Vector2f(ideal_x, ideal_y);
            
            // calculate curvature based on heading change
            float curvature = _clothoid_rate * sqrtf(2.0f * fabsf(heading_change) / _clothoid_rate);
            target_curvature = curvature;
            
            // calculate lateral correction using cross product
            float lateral_correction = -_pos_error_gain * (heading_vec % error_vec);
            
            // calculate longitudinal correction using dot product
            float longitudinal_correction = _long_error_gain * (heading_vec * error_vec);
            
            // combine corrections
            target_curvature += lateral_correction + longitudinal_correction;
            
            // transition to constant turn if we've reached the entry spiral heading change
            if (fabsf(heading_change) >= fabsf(_entry_angle)) {
                _clothoid_state = ClothoidState::CONSTANT_TURN;
            }
            break;
        }
        
        case ClothoidState::CONSTANT_TURN: {
            // constant curvature during turn
            target_curvature = 1.0f / _turn_radius;
            if (_total_turn_angle < 0) {
                target_curvature = -target_curvature;
            }
            
            // calculate heading change from start of constant turn
            float heading_change = wrap_PI(current_heading - _constant_turn_heading);
            
            // calculate turn center - perpendicular to initial heading at turn radius distance
            Vector2f turn_center;
            if (_total_turn_angle > 0) {
                // right turn - center is to the right
                turn_center.x = -_turn_radius * sinf(_constant_turn_heading);
                turn_center.y = _turn_radius * cosf(_constant_turn_heading);
            } else {
                // left turn - center is to the left
                turn_center.x = _turn_radius * sinf(_constant_turn_heading);
                turn_center.y = -_turn_radius * cosf(_constant_turn_heading);
            }
            turn_center += _constant_turn_start_ned;
            
            // calculate ideal position on circle as a vector from turn center
            Vector2f ideal_pos(_turn_radius * cosf(heading_change),
                             _turn_radius * sinf(heading_change));
            if (_total_turn_angle < 0) {
                ideal_pos.y = -ideal_pos.y;  // mirror for left turns
            }
            
            // rotate by initial heading and offset from turn center
            ideal_pos.rotate(_constant_turn_heading);
            ideal_pos += turn_center;
            
            // convert current position to NED and calculate error
            Vector3f current_pos;
            if (!current_loc.get_vector_from_origin_NEU(current_pos)) {
                return;
            }
            Vector2f current_pos_ned(current_pos.x, current_pos.y);
            Vector2f error_vec = current_pos_ned - ideal_pos;
            
            // calculate lateral correction using cross product
            float lateral_correction = -_pos_error_gain * (heading_vec % error_vec);
            
            // calculate longitudinal correction using dot product
            float longitudinal_correction = _long_error_gain * (heading_vec * error_vec);
            
            // combine corrections
            target_curvature += lateral_correction + longitudinal_correction;
            
            // transition to exit spiral when we've completed the constant turn portion
            float const_turn_angle = _total_turn_angle - _entry_angle;
            if (fabsf(heading_change) >= fabsf(const_turn_angle)) {
                _clothoid_state = ClothoidState::EXIT_SPIRAL;
            }
            break;
        }
        
        case ClothoidState::EXIT_SPIRAL: {
            // mirror of entry spiral calculations
            float heading_change = wrap_PI(current_heading - _exit_spiral_heading);
            
            float ideal_x, ideal_y;
            calc_clothoid_position(-heading_change, ideal_x, ideal_y); // note negative heading change for exit spiral
            
            // convert current position to NED and calculate error
            Vector3f current_pos;
            if (!current_loc.get_vector_from_origin_NEU(current_pos)) {
                return;
            }
            Vector2f current_pos_ned(current_pos.x, current_pos.y);
            Vector2f error_vec = current_pos_ned - _exit_spiral_start_ned - Vector2f(ideal_x, ideal_y);
            
            // calculate curvature based on heading change
            float curvature = _clothoid_rate * sqrtf(2.0f * fabsf(heading_change) / _clothoid_rate);
            target_curvature = -curvature; // note negative for exit spiral
            
            // calculate lateral correction using cross product
            float lateral_correction = -_pos_error_gain * (heading_vec % error_vec);
            
            // calculate longitudinal correction using dot product
            float longitudinal_correction = _long_error_gain * (heading_vec * error_vec);
            
            // combine corrections
            target_curvature += lateral_correction + longitudinal_correction;
            
            // transition to straight when we've completed the exit spiral
            if (fabsf(heading_change) >= fabsf(_exit_angle)) {
                _clothoid_state = ClothoidState::STRAIGHT;
            }
            break;
        }
        
        case ClothoidState::STRAIGHT:
        default: {
            // simple straight line navigation with cross track correction
            if (!is_zero(_cross_track_error)) {
                // Convert cross track error to a curvature command
                // Using the relation: curvature = d(heading)/ds ≈ heading_error/look_ahead_distance
                float heading_error = wrap_PI(radians(_wp_bearing_cd * 0.01f));
                target_curvature = constrain_float(heading_error / _look_ahead_dist, -1.0f/_turn_radius, 1.0f/_turn_radius);
            }
            break;
        }
    }

    // apply desired speed and store target curvature
    _desired_speed_limited = _atc.get_desired_speed_accel_limited(desired_speed, dt);
    _target_curvature = target_curvature;
    
    // For compatibility with parent class, calculate turn rate and lateral acceleration
    _desired_turn_rate_rads = _target_curvature * speed;
    _desired_lat_accel = _target_curvature * speed * speed;
}

// set desired location
bool AR_WPNav_Clothoid::set_desired_location(const Location &destination, Location next_destination)
{
    // call parent
    if (!AR_WPNav::set_desired_location(destination, next_destination)) {
        return false;
    }

    // reset clothoid state
    _clothoid_state = ClothoidState::STRAIGHT;

    return true;
}

// true if vehicle has reached destination
bool AR_WPNav_Clothoid::reached_destination() const
{
    // we've reached the destination when we're in the straight segment and within the acceptance radius
    return (_clothoid_state == ClothoidState::STRAIGHT && _distance_to_destination <= _radius);
}

// calculate clothoid parameters for the current path segment
void AR_WPNav_Clothoid::calculate_clothoid_parameters(const Location& prev_wp, const Location& curr_wp, const Location& next_wp)
{
    // calculate total turn angle
    _total_turn_angle = wrap_PI((next_wp.get_bearing_to(curr_wp) - prev_wp.get_bearing_to(curr_wp)) * 0.01f);

    // calculate maximum curvature (at end of entry spiral/start of constant turn)
    float max_curvature = 1.0f / _turn_radius;

    // calculate clothoid length based on rate of curvature change
    _clothoid_entry_length = sqrtf(max_curvature / _clothoid_rate);
    _clothoid_exit_length = _clothoid_entry_length;

    // calculate maximum angle change possible in a single clothoid
    // from clothoid properties: heading = 0.5 * rate * s^2
    // at max length: heading = 0.5 * rate * (sqrt(max_curv/rate))^2 = 0.5 * max_curv
    float max_clothoid_angle = 0.5f * max_curvature;

    // determine if we need a constant radius turn
    if (fabsf(_total_turn_angle) > 2.0f * max_clothoid_angle) {
        // large turn - use entry spiral, constant radius and exit spiral
        _entry_angle = max_clothoid_angle;
        _exit_angle = _total_turn_angle - max_clothoid_angle;
    } else {
        // small turn - split angle between entry and exit spirals
        _entry_angle = _total_turn_angle * 0.5f;
        _exit_angle = _total_turn_angle * 0.5f;
    }

    // calculate clothoid lengths
    _clothoid_entry_length = sqrtf(2.0f * fabsf(_entry_angle) / _clothoid_rate);
    _clothoid_exit_length = sqrtf(2.0f * fabsf(_exit_angle) / _clothoid_rate);

    // calculate straight segment length
    _straight_length = curr_wp.get_distance(next_wp);

    // store headings
    _entry_spiral_heading = prev_wp.get_bearing_to(curr_wp) * 0.01f;
    _constant_turn_heading = _entry_spiral_heading + _entry_angle;
    _exit_spiral_heading = _constant_turn_heading + (_total_turn_angle - _entry_angle - _exit_angle);
    _straight_heading = curr_wp.get_bearing_to(next_wp) * 0.01f;

    // store start positions
    Vector3f curr_pos;
    if (!curr_wp.get_vector_from_origin_NEU(curr_pos)) {
        return;
    }
    _entry_spiral_start_ned.x = curr_pos.x;
    _entry_spiral_start_ned.y = curr_pos.y;

    // calculate constant turn start position
    float entry_x, entry_y;
    calc_clothoid_position(_entry_angle, entry_x, entry_y);
    _constant_turn_start_ned = _entry_spiral_start_ned + Vector2f(entry_x, entry_y);

    // calculate exit spiral start position
    float const_turn_angle = _total_turn_angle - _entry_angle - _exit_angle;
    float const_turn_radius = _turn_radius;
    if (_total_turn_angle < 0) {
        const_turn_radius = -const_turn_radius;
    }
    float const_turn_x = const_turn_radius * (sinf(_constant_turn_heading + const_turn_angle) - sinf(_constant_turn_heading));
    float const_turn_y = const_turn_radius * (-cosf(_constant_turn_heading + const_turn_angle) + cosf(_constant_turn_heading));
    _exit_spiral_start_ned = _constant_turn_start_ned + Vector2f(const_turn_x, const_turn_y);

    // log path points when starting a new turn
    log_path_points();
}

// calculate position on clothoid given heading change from start
void AR_WPNav_Clothoid::calc_clothoid_position(float heading_change, float& x, float& y) const
{
    // Use Fresnel integrals approximation
    // For small angles, we can use Taylor series approximation
    float abs_heading = fabsf(heading_change);
    if (abs_heading < 0.1f) {
        float s = heading_change / _clothoid_rate;
        x = s * (1.0f - heading_change * heading_change / 10.0f);
        y = s * s * _clothoid_rate * (1.0f / 3.0f - heading_change * heading_change / 42.0f);
    } else {
        // For larger angles, use more terms of Fresnel integrals
        float s = sqrtf(2.0f * abs_heading / _clothoid_rate);
        float c = cosf(heading_change);
        float s2 = sinf(heading_change);
        
        x = s * (0.87890625f - 0.51562500f * c - 0.36328125f * s2);
        y = s * (0.87890625f - 0.51562500f * s2 + 0.36328125f * c);
        
        if (heading_change < 0) {
            y = -y;
        }
    }
}

// calculate heading and curvature at a given distance along clothoid
void AR_WPNav_Clothoid::calc_clothoid_properties(float distance, float& heading, float& curvature) const
{
    curvature = _clothoid_rate * distance;
    heading = 0.5f * _clothoid_rate * distance * distance;
}

// update distance and bearing from vehicle's current position to destination
void AR_WPNav_Clothoid::update_clothoid_distance_and_bearing()
{
    Location current_loc;
    if (!AP::ahrs().get_location(current_loc)) {
        return;
    }

    // get current position in NED
    Vector3f current_pos;
    if (!current_loc.get_vector_from_origin_NEU(current_pos)) {
        return;
    }
    Vector2f current_pos_ned(current_pos.x, current_pos.y);

    // calculate bearing and crosstrack error based on current navigation segment
    switch (_clothoid_state) {
        case ClothoidState::ENTRY_SPIRAL: {
            // calculate position relative to entry spiral start
            Vector2f rel_pos = current_pos_ned - _entry_spiral_start_ned;
            float heading_change = atan2f(rel_pos.y, rel_pos.x) - _entry_spiral_heading;
            _wp_bearing_cd = degrees(wrap_PI(_entry_spiral_heading + heading_change)) * 100;
            break;
        }
            
        case ClothoidState::CONSTANT_TURN: {
            // calculate position relative to constant turn start
            Vector2f rel_pos = current_pos_ned - _constant_turn_start_ned;
            float heading_change = atan2f(rel_pos.y, rel_pos.x) - _constant_turn_heading;
            _wp_bearing_cd = degrees(wrap_PI(_constant_turn_heading + heading_change)) * 100;
            break;
        }
            
        case ClothoidState::EXIT_SPIRAL: {
            // calculate position relative to exit spiral start
            Vector2f rel_pos = current_pos_ned - _exit_spiral_start_ned;
            float heading_change = atan2f(rel_pos.y, rel_pos.x) - _exit_spiral_heading;
            _wp_bearing_cd = degrees(wrap_PI(_exit_spiral_heading + heading_change)) * 100;
            break;
        }
            
        case ClothoidState::STRAIGHT:
        default:
            _wp_bearing_cd = current_loc.get_bearing_to(_destination);
            break;
    }

    // calculate cross track error (distance from current position to closest point on line between origin and destination)
    _cross_track_error = calc_crosstrack_error(current_loc);

    // calculate distance to destination
    _distance_to_destination = current_loc.get_distance(_destination);
}

// log debug information about clothoid navigation
void AR_WPNav_Clothoid::log_clothoid_debug() const
{
    // @LoggerMessage: CLTH
    // @Description: Clothoid Navigation Debug
    // @Field: TimeUS: Time since system startup
    // @Field: State: Navigation state (0:Straight, 1:Entry, 2:Turn, 3:Exit)
    // @Field: TargCrv: Target path curvature
    // @Field: TotAngle: Total turn angle
    // @Field: EntAngle: Entry spiral angle
    // @Field: ExtAngle: Exit spiral angle
    // @Field: TurnRad: Turn radius
    AP::logger().Write(
        "CLTH",
        "TimeUS,State,TargCrv,TotAngle,EntAngle,ExtAngle,TurnRad",
        "s#1rrrrm",
        "F000000",
        "QBfffff",
        AP_HAL::micros64(),
        (uint8_t)_clothoid_state,
        (double)_target_curvature,
        (double)degrees(_total_turn_angle),
        (double)degrees(_entry_angle),
        (double)degrees(_exit_angle),
        (double)_turn_radius
    );
}

// log current state information
void AR_WPNav_Clothoid::log_state_info(const Location& current_loc, float current_heading) const
{
    // get current position in NED
    Vector3f current_pos;
    if (!current_loc.get_vector_from_origin_NEU(current_pos)) {
        return;
    }

    // @LoggerMessage: CLTS
    // @Description: Clothoid State Information
    // @Field: TimeUS: Time since system startup
    // @Field: PosX: Current position X
    // @Field: PosY: Current position Y
    // @Field: Hdg: Current heading
    // @Field: IdealX: Ideal position X
    // @Field: IdealY: Ideal position Y
    // @Field: ErrX: Position error X
    // @Field: ErrY: Position error Y
    AP::logger().Write(
        "CLTS",
        "TimeUS,PosX,PosY,Hdg,IdealX,IdealY,ErrX,ErrY",
        "smmmmmm",
        "F000000",
        "Qfffffff",
        AP_HAL::micros64(),
        (double)current_pos.x,
        (double)current_pos.y,
        (double)degrees(current_heading),
        (double)0.0f,  // TODO: Add ideal position calculation based on state
        (double)0.0f,
        (double)0.0f,  // TODO: Add error vector calculation
        (double)0.0f
    );
}

// log key path points
void AR_WPNav_Clothoid::log_path_points() const
{
    // @LoggerMessage: CLTP
    // @Description: Clothoid Path Points
    // @Field: TimeUS: Time since system startup
    // @Field: Type: Point type (0:Entry, 1:Turn, 2:Exit, 3:Center)
    // @Field: PosX: Point position X
    // @Field: PosY: Point position Y
    // @Field: Hdg: Heading at point
    Vector3f pos;

    // log entry spiral start
    AP::logger().Write(
        "CLTP",
        "TimeUS,Type,PosX,PosY,Hdg",
        "s#mmr",
        "F0000",
        "QBfff",
        AP_HAL::micros64(),
        0,
        (double)_entry_spiral_start_ned.x,
        (double)_entry_spiral_start_ned.y,
        (double)degrees(_entry_spiral_heading)
    );

    // log constant turn start
    AP::logger().Write(
        "CLTP",
        "TimeUS,Type,PosX,PosY,Hdg",
        "s#mmr",
        "F0000",
        "QBfff",
        AP_HAL::micros64(),
        1,
        (double)_constant_turn_start_ned.x,
        (double)_constant_turn_start_ned.y,
        (double)degrees(_constant_turn_heading)
    );

    // log exit spiral start
    AP::logger().Write(
        "CLTP",
        "TimeUS,Type,PosX,PosY,Hdg",
        "s#mmr",
        "F0000",
        "QBfff",
        AP_HAL::micros64(),
        2,
        (double)_exit_spiral_start_ned.x,
        (double)_exit_spiral_start_ned.y,
        (double)degrees(_exit_spiral_heading)
    );
} 