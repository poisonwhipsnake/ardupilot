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
    AP_GROUPINFO("POS_P", 2, AR_WPNav_Clothoid, _pos_error_gain, 0.1f),

    // @Param: LONGGAIN
    // @DisplayName: Longitudinal position error gain
    // @Description: Gain for converting longitudinal position error into a corrective curvature. Positive when ahead of ideal point, negative when behind. Higher values cause stronger corrections.
    // @Units: 1/m
    // @Range: 0.01 1.0
    // @Increment: 0.01
    AP_GROUPINFO("POS_I", 3, AR_WPNav_Clothoid, _pos_integrator_gain, 0.02f),

    // @Param: TURNRAD
    // @DisplayName: Turn radius
    // @Description: Minimum radius for constant radius turns
    // @Units: m
    // @Range: 0.5 100.0
    // @Increment: 0.1
    AP_GROUPINFO("TURNRAD", 4, AR_WPNav_Clothoid, _min_turn_radius, 4.0f),

    // @Param: STR_ANG_P
    // @DisplayName: Straight Angle Proportional Gain
    // @Description: Gain for converting lateral angle error into a corrective curvature. Higher values cause stronger corrections.
    // @Units: m
    // @Range: 0.5 100.0
    // @Increment: 0.1
    AP_GROUPINFO("STR_A_P", 5, AR_WPNav_Clothoid, _angle_gain, 1.0f),

    AP_GROUPINFO("I_LIM", 6, AR_WPNav_Clothoid, _xtrack_integrator_distance_limit, 0.5f),

    AP_GROUPINFO("SPEED", 7, AR_WPNav, _speed_max, 3.0),

    AP_GROUPINFO("END_DIST", 8, AR_WPNav_Clothoid, _end_distance, 5.0),

    AP_GROUPINFO("SLOW_ANG", 9, AR_WPNav_Clothoid, _slow_angle, 20.0f),

    AP_GROUPINFO("TURN_SPD_MAX", 10, AR_WPNav_Clothoid, _turn_speed_max, 3.0f),

    AP_GROUPINFO("D_FILTER", 11, AR_WPNav_Clothoid, _d_filter_term, 0.1f),

    AP_GROUPINFO("POS_D", 12, AR_WPNav_Clothoid, _pos_derivative_gain, 0.01f),

    AP_GROUPINFO("V_LEN", 13,  AR_WPNav_Clothoid, _vehicle_length , 3.0f),



    AP_GROUPEND
};

// constructor
AR_WPNav_Clothoid::AR_WPNav_Clothoid(AR_AttitudeControl& atc, AR_PosControl &pos_control) :
    AR_WPNav(atc, pos_control),
    _clothoid_state(ClothoidState::STRAIGHT)
{
    AP_Param::setup_object_defaults(this, var_info);
}

void AR_WPNav_Clothoid::update_speed(float dt)
{
    float desired_speed;
    switch (_clothoid_state) {
        case ClothoidState::STRAIGHT: {
            float stopping_distance = _atc.get_stopping_distance(sqrtf(_speed_max*_speed_max - _turn_speed*_turn_speed));
            if (_distance_to_destination <= turn_start_distance + stopping_distance && fabsf(next_turn.total_turn_angle) > radians(_slow_angle))
            {
                desired_speed = _turn_speed;
            }
            else{
                desired_speed = _speed_max;
            }            
            break;
        }
        case ClothoidState::ENTRY_SPIRAL: 
        case ClothoidState::EXIT_SPIRAL:
        case ClothoidState::CONSTANT_TURN:
        default: {
            if (fabsf(current_turn.total_turn_angle) < radians(_slow_angle)){
                desired_speed = _speed_max;
            }
            else{
                desired_speed = _turn_speed;
            }
            break;
        }
    }
    _desired_speed_limited = _atc.get_desired_speed_accel_limited(desired_speed, dt);
}

// update navigation
void AR_WPNav_Clothoid::update(float dt)
{
    // exit immediately if no current location, origin or destination
    Location current_loc;
    float speed;
    if (!hal.util->get_soft_armed() || !is_destination_valid() || !AP::ahrs().get_location(current_loc) || !_atc.get_forward_speed(speed) || !current_turn.entry_spiral_start.initialised()) {
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

    Vector2f vel_NE = AP::ahrs().groundspeed_vector();
    if (vel_NE.length() > _angle_gain){
        current_heading = atan2f(vel_NE.y, vel_NE.x);
    }

    Vector2f heading_vec(cosf(current_heading), sinf(current_heading));

    // determine which segment we're in and calculate curvature
    float target_curvature = 0;

    switch (_clothoid_state) {
        case ClothoidState::ENTRY_SPIRAL: {
            // calculate heading change from start of entry spiral
            float heading_change = wrap_PI(current_heading - current_turn.entry_spiral_heading);

            float step_distance = current_loc.get_distance(_prev_location);

            _prev_location = current_loc;
            distance_along_segment += step_distance;   
            
            Vector2f ideal_clothoid = clothoid_position_from_s(distance_along_segment, _clothoid_rate);

            if (current_turn.total_turn_angle < 0) {
                ideal_clothoid.y = -ideal_clothoid.y;
            }

            ideal_clothoid.rotate(current_turn.entry_spiral_heading);
            Location ideal_loc = current_turn.entry_spiral_start;
            ideal_loc.offset(ideal_clothoid.x, ideal_clothoid.y);

            Vector2f error_vec = current_loc.get_distance_NE(ideal_loc);

            float clothoid_heading = 0.5*_clothoid_rate*distance_along_segment*distance_along_segment;
            
            if(current_turn.total_turn_angle < 0) {
                clothoid_heading = -clothoid_heading;
            }

            float target_heading = current_turn.entry_spiral_heading + clothoid_heading;

            float curvature = _clothoid_rate*distance_along_segment;
            target_curvature = curvature;
            if(current_turn.total_turn_angle < 0) {
                target_curvature = -target_curvature;
            }
            
            // calculate lateral correction using cross product
            _cross_track_error = -(heading_vec % error_vec);
            _angle_error = wrap_PI(target_heading - current_heading);

            // transition to constant turn if we've reached the entry spiral heading change
            if (fabsf(heading_change) >= fabsf(current_turn.entry_angle)) {
                _clothoid_state = ClothoidState::CONSTANT_TURN;
                distance_along_segment = 0;
            }
            break;
        }
        
        case ClothoidState::CONSTANT_TURN: {
            // constant curvature during turn
            target_curvature = 1.0f / _turn_radius;
            if (current_turn.total_turn_angle < 0) {
                target_curvature = -target_curvature;
            }
            
            // calculate heading change from start of constant turn
            float heading_change = wrap_PI(current_heading - current_turn.constant_turn_heading);
            
            _prev_location = current_loc;
            
            Vector2f current_from_centre = current_turn.turn_centre.get_distance_NE(current_loc);
            
            float bearing_from_centre = current_turn.turn_centre.get_bearing(current_loc);
            
            _cross_track_error = current_from_centre.length() - _turn_radius;

            if(current_turn.total_turn_angle > 0) {
                _cross_track_error = -_cross_track_error;
            }

            if(current_turn.total_turn_angle < 0) {

                _angle_error = wrap_PI(bearing_from_centre - M_PI_2 - current_heading);
            }
            else{
                _angle_error = wrap_PI(bearing_from_centre + M_PI_2 - current_heading);
            }
          
            // transition to exit spiral when we've completed the constant turn portion
            if (fabsf(heading_change) >= fabsf(current_turn.fixed_rate_angle)) {
                _clothoid_state = ClothoidState::EXIT_SPIRAL;
                distance_along_segment = heading_vec * current_turn.exit_spiral_start.get_distance_NE(current_loc);

                //set a reasonable minimum distance along segment
                if (distance_along_segment < -_turn_radius/5) {
                    distance_along_segment = -_turn_radius/5;
                }
            }
            break;
        }
        
        case ClothoidState::EXIT_SPIRAL: {
            // mirror of entry spiral calculations

            float step_distance = current_loc.get_distance(_prev_location);
            _prev_location = current_loc;
            distance_along_segment += step_distance;   

            float distance_to_go = current_turn.clothoid_length - distance_along_segment;
            if(distance_to_go < 0) {
                distance_to_go = 0;
            }

            Vector2f ideal_clothoid = clothoid_position_from_s(distance_to_go, _clothoid_rate);
            if (current_turn.total_turn_angle > 0) {
                ideal_clothoid.y = -ideal_clothoid.y;
            }
            
            ideal_clothoid.rotate(current_turn.turn_complete_heading + M_PI);
            Location ideal_loc = current_turn.exit_spiral_end;
            ideal_loc.offset(ideal_clothoid.x, ideal_clothoid.y);

            float clothoid_heading = 0.5*_clothoid_rate*distance_to_go*distance_to_go;

            if(current_turn.total_turn_angle > 0) {
                clothoid_heading = -clothoid_heading;
            }

            float target_heading = current_turn.turn_complete_heading + clothoid_heading;

            Vector2f error_vec = current_loc.get_distance_NE(ideal_loc);
            
            // calculate curvature based on heading change
            float curvature = _clothoid_rate*distance_to_go;
            target_curvature = curvature; 
            if(current_turn.total_turn_angle < 0) {
                target_curvature = -target_curvature;
            }

            // calculate lateral correction using cross product
            _cross_track_error = -(heading_vec % error_vec);
            _angle_error = wrap_PI(target_heading - current_heading);

            // transition to straight when we've completed the exit spiral
            if (distance_to_go < 0.1f) {
                _clothoid_state = ClothoidState::STRAIGHT;
            }
            break;
        }
        
        case ClothoidState::STRAIGHT:
        default: {
            _cross_track_error = calc_crosstrack_error_straight(current_loc);
            _angle_error = wrap_PI(_current_track_heading - current_heading);
            target_curvature = 0;
            break;

        }
    }




    if(fabsf(_cross_track_error) < _xtrack_integrator_distance_limit){//} && AP::ahrs().get_velocity_NED().length() > 0.2f) {

        _cross_track_integrator += -_cross_track_error * dt;
    //    if (_speed_correction_active<0){
    //        
    //    }
    //    else{
    //        _cross_track_integrator += -_cross_track_error * dt* speed;
    //    }
    }
    else{
        _cross_track_integrator = 0;
    }


    //float smoothed_cross_track_error = ((_d_filter_term * _cross_track_error) + ((1-_d_filter_term) * _previous_cross_track_error));
    
    
    //float derivative = (smoothed_cross_track_error - _previous_cross_track_error) / dt;

    //if (_speed_correction_active>2.0f){
    //    if (speed > 0.2){
    //        derivative = derivative / speed;
    //    }
    //    else{
    //        derivative = 0;
    //    }
    //}
    //_previous_cross_track_error = smoothed_cross_track_error;


    float iTerm = _cross_track_integrator*_pos_integrator_gain;
    //float pTerm = -_cross_track_error*_pos_error_gain;
    //float angTerm = _angle_error*_angle_gain;
    //float dTerm = -derivative*_pos_derivative_gain;
    //float local_speed = speed;
    //if (speed< 0.1f) {
    //    local_speed = 0.1f;
    //}

    //float crosstrack_band = _angle_gain;
    //float ramp = _pos_derivative_gain;

    float shaped_angle_error = _angle_error;

    shaped_angle_error = fmaxf(fminf(shaped_angle_error, M_PI_2), -M_PI_2);

    //if (fabsf(_cross_track_error) < crosstrack_band){
    //    float portion_of_band = fabsf(_cross_track_error)/crosstrack_band;
    //    shaped_angle_error = portion_of_band * _angle_error;
    //}

    float smoothed_angle_error = ((_d_filter_term * shaped_angle_error) + ((1-_d_filter_term) * _previous_angle_error));
    _previous_angle_error = smoothed_angle_error;
        

    smoothed_angle_error = smoothed_angle_error * _pos_derivative_gain;

    float steering_angle_target = smoothed_angle_error - asinf(fmaxf(fminf((_cross_track_error)/_pos_error_gain, 0.99f), -0.99f));
    steering_angle_target = fmaxf(fminf(steering_angle_target, M_PI_2*0.8), -M_PI_2*0.8);
    float stanley = (1/_vehicle_length)*tanf(steering_angle_target);
    
    

    float target_curvature_control =  stanley + iTerm;

     _pid_info.I = iTerm;
    _pid_info.P = _angle_error;
    _pid_info.D = stanley;
    _pid_info.FF = smoothed_angle_error;
    
    _pid_info.actual = -_cross_track_error;
    _pid_info.target = target_curvature_control;
    


    /*if ((_cross_track_error < 0 && _angle_error < -0.1) || (_cross_track_error > 0 && _angle_error > 0.1)){

        float cross_track_factor = (M_PI_4 - (fabsf(_angle_error)-0.1))/M_PI_4;
        if ((fabsf(_angle_error)-0.1) > M_PI_4){
            cross_track_factor = 0;
        }
        target_curvature_control = (cross_track_factor*(-_cross_track_error*_pos_error_gain) )+ (_angle_error*_angle_gain);
        
    }*/

    target_curvature += target_curvature_control;
    /*
    if (target_curvature > 2.0f / _turn_radius) {
        target_curvature = 2.0f / _turn_radius;
    } else if (target_curvature < -2.0f / _turn_radius) {
        target_curvature = -2.0f / _turn_radius;
    }*/

    // apply desired speed and store target curvature
    update_speed(dt);
    _target_curvature = target_curvature;
    _pid_info.slew_rate = target_curvature;
    
    // For compatibility with parent class, calculate turn rate and lateral acceleration
    _desired_turn_rate_rads = _target_curvature * speed;
    _desired_lat_accel = _target_curvature * speed * speed;

    // if we give a really silly combination of waypoints, this ensures more reasonable behaviour
    //if (fabsf(_cross_track_error) > _turn_radius/_pos_derivative_gain){
    //    _clothoid_state = ClothoidState::STRAIGHT;
    //}
   
}

// calculate the crosstrack error
float AR_WPNav_Clothoid::calc_crosstrack_error_straight(const Location& current_loc) const
{
    // calculate the NE position of destination relative to origin
    Vector2f dest_from_origin = _prev_wp.get_distance_NE(_curr_wp);

    // return distance to destination if length of track is very small
    if (dest_from_origin.length() < 1.0e-6f) {
        return current_loc.get_distance_NE(_curr_wp).length();
    }

    // convert to a vector indicating direction only
    dest_from_origin.normalize();

    // calculate the NE position of the vehicle relative to origin
    const Vector2f veh_from_origin = _prev_wp.get_distance_NE(current_loc);

    // calculate distance to target track, for reporting
    return -veh_from_origin % dest_from_origin;
}

// set desired location
bool AR_WPNav_Clothoid::set_desired_location(const Location &destination, Location next_destination)
{
    // call parent
    if (!AR_WPNav::set_desired_location(destination, next_destination)) {
        return false;
    }

    return true;
}

// true if vehicle has reached destination
bool AR_WPNav_Clothoid::reached_destination() const
{
    // we've reached the destination when we're in the straight segment and within the acceptance radius
    Location current_loc;
    if (!AP::ahrs().get_location(current_loc)) {
        return false;
    }
    
    if (fabsf(_angle_error) < M_PI_2){
        float heading_to_destination = current_loc.get_bearing(_destination);
        float relative_heading_to_destination = wrap_PI(heading_to_destination - _current_track_heading);
        if (fabsf(relative_heading_to_destination) > M_PI_2){
            return true;
        } 

        if (_distance_to_destination <= turn_start_distance){
            return true;
        }
        float transition_heading = _destination.get_bearing_to(_prev_wp) * 0.01f;
        Location transition_point = _destination;
        transition_point.offset_bearing(transition_heading, turn_start_distance);
        // If we are slightly off track, need to tick off waypoint based on transition point
        if (current_loc.past_interval_finish_line(_prev_wp, transition_point)){
            return true;
        }
        // Ensure we tick off waypoint if we end up past the destination
        if (current_loc.past_interval_finish_line(_prev_wp, _destination)){
            return true;
        }
    }
  
    return false;
}

// calculate clothoid parameters for the current path segment
void AR_WPNav_Clothoid::calculate_clothoid_parameters(const Location& prev_wp, const Location& curr_wp, const Location& next_wp, bool reset_state, uint16_t clothoid_params)
{
    _turn_radius = LOWBYTE(clothoid_params) * 0.2f; // 0m to 51m in 0.2m increments
    if (_turn_radius < _min_turn_radius) {
        _turn_radius = _min_turn_radius;
    }
    _turn_speed = HIGHBYTE(clothoid_params) * 0.1f / 3.6f; // 0m/s to 25.5km/h in 0.1 km/h increments
    if (_turn_speed < 0.01f || _turn_speed > _turn_speed_max) {
        _turn_speed = _turn_speed_max;
    }

    _prev_wp = prev_wp;
    _curr_wp = curr_wp;
    _next_wp = next_wp;
    
    Location current_loc;
    if (!AP::ahrs().get_location(current_loc)) {
        return;
    }

    if(reset_state) {
        _clothoid_state = ClothoidState::ENTRY_SPIRAL;
        distance_along_segment = 0;
     
        _prev_location = current_loc;

    }   
    else{
        _clothoid_state = ClothoidState::STRAIGHT;
        _prev_wp = current_loc;
        _cross_track_integrator = 0;
    }
    
    current_turn = next_turn;
    
    Vector2f first_vector = _prev_wp.get_distance_NE(_curr_wp);
    _current_track_heading = first_vector.angle();

    if (!_next_wp.initialised()) {
        // Complete the mission at _radius metres from the final waypoint
        turn_start_distance = _end_distance;
        return;
    }

    Vector2f next_vector = _curr_wp.get_distance_NE(_next_wp);


    // calculate total turn angle
    next_turn.total_turn_angle = wrap_PI(next_vector.angle() - first_vector.angle());


    // calculate maximum curvature (at end of entry spiral/start of constant turn)
    float max_curvature = 1.0f / _turn_radius;

    // calculate maximum angle change possible in a single clothoid
    // from clothoid properties: heading = 0.5 * rate * s^2

    float clothoid_angle = 0.5f * max_curvature*max_curvature/_clothoid_rate;

    float curvature_max = 1.0f / _turn_radius;

    next_turn.fixed_rate_angle = 0;

    float a, omega;
    // determine if we need a constant radius turn
    if (fabsf(next_turn.total_turn_angle) > 2.0f * clothoid_angle) {
        // large turn - use entry spiral, constant radius and exit spiral

        next_turn.use_fixed_radius = true;
        next_turn.fixed_rate_angle = fabsf(next_turn.total_turn_angle) - (2.0f * clothoid_angle);
        
        next_turn.clothoid_length = curvature_max/ _clothoid_rate;
        
        if(next_turn.total_turn_angle < 0) {
            clothoid_angle = -clothoid_angle;
        }
        next_turn.entry_angle = clothoid_angle;
        next_turn.exit_angle = next_turn.total_turn_angle - clothoid_angle;

        a = _turn_radius * sinf(next_turn.fixed_rate_angle / 2);
        omega = M_PI_2 - fabsf(clothoid_angle) - (next_turn.fixed_rate_angle / 2);
    } else {
        // review and update this section
        next_turn.use_fixed_radius = false;
        next_turn.entry_angle = next_turn.total_turn_angle * 0.5f;
        next_turn.exit_angle = next_turn.total_turn_angle * 0.5f;
        clothoid_angle = fabsf(next_turn.total_turn_angle * 0.5f);
        next_turn.clothoid_length = sqrtf(2.0f * clothoid_angle / _clothoid_rate);

        a = 0;
        omega =  M_PI_2 - clothoid_angle;
    }

    float x, y;
    calc_clothoid_position(clothoid_angle, x, y);
    float cos_omega = cosf(omega);
    float b;
    if (fabsf(cos_omega) < 1e-6f) {
        b = y / 1e-6f;
    } else {
        b = y / cos_omega;
    }    
    float c = sqrtf((b * b) - (y * y));
    float d = ((a + b) / sinf(fabsf(M_PI - next_turn.total_turn_angle) / 2));
    turn_start_distance = d + x - c;

    // store headings
    next_turn.entry_spiral_heading = radians(_prev_wp.get_bearing_to(_curr_wp) * 0.01f);
    next_turn.constant_turn_heading = next_turn.entry_spiral_heading + next_turn.entry_angle;
    next_turn.exit_spiral_heading = next_turn.entry_spiral_heading + next_turn.exit_angle;
    next_turn.turn_complete_heading = radians(_curr_wp.get_bearing_to(_next_wp) * 0.01f);

    // calculate entry spiral start positions
    next_turn.entry_spiral_start = _curr_wp;
    next_turn.entry_spiral_start.offset_bearing(degrees(next_turn.entry_spiral_heading), -turn_start_distance);
    
    //calculate constant turn start position
    float bearing_to_clothoid_point = tanf(y/x);
    if (next_turn.total_turn_angle < 0) {
        bearing_to_clothoid_point = -bearing_to_clothoid_point;
    }
    float distance_to_clothoid_point = sqrtf(x*x + y*y);
    next_turn.constant_turn_start = next_turn.entry_spiral_start;
    next_turn.constant_turn_start.offset_bearing(degrees(bearing_to_clothoid_point+next_turn.entry_spiral_heading), distance_to_clothoid_point);

    //calculate exit spiral start position
    next_turn.exit_spiral_start = _curr_wp;
    next_turn.exit_spiral_start.offset_bearing(degrees(next_turn.turn_complete_heading), turn_start_distance);
    next_turn.exit_spiral_start.offset_bearing(degrees(-bearing_to_clothoid_point+next_turn.turn_complete_heading), -distance_to_clothoid_point);

    next_turn.exit_spiral_end = _curr_wp;
    next_turn.exit_spiral_end.offset_bearing(degrees(next_turn.turn_complete_heading), turn_start_distance);


    Vector2f turn_center(_turn_radius, 0);
    turn_center.rotate(next_turn.constant_turn_heading);


    if (next_turn.total_turn_angle < 0) {
        // right turn - center is to the right
        turn_center.rotate(-M_PI_2);
    } else {
        // left turn - center is to the left
        turn_center.rotate(M_PI_2);
    }
    next_turn.turn_centre = next_turn.constant_turn_start;
    next_turn.turn_centre.offset(turn_center.x, turn_center.y);

    // _cross_track_error = calc_crosstrack_error_straight(current_loc);
    // _angle_error = wrap_PI(_current_track_heading - AP::ahrs().get_yaw());
}

// calculate position on clothoid given heading change from start
void AR_WPNav_Clothoid::calc_clothoid_position(float heading_change, float& x, float& y) const
{
    // Use Fresnel integrals approximation
    // For small angles, we can use Taylor series approximation
    float abs_heading = fabsf(heading_change);
    Vector2f pos = clothoid_position_from_heading(abs_heading, _clothoid_rate);
    x = pos.x;
    y = pos.y;
    
    /*
    if (abs_heading < 0.1f) {
        float s = abs_heading / _clothoid_rate;
        x = s * (1.0f - abs_heading * abs_heading / 10.0f);
        y = s * s * _clothoid_rate * (1.0f / 3.0f - abs_heading * abs_heading / 42.0f);
    } else {
        // For larger angles, use more terms of Fresnel integrals
        float s = sqrtf(2.0f * abs_heading / _clothoid_rate);
        float c = cosf(abs_heading);
        float s2 = sinf(abs_heading);
        
        x = s * (0.87890625f - (0.51562500f * c) - (0.36328125f * s2));
        y = s * (0.87890625f - (0.51562500f * s2) + (0.36328125f * c));
        
        //if (heading_change < 0) {
           // y = -y;
        //}
    }
        */



}

// Fresnel integrals (simple accurate approximation)
void AR_WPNav_Clothoid::fresnel_CS(float t, float& S, float& C) const
{
    // Use a polynomial approximation (or add lookup later if needed)
    // Abramowitz & Stegun-based power series (valid for t < 2)
    if (t < 1.6f) {
        float t2 = t * t;
        float t3 = t2 * t;
        float t5 = t3 * t2;
        float t7 = t5 * t2;
        float t9 = t7 * t2;

        C = t - (M_PI * M_PI * t5) / 40.0f + (M_PI * M_PI * M_PI * M_PI * t9) / 3456.0f;
        S = (M_PI * t3) / 6.0f - (M_PI * M_PI * M_PI * t7) / 336.0f;
    } else {
        // Asymptotic expansion for large t
        float z = M_PI_2 * t * t;
        C = 0.5f + cosf(z) / (M_PI * t);
        S = 0.5f - sinf(z) / (M_PI * t);
    }
}

/// Computes clothoid position from heading change and clothoid rate
Vector2f AR_WPNav_Clothoid::clothoid_position_from_heading(float heading_change, float clothoid_rate) const
{
    if (heading_change <= 0.0f) {
        return Vector2f(0.0f, 0.0f);
    }

    // Arc length from heading
    float s = sqrtf(2.0f * heading_change / clothoid_rate);

    // Fresnel normalization
    float A = sqrtf(clothoid_rate / M_PI);
    float t = A * s;

    float S, C;
    fresnel_CS(t, S, C);

    float x = C / A;
    float y = S / A;

    return Vector2f(x, y);
}

/// Computes clothoid position from arc length and clothoid rate
Vector2f AR_WPNav_Clothoid::clothoid_position_from_s(float s, float clothoid_rate) const
{
    if (s <= 0.0f) {
        return Vector2f(0.0f, 0.0f);
    }

    float A = sqrtf(clothoid_rate / M_PI);
    float t = A * s;

    float S, C;
    fresnel_CS(t, S, C);

    float x = C / A;
    float y = S / A;

    return Vector2f(x, y);
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

    _wp_bearing_cd = current_loc.get_bearing_to(_destination);

    _desired_heading_cd = ((float)_clothoid_state);
    // calculate cross track error (distance from current position to closest point on line between origin and destination)
    //_cross_track_error = calc_crosstrack_error(current_loc);

    // calculate distance to destination
    _distance_to_destination = current_loc.get_distance(_destination);
}

