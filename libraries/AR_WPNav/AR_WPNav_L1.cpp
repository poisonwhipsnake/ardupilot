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

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Common/AP_Common.h> // Include this header for constrain_float
#include <AP_HAL/AP_HAL.h>
#include "AR_WPNav_L1.h"
#include <GCS_MAVLink/GCS.h>
#include <AP_InternalError/AP_InternalError.h>

extern const AP_HAL::HAL& hal;

// table of user settable parameters
const AP_Param::GroupInfo AR_WPNav_L1::var_info[] = {
    // @Param: PERIOD
    // @DisplayName: L1 control period
    // @Description: Period in seconds of L1 tracking loop. This parameter is the primary control for agressiveness of turns in auto mode. This needs to be larger for less responsive airframes. The default of 20 is quite conservative, but for most RC aircraft will lead to reasonable flight. For smaller more agile aircraft a value closer to 15 is appropriate, or even as low as 10 for some very agile aircraft. When tuning, change this value in small increments, as a value that is much too small (say 5 or 10 below the right value) can lead to very radical turns, and a risk of stalling.
    // @Units: s
    // @Range: 1 60
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("PERIOD",    0, AR_WPNav_L1, _L1_period, 20),

    // @Param: DAMPING
    // @DisplayName: L1 control damping ratio
    // @Description: Damping ratio for L1 control. Increase this in increments of 0.05 if you are getting overshoot in path tracking. You should not need a value below 0.7 or above 0.85.
    // @Range: 0.6 1.0
    // @Increment: 0.05
    // @User: Advanced
    AP_GROUPINFO("DAMPING",   1, AR_WPNav_L1, _L1_damping, 0.75f),

    // @Param: XTRACK_I
    // @DisplayName: L1 control crosstrack integrator gain
    // @Description: Crosstrack error integrator gain. This gain is applied to the crosstrack error to ensure it converges to zero. Set to zero to disable. Smaller values converge slower, higher values will cause crosstrack error oscillation.
    // @Range: 0 0.1
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("XTRACK_I",   2, AR_WPNav_L1, _L1_xtrack_i_gain, 0.1),

    // @Param: Turn Rate correction factor
	// @DisplayName: Maximum Roll Acceleration
	// @Description: Maximum Acceleration of Roll rate for nav_rll (deg/s/s).
	// @Range: 0.1 2
	// @Increment: 1
	// @User: User
	AP_GROUPINFO("TRCF", 8, AR_WPNav_L1, _turn_rate_correction_factor,1.0f),

    // @Param: Auto_L1_Period
    // @DisplayName:  integral gain to account for wind speed measurement error
    // @Description: Blank
    // @Units: 1/s
    // @Range: 0 89
    // @User: Advanced
    AP_GROUPINFO("A_PERIOD", 9, AR_WPNav_L1, _L1_Auto_Period, 40.0f),

    AP_GROUPINFO("TEXIT", 10, AR_WPNav_L1, _L1_Turn_Exit_Fraction, 1.0f),

    AP_GROUPINFO("TGAIN", 11, AR_WPNav_L1, _L1_Mid_Turn_Gain, 1.0f),

    AP_GROUPINFO("TDELAY", 12, AR_WPNav_L1, _L1_Turn_Delay,  0.0f),

    AP_GROUPINFO("GTR",14, AR_WPNav_L1, _ground_turn_radius, 20.0),

    AP_GROUPINFO("GTCF",15, AR_WPNav_L1, _ground_turn_correction_factor, 20.0),

    AP_GROUPINFO("GTEI",16, AR_WPNav_L1, _ground_turn_early_initiation, 50.0),

    AP_GROUPINFO("T_ERROR",19, AR_WPNav_L1, _max_auto_point_distance, 1000.0f),

    AP_GROUPINFO("E_DECEL",21, AR_WPNav_L1, _emergency_stop_deceleration, 3.1f),

    AP_GROUPINFO("GRE_F_TC", 22, AR_WPNav_L1, _ground_risk_filter_tc, 5.0f),

    AP_GROUPINFO("GRE_EX_T", 23, AR_WPNav_L1, _ground_risk_exclusion_timeout, 5.0f),

    AP_GROUPINFO("STR_MAX", 24, AR_WPNav_L1, _steering_angle_max_param, 30.0f),

    AP_GROUPINFO("STR_VEL", 25, AR_WPNav_L1, _steering_angle_velocity_param, 20.0f),

    AP_GROUPINFO("STR_ACCEL", 26, AR_WPNav_L1, _steering_angle_acceleration_param, 50.0f),

    AP_GROUPINFO("STR_WB", 27, AR_WPNav_L1, _steering_wheelbase, 0.4f),

    AP_GROUPINFO("SPD_MAX", 28, AR_WPNav_L1, _speed_max_param, 2.0f),

    AP_GROUPINFO("ACCEL_MAX", 29, AR_WPNav_L1, _accel_max, 1.0f),

    AP_GROUPINFO("DECEL_MAX", 30, AR_WPNav_L1, _decel_max, 1.0f),

    AP_GROUPINFO("TURN_G",31, AR_WPNav_L1, _turn_lateral_G , 0.1f),

    AP_GROUPEND
};


void AR_WPNav_L1::init(float speed_max)
{
    //initilaize locations to zero
    auto_turn_centre.zero();
    next_auto_waypoint.zero();
    prev_auto_waypoint.zero();
    _base_speed_max = _speed_max_param;
    current_speed = 0;


}


// update navigation
void AR_WPNav_L1::update(float dt)
{
  
    // exit immediately if no current location, origin or destination
    Location current_loc;
    float speed;
    if (!hal.util->get_soft_armed() || !_orig_and_dest_valid || !AP::ahrs().get_location(current_loc) || !_atc.get_forward_speed(speed)) {
        _desired_speed_limited = _atc.get_desired_speed_accel_limited(0.0f, dt);
        _desired_lat_accel = 0.0f;
        _desired_turn_rate_rads = 0.0f;
        _cross_track_error = 0;
        return;
    }

    _last_update_ms = AP_HAL::millis();

    update_waypoint( prev_auto_waypoint,current_auto_waypoint);

    Vector2f turnDistance = turn_distance_ground_frame(prev_auto_waypoint,current_loc, current_auto_waypoint,next_auto_waypoint, 5.0f);

    float distance_to_waypoint = current_loc.get_distance_NE(current_auto_waypoint).length() ;
    float turn_distance = turnDistance.length();

    //float target_speed = current_speed;

    float distance_to_turn = distance_to_waypoint - turn_distance;

    if (distance_to_turn <0){
        _reached_destination = true;

    }
    else{
        _reached_destination = false;
    }
    float turn_speed = sqrtf(waypoint_radius*_turn_lateral_G*GRAVITY_MSS);

    float slow_down_distance = 0.5f*((speed*speed) -(turn_speed*turn_speed))/_decel_max;

    float target_speed = waypoint_speed;

    if (distance_to_turn < slow_down_distance ){
        target_speed = turn_speed;
    }
    if (!initial_turn_complete()){
        target_speed = sqrtf(prev_waypoint_radius*_turn_lateral_G*GRAVITY_MSS);
    }


    float steering_angle_max = DEG_TO_RAD*_steering_angle_max_param;
    float steering_angle_max_rate = DEG_TO_RAD*_steering_angle_velocity_param;
    float steering_angle_max_accel = DEG_TO_RAD*_steering_angle_acceleration_param;

    float steering_angle = nav_steering_angle(_ahrs.groundspeed_vector().length(), _steering_wheelbase, steering_angle_max, steering_angle_max_rate, steering_angle_max_accel, prev_waypoint_radius);

    float turn_radius = _steering_wheelbase/tanf(steering_angle);

    // calculate the desired turn rate from velocity and turn radius
    float desired_turn_rate = _ahrs.groundspeed_vector().length()/turn_radius;

    
    _cross_track_error = calc_crosstrack_error(current_loc);

    // update position controller
    _pos_control.set_reversed(_reversed);
    _pos_control.overRideTurnRate(desired_turn_rate);
    _pos_control.overRideSpeed(target_speed);
    _pos_control.update(dt);


    _desired_speed_limited = _pos_control.get_desired_speed();
    _desired_turn_rate_rads = desired_turn_rate;
    _desired_lat_accel = _pos_control.get_desired_lat_accel();

}


void AR_WPNav_L1::update_speed_demand(float dt)
{
    float desired_speed = _base_speed_max;
    float speed = _ahrs.groundspeed_vector().length();
    float speed_error = desired_speed - speed;
    float speed_change_max = _atc.get_accel_max() * dt;
    float speed_change = constrain_float(speed_error, -speed_change_max, speed_change_max);
    _base_speed_max = speed + speed_change;
}   





void AR_WPNav_L1::reset(){
    _initial_turn_complete = true;
    auto_turn_centre.zero();
}

float AR_WPNav_L1::get_yaw()
{
    if (_reverse) {
        return wrap_PI(M_PI + _ahrs.get_yaw());
    }
    return _ahrs.get_yaw();
}

// Is run when we update the WP or we enter AUTO mode from a different mode (ie LOITER)
void AR_WPNav_L1::start_new_turn(void)
{

    if (!_initial_turn_complete ) {
        gcs().send_text(MAV_SEVERITY_INFO, "Didnt make the turn - cancelled");
        _initial_turn_complete = true;
        auto_turn_centre.zero();
    } else {
        //start a new turn if we were on track
        _initial_turn_complete = false;
    }

}


bool AR_WPNav_L1::initial_turn_complete(void){
    return _initial_turn_complete;
}

/*
  Wrap AHRS yaw sensor if in reverse - centi-degress
 */
int32_t AR_WPNav_L1::get_yaw_sensor() const
{
    if (_reverse) {
        return wrap_180_cd(18000 + _ahrs.yaw_sensor);
    }
    return _ahrs.yaw_sensor;
}



float AR_WPNav_L1::nav_steering_angle(float groundspeed, float wheelbase, float _steering_angle_max, float _steering_angle_max_rate, float _steering_angle_max_accel, float turn_radius) 
{

    Location current_location;

    if(!_ahrs.get_location(current_location)){
        return 0;
    }
    
    if(_steering_angle_max< 0.5){
        _steering_angle_max = 0.5;
    }

    if(_steering_angle_max_rate<0.1){
        _steering_angle_max_rate =0.1;
    }

    if(_steering_angle_max_accel< (_steering_angle_max_rate*_steering_angle_max_rate/(2.0f*_steering_angle_max))){
        _steering_angle_max_accel = (_steering_angle_max_rate*_steering_angle_max_rate/(2.0f*_steering_angle_max)) + 0.1;
    }

    // calc steering angle for a given turn radius

    float turn_steering_angle = atanf(wheelbase/turn_radius);

    // turn radius resulting from half the steering angle required for the planned turn rate
    float thetaTR = wheelbase/tanf(turn_steering_angle*0.5f);

    // theta is the heading change between straight flight and full commanded bank angle
    float thetaTime = (_steering_angle_max/_steering_angle_max_rate) + (_steering_angle_max_rate/_steering_angle_max_accel);    
    float theta = thetaTime*groundspeed/thetaTR;
    float _curr_groundspeed_heading = atan2f(_ahrs.groundspeed_vector().y,_ahrs.groundspeed_vector().x);    // current ground speed heading of the aircraft


    float ground_turn_angle_remaining = wrap_2PI(auto_turn_exit_track-_curr_groundspeed_heading+ M_PI)- M_PI; // angle between current ground speed heading of aircraft and exit track direction
    //float loiter_turn_remaining = wrap_2PI(_loiter_exit_angle-_curr_groundspeed_heading+ M_PI)- M_PI;

    if (auto_turn_centre.lat == 0 && auto_turn_centre.lng == 0) {
        if (!_initial_turn_complete) {
            gcs().send_text(MAV_SEVERITY_INFO, "Ignoring turn");
        }
        _initial_turn_complete = true;
        auto_turn_centre.zero();
    }

    
    if( (ground_turn_angle_remaining*auto_turn_clockwise) / theta < _L1_Turn_Exit_Fraction && !_initial_turn_complete ) {
        _initial_turn_complete = true;
        auto_turn_centre.zero();
        _L1_xtrack_i = 0.0f;
        gcs().send_text(MAV_SEVERITY_INFO, "Ground frame turn complete ");
    }

    float ideal_turn_radius = groundspeed*groundspeed/_latAccDem;

    float ideal_steering_angle = atanf(wheelbase/ideal_turn_radius);

    //limit the steering angle to between the max and min steering angle
    return constrain_float(ideal_steering_angle, -_steering_angle_max, _steering_angle_max);

}

/*
  return the lateral acceleration needed to achieve tracking from the last
  update_*() operation
 */
float AR_WPNav_L1::lateral_acceleration(void) const
{
    return _latAccDem;
}

int32_t AR_WPNav_L1::nav_bearing_cd(void) const
{
    return wrap_180_cd(RadiansToCentiDegrees(_nav_bearing));
}

int32_t AR_WPNav_L1::bearing_error_cd(void) const
{
    return RadiansToCentiDegrees(_bearing_error);
}

float  AR_WPNav_L1::crosstrack_velo(void) const
{
    return _crosstrack_velo_portion * _ahrs.groundspeed_vector().length();
}

int32_t AR_WPNav_L1::target_bearing_cd(void) const
{
    return wrap_180_cd(_target_bearing_cd);
}

// Trigger a GRE exclusion period. We ignore GRE errors during this time as we know that the crosstrack error is incorrect
void AR_WPNav_L1::ground_risk_exclusion_event_trigger(void)
{
    ground_risk_exclusion_event_time = AP_HAL::millis();
    return;
}


// Return the distance away from the next waypoint where we should tick off that waypoint and commence the turn onto the new track.
Vector2f AR_WPNav_L1::turn_distance_ground_frame(const struct Location &previous_wp, const struct Location &current_loc, const struct Location &turn_WP, const struct Location &next_WP, float _trimspeed) //const
{
    Location potential_turn_centre;
    Vector3f potential_turn_vector;
    int8_t potential_turn_direction;
    float _groundspeed_heading_1;
    float _groundspeed_heading_2;
    Vector2f returnValue;

    Location::calc_orbit_turn_centre(previous_wp, current_loc, turn_WP, next_WP, waypoint_radius, _ground_turn_early_initiation,
                                     potential_turn_centre, potential_turn_vector, potential_turn_direction, _groundspeed_heading_1, _groundspeed_heading_2, returnValue);

    // If you are not in a turn. ie you are flying along "Current_Track"
    if (initial_turn_complete()){
        auto_turn_clockwise = potential_turn_direction; // direciton of auto_turn. Positive is clockwise
        auto_turn_centre.clone(potential_turn_centre);  // set auto_turn_centre location to the calculated potiential_turn_centre above
        auto_turn_exit_track = _groundspeed_heading_2;  // set auto_turn_exit_track to be the linear track between the WP to be completed and the WP following
    }
    // If you are in a turn. The exit track is the angle of the "Current Track" that you are aiming to turn onto.
    else{
        returnValue.normalize();                        // If in turn, make return value a return vector
        auto_turn_exit_track = _groundspeed_heading_1;  // set auto_turn_exit_track to be the linear track between the WP we just ticked off and are in the turn, and the WP following
    }


    return returnValue;

}

/**
   prevent indecision in our turning by using our previous turn
   decision if we are in a narrow angle band pointing away from the
   target and the turn angle has changed sign
 */
void AR_WPNav_L1::_prevent_indecision(float &Nu)
{
    const float Nu_limit = 0.9f*M_PI;
    if (fabsf(Nu) > Nu_limit &&
        fabsf(_last_Nu) > Nu_limit &&
        labs(wrap_180_cd(_target_bearing_cd - get_yaw_sensor())) > 12000 &&
        Nu * _last_Nu < 0.0f) {
        // we are moving away from the target waypoint and pointing
        // away from the waypoint (not flying backwards). The sign
        // of Nu has also changed, which means we are
        // oscillating in our decision about which way to go
        Nu = _last_Nu;
    }
}

void AR_WPNav_L1::update_waypoint(const struct Location &prev_WP, const struct Location &next_WP, float dist_min)
{
    Location _current_loc;
    Vector3f velocity;
     // Get current position and velocity
    bool canLoiterTurn = false;
    bool inTurnRadius = false;


    // variables for logging
    float anglePositive = 0;

    current_auto_waypoint.clone(next_WP);
    prev_auto_waypoint.clone(prev_WP);

    Vector2f centre_from_current = Vector2f(0,0);
    //Vector2f loiter_from_current = Vector2f(0,0);
    if(_ahrs.get_location(_current_loc) && _ahrs.get_velocity_NED(velocity)){

        // do the "Is it sensible to loiter around the turn point" checks
        if (auto_turn_centre.lat == 0 && auto_turn_centre.lng == 0) {
            if (!_initial_turn_complete) {
                gcs().send_text(MAV_SEVERITY_INFO, "Ignoring turn");
            }
            inTurnRadius = false;
            _initial_turn_complete = true;
            auto_turn_centre.zero();
        } else {
            inTurnRadius = _current_loc.get_distance(auto_turn_centre) < in_turn_error_scalar*prev_waypoint_radius;
        }

        centre_from_current = _current_loc.get_distance_NE(auto_turn_centre);

        anglePositive = (-centre_from_current.x*velocity.y) + (centre_from_current.y*velocity.x);
        // If the loiter point is on the correct side of the track for the auto turn direction and
        // If within an allowable distance from the centrepoint of the auto turn
        if(anglePositive * auto_turn_clockwise >0 && inTurnRadius){
            canLoiterTurn = true;
        }
        else if (!initial_turn_complete()){
            gcs().send_text(MAV_SEVERITY_INFO, "loiter turn failed");
        }
       
    }

   
    if (!initial_turn_complete() && canLoiterTurn){

        update_loiter(auto_turn_centre,prev_waypoint_radius-_ground_turn_correction_factor,auto_turn_clockwise);
        _current_nav_type = 1;
    }
    else{
        update_waypoint_straight(prev_WP,next_WP,dist_min);
        _current_nav_type = 0;
    }





}

bool AR_WPNav_L1::set_waypoint_speed(float speed)
{
    waypoint_speed = speed;
    return true;
}

bool AR_WPNav_L1::set_desired_location(const Location& destination, Location next_destination)
{

    _origin = _destination;
    _destination = destination;
    _orig_and_dest_valid = true;
    _reached_destination = false;
    Location _current_loc;
    if (_ahrs.get_location(_current_loc) == false) {
        return false;
    }


    if (current_auto_waypoint.is_zero()){
        prev_auto_waypoint.clone(_current_loc);
        prev_waypoint_radius = _ground_turn_radius;
        
    }
    else{
        prev_auto_waypoint.clone(current_auto_waypoint);
        if (prev_auto_waypoint.get_alt_frame() == Location::AltFrame::ABOVE_HOME)
        {
            prev_waypoint_radius = prev_auto_waypoint.alt/100;
        }
        else
        {
            prev_waypoint_radius = _ground_turn_radius;
        }

    }

    waypoint_radius = _ground_turn_radius;


    current_auto_waypoint.clone(_destination);
    next_auto_waypoint.clone(next_destination);

    if (current_auto_waypoint.get_alt_frame() == Location::AltFrame::ABOVE_HOME)
    {
        waypoint_radius = current_auto_waypoint.alt/100;
    }

    start_new_turn();
    return true;
}

// update L1 control for waypoint navigation
void AR_WPNav_L1::update_waypoint_straight(const struct Location &prev_WP, const struct Location &next_WP, float dist_min)
{

    struct Location _current_loc;
    float Nu;
    float xtrackVel;
    float ltrackVel;

    uint32_t now = AP_HAL::micros();
    float dt = (now - _last_update_waypoint_us) * 1.0e-6f;
    if (dt > 0.1) {
        dt = 0.1;
        _L1_xtrack_i = 0.0f;
        _groundRiskErrorFiltered = 0.0f;
    }
    _last_update_waypoint_us = now;

    // Calculate L1 gain required for specified damping
    float K_L1 = 4.0f * _L1_damping * _L1_damping;

    // Get current position and velocity
    if (_ahrs.get_location(_current_loc) == false) {
        // if no GPS loc available, maintain last nav/target_bearing
        _data_is_stale = true;
        return;
    }

    Vector2f _groundspeed_vector = _ahrs.groundspeed_vector();

    // update _target_bearing_cd
    _target_bearing_cd = _current_loc.get_bearing_to(next_WP);

    //Calculate groundspeed
    float groundSpeed = _groundspeed_vector.length();
    if (groundSpeed < 0.1f) {
        // use a small ground speed vector in the right direction,
        // allowing us to use the compass heading at zero GPS velocity
        groundSpeed = 0.1f;
        _groundspeed_vector = Vector2f(cosf(get_yaw()), sinf(get_yaw())) * groundSpeed;
    }

    // Calculate time varying control parameters
    // Calculate the L1 length required for specified period
    // 0.3183099 = 1/1/pipi
    if(!_initial_turn_complete){
        _L1_dist =  MAX(0.3183099f * _L1_damping * _L1_Auto_Period * groundSpeed, dist_min);
    }
    else{
        _L1_dist =  MAX(0.3183099f * _L1_damping * _L1_period * groundSpeed, dist_min);
    }

    // Calculate the NE position of WP B relative to WP A
    Vector2f AB = prev_WP.get_distance_NE(next_WP);
    float AB_length = AB.length();

    // Check for AB zero length and track directly to the destination
    // if too small
    if (AB.length() < 1.0e-6f) {
        AB = _current_loc.get_distance_NE(next_WP);
        if (AB.length() < 1.0e-6f) {
            AB = Vector2f(cosf(get_yaw()), sinf(get_yaw()));
        }
    }
    AB.normalize();

    // Calculate the NE position of the vehicle relative to WP A
    const Vector2f A_air = prev_WP.get_distance_NE(_current_loc);

    // calculate distance to target track, for reporting
    _crosstrack_error = A_air % AB;

    _stoppingDistance = (groundSpeed*groundSpeed)/(2*_emergency_stop_deceleration);

    _crosstrack_velo_portion = _groundspeed_vector.normalized() % AB;

    // Check for GRE
    _groundRiskError = _crosstrack_error + (_stoppingDistance * _crosstrack_velo_portion);

    // Force GRE to false if we know that the crosstrack error is incorrect for a known and accepted reason
    if (AP_HAL::millis() - ground_risk_exclusion_event_time < _ground_risk_exclusion_timeout *1000){
        _groundRiskError = 0;
    }

    _groundRiskErrorFiltered = ((dt/_ground_risk_filter_tc)*_groundRiskError) + (((_ground_risk_filter_tc-dt)/_ground_risk_filter_tc)*_groundRiskErrorFiltered);

    _previousCrosstrack_error = _crosstrack_error;

    //Determine if the aircraft is behind a +-135 degree degree arc centred on WP A
    //and further than L1 distance from WP A. Then use WP A as the L1 reference point
    //Otherwise do normal L1 guidance
    float WP_A_dist = A_air.length();
    float alongTrackDist = A_air * AB;

    if (WP_A_dist > _L1_dist && alongTrackDist/MAX(WP_A_dist, 1.0f) < -0.7071f)
    {
        //Calc Nu to fly To WP A
        Vector2f A_air_unit = (A_air).normalized(); // Unit vector from WP A to aircraft
        xtrackVel = _groundspeed_vector % (-A_air_unit); // Velocity across line
        ltrackVel = _groundspeed_vector * (-A_air_unit); // Velocity along line
        Nu = atan2f(xtrackVel,ltrackVel);
        _nav_bearing = atan2f(-A_air_unit.y , -A_air_unit.x); // bearing (radians) from AC to L1 point
    } else if (alongTrackDist > AB_length + groundSpeed*3) {
        // we have passed point B by 3 seconds. Head towards B
        // Calc Nu to fly To WP B
        const Vector2f B_air = next_WP.get_distance_NE(_current_loc);
        Vector2f B_air_unit = (B_air).normalized(); // Unit vector from WP B to aircraft
        xtrackVel = _groundspeed_vector % (-B_air_unit); // Velocity across line
        ltrackVel = _groundspeed_vector * (-B_air_unit); // Velocity along line
        Nu = atan2f(xtrackVel,ltrackVel);
        _nav_bearing = atan2f(-B_air_unit.y , -B_air_unit.x); // bearing (radians) from AC to L1 point
    } else { //Calc Nu to fly along AB line

        //Calculate Nu2 angle (angle of velocity vector relative to line connecting waypoints)
        xtrackVel = _groundspeed_vector % AB; // Velocity cross track
        ltrackVel = _groundspeed_vector * AB; // Velocity along track
        float Nu2 = atan2f(xtrackVel,ltrackVel);
        //Calculate Nu1 angle (Angle to L1 reference point)
        float sine_Nu1 = _crosstrack_error/MAX(_L1_dist, 0.1f);
        //Limit sine of Nu1 to provide a controlled track capture angle of 45 deg
        sine_Nu1 = constrain_float(sine_Nu1, -0.7071f, 0.7071f);
        float Nu1 = asinf(sine_Nu1);

        // compute integral error component to converge to a crosstrack of zero when traveling
        // straight but reset it when disabled or if it changes. That allows for much easier
        // tuning by having it re-converge each time it changes.
        if (_L1_xtrack_i_gain <= 0 || !is_equal(_L1_xtrack_i_gain.get(), _L1_xtrack_i_gain_prev)) {
            _L1_xtrack_i = 0;
            _L1_xtrack_i_gain_prev = _L1_xtrack_i_gain;
        } else if (fabsf(Nu1) < radians(5)) {
            _L1_xtrack_i += Nu1 * _L1_xtrack_i_gain * dt;

            // an AHRS_TRIM_X=0.1 will drift to about 0.08 so 0.1 is a good worst-case to clip at
            _L1_xtrack_i = constrain_float(_L1_xtrack_i, -0.1f, 0.1f);
        }
        else{ // reset integrator if L1 angle is greater than 20 degrees
            _L1_xtrack_i = 0;
        }

        // to converge to zero we must push Nu1 harder
        Nu1 += _L1_xtrack_i;

        Nu = Nu1 + Nu2;
        _nav_bearing = atan2f(AB.y, AB.x) + Nu1; // bearing (radians) from AC to L1 point
    }

    _prevent_indecision(Nu);
    _last_Nu = Nu;

    //Limit Nu to +-(pi/2)
    Nu = constrain_float(Nu, -1.5708f, +1.5708f);
    _latAccDem = K_L1 * groundSpeed * groundSpeed / _L1_dist * sinf(Nu);

    // Waypoint capture status is always false during waypoint following
    _WPcircle = false;

    _bearing_error = Nu; // bearing error angle (radians), +ve to left of track

    _data_is_stale = false; // status are correctly updated with current waypoint data
}

// update L1 control for loitering
void AR_WPNav_L1::update_loiter(const struct Location &center_WP, float radius, int8_t loiter_direction)
{


    uint32_t now = AP_HAL::micros();
    float dt = (now - _last_update_waypoint_us) * 1.0e-6f;
    if (dt > 0.1) {
        dt = 0.1;
        _L1_xtrack_i = 0.0f;
        _groundRiskErrorFiltered = 0.0f;
    }
    _last_update_waypoint_us = now;

    struct Location _current_loc;

    // Calculate guidance gains used by PD loop (used during circle tracking)
    float omega = (6.2832f / _L1_period);
    float Kx = omega * omega;
    float Kv = 2.0f * _L1_damping * omega;

    // Calculate L1 gain required for specified damping (used during waypoint capture)
    float K_L1 = 4.0f * _L1_damping * _L1_damping;

    //Get current position and velocity
    if (_ahrs.get_location(_current_loc) == false) {
        // if no GPS loc available, maintain last nav/target_bearing
        _data_is_stale = true;
        return;
    }

    Vector2f _groundspeed_vector = _ahrs.groundspeed_vector();

    //Calculate groundspeed
    float groundSpeed = MAX(_groundspeed_vector.length() , 1.0f);


    // update _target_bearing_cd
    _target_bearing_cd = _current_loc.get_bearing_to(center_WP);


    // Calculate time varying control parameters
    // Calculate the L1 length required for specified period
    // 0.3183099 = 1/pi
    _L1_dist = 0.3183099f * _L1_damping * _L1_period * groundSpeed;

    //Calculate the NE position of the aircraft relative to WP A
    const Vector2f A_air = center_WP.get_distance_NE(_current_loc);

    // Calculate the unit vector from WP A to aircraft
    // protect against being on the waypoint and having zero velocity
    // if too close to the waypoint, use the velocity vector
    // if the velocity vector is too small, use the heading vector
    Vector2f A_air_unit;
    if (A_air.length() > 0.1f) {
        A_air_unit = A_air.normalized();
    } else {
        if (_groundspeed_vector.length() < 0.1f) {
            
            A_air_unit = Vector2f(cosf(_ahrs.get_yaw() ), sinf(_ahrs.get_yaw()));
        } else {
            A_air_unit = _groundspeed_vector.normalized();
        }
    }

    //Calculate Nu to capture center_WP
    float xtrackVelCap = A_air_unit % _groundspeed_vector; // Velocity across line - perpendicular to radial inbound to WP
    float ltrackVelCap = - (_groundspeed_vector * A_air_unit); // Velocity along line - radial inbound to WP
    float Nu = atan2f(xtrackVelCap,ltrackVelCap);

    _prevent_indecision(Nu);
    _last_Nu = Nu;

    Nu = constrain_float(Nu, -M_PI_2, M_PI_2); //Limit Nu to +- Pi/2

    //Calculate lat accln demand to capture center_WP (use L1 guidance law)
    float latAccDemCap = K_L1 * groundSpeed * groundSpeed / _L1_dist * sinf(Nu);

    //Calculate radial position and velocity errors
    float xtrackVelCirc = -ltrackVelCap; // Radial outbound velocity - reuse previous radial inbound velocity
    float xtrackErrCirc = A_air.length() - radius; // Radial distance from the loiter circle

    // keep crosstrack error for reporting
    _crosstrack_error = loiter_direction*xtrackErrCirc;

    _stoppingDistance = (groundSpeed*groundSpeed)/(2*_emergency_stop_deceleration);

    _crosstrack_velo_portion = loiter_direction*xtrackVelCirc/groundSpeed;

    // Check for GRE
    _groundRiskError = _crosstrack_error + (_stoppingDistance * _crosstrack_velo_portion);

    // Force GRE to false if we know that the crosstrack error is incorrect for a known and accepted reason
   if (AP_HAL::millis() - ground_risk_exclusion_event_time < _ground_risk_exclusion_timeout *1000){
        _groundRiskError = 0;
    }


    _groundRiskErrorFiltered = ((dt/_ground_risk_filter_tc)*_groundRiskError) + (((_ground_risk_filter_tc-dt)/_ground_risk_filter_tc)*_groundRiskErrorFiltered);

    _previousCrosstrack_error = _crosstrack_error;
    //Calculate PD control correction to circle waypoint_ahrs.roll
    float latAccDemCircPD = (xtrackErrCirc * Kx + xtrackVelCirc * Kv);

    //Calculate tangential velocity
    float velTangent = xtrackVelCap * float(loiter_direction);

    //Prevent PD demand from turning the wrong way by limiting the command when flying the wrong way
    if (ltrackVelCap < 0.0f && velTangent < 0.0f) {
        latAccDemCircPD =  MAX(latAccDemCircPD, 0.0f);
    }

    // Calculate centripetal acceleration demand
    float latAccDemCircCtr = velTangent * velTangent / MAX((0.5f * radius), (radius + xtrackErrCirc));

    //Sum PD control and centripetal acceleration to calculate lateral manoeuvre demand
    float latAccDemCirc = loiter_direction * (latAccDemCircPD + latAccDemCircCtr);

    // Perform switchover between 'capture' and 'circle' modes at the
    // point where the commands cross over to achieve a seamless transfer
    // Only fly 'capture' mode if outside the circle
    if (xtrackErrCirc > 0.0f && loiter_direction * latAccDemCap < loiter_direction * latAccDemCirc) {
        _latAccDem = latAccDemCap;
        _WPcircle = false;
        _bearing_error = Nu; // angle between demanded and achieved velocity vector, +ve to left of track
        _nav_bearing = atan2f(-A_air_unit.y , -A_air_unit.x); // bearing (radians) from AC to L1 point
    } else {
        _latAccDem = latAccDemCirc;
        _WPcircle = true;
        _bearing_error = 0.0f; // bearing error (radians), +ve to left of track
        _nav_bearing = atan2f(-A_air_unit.y , -A_air_unit.x); // bearing (radians)from AC to L1 point
    }

    _data_is_stale = false; // status are correctly updated with current waypoint data
}


// update L1 control for heading hold navigation
void AR_WPNav_L1::update_heading_hold(int32_t navigation_heading_cd)
{
    // Calculate normalised frequency for tracking loop
    const float omegaA = 4.4428f/_L1_period; // sqrt(2)*pi/period
    // Calculate additional damping gain

    int32_t Nu_cd;
    float Nu;

    // copy to _target_bearing_cd and _nav_bearing
    _target_bearing_cd = wrap_180_cd(navigation_heading_cd);
    _nav_bearing = radians(navigation_heading_cd * 0.01f);

    Nu_cd = _target_bearing_cd - wrap_180_cd(_ahrs.yaw_sensor);
    Nu_cd = wrap_180_cd(Nu_cd);
    Nu = radians(Nu_cd * 0.01f);

    Vector2f _groundspeed_vector = _ahrs.groundspeed_vector();

    //Calculate groundspeed
    float groundSpeed = _groundspeed_vector.length();

    // Calculate time varying control parameters
    _L1_dist = groundSpeed / omegaA; // L1 distance is adjusted to maintain a constant tracking loop frequency
    float VomegaA = groundSpeed * omegaA;

    // Waypoint capture status is always false during heading hold
    _WPcircle = false;

    _crosstrack_error = 0;

    _bearing_error = Nu; // bearing error angle (radians), +ve to left of track

    // Limit Nu to +-pi
    Nu = constrain_float(Nu, -M_PI_2, M_PI_2);
    _latAccDem = 2.0f*sinf(Nu)*VomegaA;

    _data_is_stale = false; // status are correctly updated with current waypoint data
}
