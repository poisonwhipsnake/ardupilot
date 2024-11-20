#include <AP_HAL/AP_HAL.h>
#include "AP_L1_Control.h"

extern const AP_HAL::HAL& hal;

// table of user settable parameters
const AP_Param::GroupInfo AP_L1_Control::var_info[] = {
    // @Param: PERIOD
    // @DisplayName: L1 control period
    // @Description: Period in seconds of L1 tracking loop. This parameter is the primary control for agressiveness of turns in auto mode. This needs to be larger for less responsive airframes. The default of 20 is quite conservative, but for most RC aircraft will lead to reasonable flight. For smaller more agile aircraft a value closer to 15 is appropriate, or even as low as 10 for some very agile aircraft. When tuning, change this value in small increments, as a value that is much too small (say 5 or 10 below the right value) can lead to very radical turns, and a risk of stalling.
    // @Units: s
    // @Range: 1 60
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("PERIOD",    0, AP_L1_Control, _L1_period, 20),

    // @Param: DAMPING
    // @DisplayName: L1 control damping ratio
    // @Description: Damping ratio for L1 control. Increase this in increments of 0.05 if you are getting overshoot in path tracking. You should not need a value below 0.7 or above 0.85.
    // @Range: 0.6 1.0
    // @Increment: 0.05
    // @User: Advanced
    AP_GROUPINFO("DAMPING",   1, AP_L1_Control, _L1_damping, 0.75f),

    // @Param: XTRACK_I
    // @DisplayName: L1 control crosstrack integrator gain
    // @Description: Crosstrack error integrator gain. This gain is applied to the crosstrack error to ensure it converges to zero. Set to zero to disable. Smaller values converge slower, higher values will cause crosstrack error oscillation.
    // @Range: 0 0.1
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("XTRACK_I",   2, AP_L1_Control, _L1_xtrack_i_gain, 0.1),

    // @Param: LIM_BANK
    // @DisplayName: Loiter Radius Bank Angle Limit
    // @Description: The sealevel bank angle limit for a continous loiter. (Used to calculate airframe loading limits at higher altitudes). Setting to 0, will instead just scale the loiter radius directly
    // @Units: deg
    // @Range: 0 89
    // @User: Advanced
    AP_GROUPINFO_FRAME("LIM_BANK",   3, AP_L1_Control, _loiter_bank_limit, 0.0f, AP_PARAM_FRAME_PLANE),

       // @Param: AUTO_LIM_BANK
    // @DisplayName:  Bank Angle for calculating turn in point in auto nav
    // @Description: Blank
    // @Units: deg
    // @Range: 0 89
    // @User: Advanced
    AP_GROUPINFO("AUTO_BANK",   4, AP_L1_Control, _auto_bank_limit, 40.0f),


    // @Param: Turn Rate correction factor
	// @DisplayName: Maximum Roll Acceleration
	// @Description: Maximum Acceleration of Roll rate for nav_rll (deg/s/s).
	// @Range: 0.1 2
	// @Increment: 1
	// @User: User
	AP_GROUPINFO("TRCF", 8, AP_L1_Control, _turn_rate_correction_factor,1.0f),

    // @Param: Auto_L1_Period
    // @DisplayName:  integral gain to account for wind speed measurement error
    // @Description: Blank
    // @Units: 1/s
    // @Range: 0 89
    // @User: Advanced
    AP_GROUPINFO("A_PERIOD", 9, AP_L1_Control, _L1_Auto_Period, 40.0f),

    AP_GROUPINFO("TEXIT", 10, AP_L1_Control, _L1_Turn_Exit_Fraction, 1.0f),

    AP_GROUPINFO("TGAIN", 11, AP_L1_Control, _L1_Mid_Turn_Gain, 1.0f),

    AP_GROUPINFO("TDELAY", 12, AP_L1_Control, _L1_Turn_Delay,  0.0f),

    AP_GROUPINFO("FRAME",13, AP_L1_Control, _Turn_Frame_Type, 1),

    AP_GROUPINFO("GTR",14, AP_L1_Control, _ground_turn_radius, 200.0),

    AP_GROUPINFO("GTCF",15, AP_L1_Control, _ground_turn_correction_factor, 20.0),

    AP_GROUPINFO("GTEI",16, AP_L1_Control, _ground_turn_early_initiation, 50.0),

    AP_GROUPINFO("L_DIR",17, AP_L1_Control, _loiter_side, 1 ),

    AP_GROUPINFO("L_RAD", 18, AP_L1_Control, _loiter_radius, 200.0 ),

    AP_GROUPINFO("T_ERROR",19, AP_L1_Control, _max_auto_point_distance, 1000.0f),

    AP_GROUPINFO("ALT_LOIT",20, AP_L1_Control, _use_loiter_vector_alt, 1),

    AP_GROUPINFO("E_DECEL",21, AP_L1_Control, _emergency_land_deceleration, 3.1f),

    AP_GROUPINFO("GRE_F_TC", 22, AP_L1_Control, _ground_risk_filter_tc, 5.0f),

    AP_GROUPINFO("TA_DONE", 23, AP_L1_Control, _turn_around_reintercept_tollerance, 10.0f),

    AP_GROUPINFO("MAX_XT", 24, AP_L1_Control, _max_XT, 500.0f),


    AP_GROUPEND
};

//Bank angle command based on angle between aircraft velocity vector and reference vector to path.
//S. Park, J. Deyst, and J. P. How, "A New Nonlinear Guidance Logic for Trajectory Tracking,"
//Proceedings of the AIAA Guidance, Navigation and Control
//Conference, Aug 2004. AIAA-2004-4900.
//Modified to use PD control for circle tracking to enable loiter radius less than L1 length
//Modified to enable period and damping of guidance loop to be set explicitly
//Modified to provide explicit control over capture angle


/*
  Wrap AHRS yaw if in reverse - radians
 */

void AP_L1_Control::reset(){
    _initial_turn_complete = true;
    _use_loiter_for_turn = false;
    auto_turn_vector = Vector3f(0,0,1.0f);
    loiter_vector = Vector3f(0,0,1.0f);
    _turn_around_gre_exception = false;


}

float AP_L1_Control::get_yaw()
{
    if (_reverse) {
        return wrap_PI(M_PI + _ahrs.yaw);
    }
    return _ahrs.yaw;
}

void AP_L1_Control::start_new_turn(void){

    if(!_initial_turn_complete && !_use_loiter_for_turn){
        gcs().send_text(MAV_SEVERITY_INFO, "Didnt make the turn - cancelled");
        _initial_turn_complete = true;
    }
    else{
    //start a new turn if we were on track
        _initial_turn_complete = false;
    }
}

void AP_L1_Control::setup_loiter_to_track(void){
     _use_loiter_for_turn = true;
    _turn_around_gre_exception = true;
    _filteredApproachSpeed = -10;

}

void AP_L1_Control::setup_loiter_to_new_track(Vector2f newTrack){
    _use_loiter_for_turn = true;
    _loiter_exit_track = wrap_2PI(atan2f(newTrack.y,newTrack.x)+M_PI_4*_loiter_side);
    if(_loiter_exit_track >M_PI){
        _loiter_exit_track -= M_2PI;
    }
    _initial_turn_complete = true;
    _turn_around_gre_exception = true;
    _filteredApproachSpeed = -10;

}

bool AP_L1_Control::initial_turn_complete(void){
    return _initial_turn_complete;
}

/*
  Wrap AHRS yaw sensor if in reverse - centi-degress
 */
int32_t AP_L1_Control::get_yaw_sensor() const
{
    if (_reverse) {
        return wrap_180_cd(18000 + _ahrs.yaw_sensor);
    }
    return _ahrs.yaw_sensor;
}

/*
  return the bank angle needed to achieve tracking from the last
  update_*() operation
 */
int32_t AP_L1_Control::nav_roll_cd() const
{
    float ret;
    ret = cosf(_ahrs.pitch)*degrees(atanf(_latAccDem * 0.101972f) * 100.0f); // 0.101972 = 1/9.81
    ret = constrain_float(ret, -9000, 9000);
    return ret;
}

/*
    return the bank angle needed to achieve tracking from the last update_*() operation
    Considers
        - bank limit
        - roll rate limit
        - roll accel limit

    A turn is divided into three components:
        - Transition from 0 roll angle to commanded turn roll angle
        - Hold commanded turn roll angle
        - Transition from commanded turn roll angle back to 0 roll angle

    The first and last stages are symmetrical and therefore cover the same amount of distance in the air frame
 */
int32_t AP_L1_Control::nav_roll_cd_special(float _amax, float _rmax, float _trimspeed, float _minspeed, Location current_location)
{

    float bank_limit = DEG_TO_RAD*_auto_bank_limit;
    float roll_rate = DEG_TO_RAD*_rmax;
    float roll_accel = DEG_TO_RAD*_amax;
    float airspeed = 0;
    const bool gotAirspeed = _ahrs.airspeed_estimate_true(&airspeed);

    if(!gotAirspeed || airspeed <  _minspeed){
        airspeed = _trimspeed; //need to make it a reference to trim speed
        if (airspeed<1.0f){
            airspeed =1;
        }
    }

    if(bank_limit< 0.5){
        bank_limit = 0.5;
    }

    if(roll_rate<0.1){
        roll_rate =0.1;
    }


    if(roll_accel< (roll_rate*roll_rate/(2.0f*bank_limit))){
        roll_accel = (roll_rate*roll_rate/(2.0f*bank_limit)) + 0.1;
    }

    // theta is the heading change between straight flight and full commanded bank angle
    float theta = (bank_limit*0.5f)*((bank_limit/roll_rate)+(roll_rate/roll_accel))*((6*GRAVITY_MSS)/(5*airspeed));
    Vector2f target_ground_velocity =  Vector2f(cosf(_nav_bearing),sinf(_nav_bearing));
    Vector2f _windspeed_vector = Vector2f(_ahrs.wind_estimate().x, _ahrs.wind_estimate().y);
    Vector2f _check_windspeed =  _ahrs.groundspeed_vector() - (Vector2f(cosf( _ahrs.get_yaw()),sinf(_ahrs.get_yaw()))*airspeed);

     //check if the wind estimate makes any sense
    if (_windspeed_vector.angle(_check_windspeed) > M_PI_4 || _windspeed_vector.length() <_check_windspeed.length() *0.5 ||_windspeed_vector.length()>_check_windspeed.length()*2.0){
        _windspeed_vector =  _check_windspeed; //this is more robust, but less accurate
    }

    Vector2f target_air_velocity = get_airspeed_from_wind_ground(_windspeed_vector,target_ground_velocity,airspeed);


    float _groundspeed_heading_1 = atan2f(_ahrs.groundspeed_vector().y,_ahrs.groundspeed_vector().x);
    float _ground_turn_angle = wrap_2PI(_nav_bearing-_groundspeed_heading_1+ M_PI)- M_PI;
    float _airspeed_heading_2 = atan2f(target_air_velocity.y,target_air_velocity.x);
    float _air_turn_angle = wrap_2PI(_airspeed_heading_2-_ahrs.get_yaw()+ M_PI)- M_PI;

    // if the best angle to turn in the air frame is different to the ground frame,
    // force the air turn to be in the same direction as the ground turn

    if( _air_turn_angle*_ground_turn_angle<0 && abs(_ground_turn_angle)>M_PI_2 && abs(_air_turn_angle)>0.001 ){
        _air_turn_angle += -(abs(_air_turn_angle)/_air_turn_angle)*(M_PI*2);
    }

    float ground_turn_angle_remaining = wrap_2PI(auto_turn_exit_track-_groundspeed_heading_1+ M_PI)- M_PI;
    float loiter_turn_remaining = wrap_2PI(_loiter_exit_track-_groundspeed_heading_1+ M_PI)- M_PI;


    // If initial acceleration portion of the turn is complete
    if(_Turn_Frame_Type == 1 && _use_loiter_for_turn){
        if(abs(loiter_turn_remaining)/theta<_L1_Turn_Exit_Fraction || (current_location.get_distance(desired_loiter_point)> 2*_ground_turn_radius) ){
            gcs().send_text(MAV_SEVERITY_INFO, "Loiter Navigation Exit Complete");
            _L1_xtrack_i = 0.0f;
            _use_loiter_for_turn = false;
        }
    }
    else if(_Turn_Frame_Type == 0  || (current_location.get_distance(auto_turn_centre)> 2*_ground_turn_radius)){
        if( abs(_air_turn_angle/theta)<_L1_Turn_Exit_Fraction  && !_initial_turn_complete ){
            _initial_turn_complete = true;
            gcs().send_text(MAV_SEVERITY_INFO, "Initial turn complete ");
        }
    }
    else {
        if( (ground_turn_angle_remaining*auto_turn_clockwise)/theta<_L1_Turn_Exit_Fraction && !_initial_turn_complete ){
            _initial_turn_complete = true;
            _L1_xtrack_i = 0.0f;
            gcs().send_text(MAV_SEVERITY_INFO, "Ground frame turn complete ");
        }
    }

    float bank_angle;

    if(_initial_turn_complete || _Turn_Frame_Type == 1){

        bank_angle = cosf(_ahrs.pitch)*degrees(atanf(_latAccDem * 0.101972f)); // 0.101972 = 1/9.81
        bank_angle = constrain_float(bank_angle, -_auto_bank_limit, _auto_bank_limit)* 100.0f;

    }
    else{

        bank_angle = (constrain_float(_air_turn_angle*_L1_Mid_Turn_Gain/theta,-1.0f,1.0f)* _auto_bank_limit)*100.0f;

    }


    return (int32_t)bank_angle;
}


/*
  return the lateral acceleration needed to achieve tracking from the last
  update_*() operation
 */
float AP_L1_Control::lateral_acceleration(void) const
{
    return _latAccDem;
}

int32_t AP_L1_Control::nav_bearing_cd(void) const
{
    return wrap_180_cd(RadiansToCentiDegrees(_nav_bearing));
}

int32_t AP_L1_Control::bearing_error_cd(void) const
{
    return RadiansToCentiDegrees(_bearing_error);
}

float  AP_L1_Control::crosstrack_velo(void) const
{
    return _crosstrack_velo_portion * _ahrs.groundspeed_vector().length();
}

int32_t AP_L1_Control::target_bearing_cd(void) const
{
    return wrap_180_cd(_target_bearing_cd);
}

/*
  this is the turn distance assuming a 90 degree turn
 */
float AP_L1_Control::turn_distance(float wp_radius) const
{
    wp_radius *= sq(_ahrs.get_EAS2TAS());
    return MIN(wp_radius, _L1_dist);
}

/*
  this approximates the turn distance for a given turn angle. If the
  turn_angle is > 90 then a 90 degree turn distance is used, otherwise
  the turn distance is reduced linearly.
  This function allows straight ahead mission legs to avoid thinking
  they have reached the waypoint early, which makes things like camera
  trigger and ball drop at exact positions under mission control much easier
 */
float AP_L1_Control::turn_distance(float groundspeed, float turn_angle) const
{
    if (turn_angle < 0.1){
        return 1.0f; // avoid tan(90)
    }
    float turn_radius = groundspeed*groundspeed/(10*tanf(radians(_auto_bank_limit)));
    float turn_distance = turn_radius/tanf(radians((180.0f-abs(turn_angle))*0.5f));
    return turn_distance;
}

/*
  return the turn distance for a given turn angle
    Considers
        - bank limit
        - roll rate limit
        - roll accel limit
        - ground velocity frame
        - air velocity frame
*/
Vector2f AP_L1_Control::turn_distance_air_frame( const struct Location &current_loc, const struct Location &turn_WP, const struct Location &next_WP, const float roll_rate_deg, const float roll_accel_deg, float _trimspeed, float _minspeed, float _current_roll) //const
{


    float bank_limit = DEG_TO_RAD*_auto_bank_limit;
    float roll_rate = DEG_TO_RAD*roll_rate_deg;
    float roll_accel = DEG_TO_RAD*roll_accel_deg;

    float airspeed = 1.0f; // should set to trim airspeed
    const bool gotAirspeed = _ahrs.airspeed_estimate_true(&airspeed);

    if(_ahrs.groundspeed_vector().length() < 0.5f){
        return Vector2f(0.1f,0.1f);
    }

    if(!gotAirspeed || airspeed <  _minspeed){
       return _ahrs.groundspeed_vector().normalized()*10.0f;
    }

    if(_ahrs.groundspeed() <2.0f|| turn_WP.get_distance_NE(next_WP).length()<10.0f ){
        return _ahrs.groundspeed_vector().normalized()* airspeed *2.0f ;
    }

    //remove any change of divide by zero

    if(bank_limit< 0.5){
        bank_limit = 0.5;
    }

    if(roll_rate<0.1){
        roll_rate =0.1;
    }

    if(roll_accel< (roll_rate*roll_rate/(2.0f*bank_limit))){
        roll_accel = (roll_rate*roll_rate/(2.0f*bank_limit)) + 0.1;
    }


    float theta = _turn_rate_correction_factor*(bank_limit*0.5f)*((bank_limit/roll_rate)+(roll_rate/roll_accel))*((6*GRAVITY_MSS)/(5*airspeed));
    Vector2f _groundspeed_vector_1 = _ahrs.groundspeed_vector();
    Vector2f _windspeed_vector = Vector2f(_ahrs.wind_estimate().x, _ahrs.wind_estimate().y);
    Vector2f _check_windspeed =  _ahrs.groundspeed_vector() - (Vector2f(cosf( _ahrs.get_yaw()),sinf(_ahrs.get_yaw()))*airspeed);

    //check if the wind estimate makes any sense
    if (_windspeed_vector.angle(_check_windspeed) > M_PI_4 || _windspeed_vector.length() <_check_windspeed.length() *0.5 ||_windspeed_vector.length()>_check_windspeed.length()*2.0){
        _windspeed_vector =  _check_windspeed; //this is more robust, but less accurate
    }

    Vector2f _airspeed_vector_1 = _groundspeed_vector_1 - _windspeed_vector;
    Vector2f _groundspeed_vector_2 = turn_WP.get_distance_NE(next_WP);
    Vector2f _airspeed_vector_2 = get_airspeed_from_wind_ground(_windspeed_vector, _groundspeed_vector_2, airspeed);


    //catches case where we have to turn more than 180 degrees in air frame
    float _groundspeed_heading_1 = atan2f(_groundspeed_vector_1.y,_groundspeed_vector_1.x);
    float _groundspeed_heading_2 = atan2f(_groundspeed_vector_2.y,_groundspeed_vector_2.x);
    float _ground_turn_angle = wrap_2PI(_groundspeed_heading_2-_groundspeed_heading_1+ M_PI)- M_PI;
    float _airspeed_heading_1 = atan2f(_airspeed_vector_1.y,_airspeed_vector_1.x);
    float _airspeed_heading_2 = atan2f(_airspeed_vector_2.y,_airspeed_vector_2.x);
    float _air_turn_angle = wrap_2PI(_airspeed_heading_2-_airspeed_heading_1+ M_PI)- M_PI;

    //fix the divide by zero nonesense (your turn is near strait anyway)
    if(abs(_air_turn_angle)<0.02 || abs(_ground_turn_angle)<0.02){
       return _ahrs.groundspeed_vector() *2.0f;
    }

    //reduce the distance if you are already rolled the correct direction
    Vector2f turn_distance_extra = Vector2f(0.0f,0.0f);
    if(abs(_current_roll)>0.01){
        if (_current_roll *_air_turn_angle<0.0f){
            float turn_rate = 0.5*_turn_rate_correction_factor*bank_limit*((6*GRAVITY_MSS)/(5*airspeed));
            turn_distance_extra =_ahrs.groundspeed_vector().normalized()*(abs(_current_roll)/bank_limit)*(theta/turn_rate) *_ahrs.groundspeed();
        }
        else{
            theta = (1 - (0.5*abs(_current_roll)/bank_limit))*theta;
        }
    }

    // If there is no constant roll portion of the turn
    if(abs(_air_turn_angle) <  2*theta ){

        float turnRadius= sq(_ahrs.groundspeed_vector().length())/(GRAVITY_MSS*tanf(bank_limit*0.5f));
        float turnDistanceScalar = 2*turnRadius/tanf((M_PI-abs(_air_turn_angle))*0.5f);
        if(turnDistanceScalar < _ahrs.groundspeed()*2.0){
            turnDistanceScalar = _ahrs.groundspeed()*2.0f;
        }
        if(current_loc.get_distance_NE(turn_WP).length()<1.0f){
            return _groundspeed_vector_1;
        }
        Vector2f turnDistanceReturn  = current_loc.get_distance_NE(turn_WP).normalized()*(turnDistanceScalar +turn_distance_extra.length() + (_ahrs.groundspeed()*_L1_Turn_Delay)  );

        return turnDistanceReturn;
    }

    // if the best angle to turn in the air frame is different to the ground frame,
    // force the air turn to be in the same direction as the ground turn
    if(_air_turn_angle*_ground_turn_angle<0 ){
        _air_turn_angle += -(abs(_air_turn_angle)/_air_turn_angle)*(M_PI*2);
    }

    Vector2f _airspeed_vector_1_normalized = _airspeed_vector_1.normalized();
    Vector2f perp_airspeed_vector_1 = Vector2f(-_airspeed_vector_1_normalized.y, _airspeed_vector_1_normalized.x); // perpendicular airspeed vector
    // set direction of perpendicular airspeed vector based on turn direction
    if(_air_turn_angle<0.0f){
        perp_airspeed_vector_1 = -perp_airspeed_vector_1;
    }

    float beta = abs(_air_turn_angle);
    Vector2f turnDistanceXY = ( (perp_airspeed_vector_1* (2-cosf(theta) + cosf(beta-theta) -(2*cosf(beta)))) + (_airspeed_vector_1_normalized*(sinf(theta)-sinf(beta-theta)+(2*sinf(beta)))));
    turnDistanceXY = turnDistanceXY*((5*sq(airspeed))/(6*GRAVITY_MSS*bank_limit*_turn_rate_correction_factor));

    float turnTimealphaPortion = ((5*airspeed)/(6*GRAVITY_MSS*bank_limit*_turn_rate_correction_factor))*(abs(_air_turn_angle)-(2*theta));
    float turnTimeThetaPortion = (2*((bank_limit/roll_rate)+(roll_rate/roll_accel)));
    float turnTime = turnTimealphaPortion + turnTimeThetaPortion;

    turnDistanceXY = turnDistanceXY + (_windspeed_vector*turnTime)+turn_distance_extra + (_ahrs.groundspeed_vector()*_L1_Turn_Delay) ;

    return turnDistanceXY;

}


Vector2f AP_L1_Control::turn_distance_ground_frame(const struct Location &previous_wp, const struct Location &current_loc, const struct Location &turn_WP, const struct Location &next_WP, float _trimspeed) //const
{
    Vector2f Current_Track_Full = previous_wp.get_distance_NE(turn_WP);
    Vector2f Next_Track_Full = turn_WP.get_distance_NE(next_WP);
    Vector2f Current_Track = Current_Track_Full.normalized();
    Vector2f Next_Track = Next_Track_Full.normalized();
    Vector3f Current_Vector = Vector3f(Current_Track_Full.x,Current_Track_Full.y,(turn_WP.alt-previous_wp.alt)/100.0f); ////I think this assumes the WP have the same height reference frame
    Vector3f Next_Vector = Vector3f(Next_Track_Full.x,Next_Track_Full.y,(next_WP.alt - turn_WP.alt)/100.0f); ////I think this assumes the WP have the same height reference frame

    //float _ground_turn_angle = Current_Track.angle(Next_Track);


    float _groundspeed_heading_1 = atan2f(Current_Track.y,Current_Track.x);
    float _groundspeed_heading_2 = atan2f(Next_Track.y,Next_Track.x);
    float _ground_turn_angle = wrap_2PI(_groundspeed_heading_2-_groundspeed_heading_1+ M_PI)- M_PI;

    float turn_distance = tanf(abs(_ground_turn_angle/2))*_ground_turn_radius;

    float current_angle_to_track = Current_Track.angle(current_loc.get_distance_NE(turn_WP));

    Vector2f returnValue = (current_loc.get_distance_NE(turn_WP).normalized()/cosf(current_angle_to_track))*(turn_distance+_ground_turn_early_initiation);

    int8_t potential_turn_direction = -1;
    Location potential_turn_centre;
    Vector2f centre_from_turn_WP = -Current_Track*turn_distance + Vector2f(Current_Track.y,-Current_Track.x)*_ground_turn_radius;

    Vector3f potential_turn_vector = (Current_Vector%Next_Vector).normalized();
    if(potential_turn_vector.z <0.0f){
        potential_turn_vector = -potential_turn_vector;
    }
    if(_ground_turn_angle>0){
        centre_from_turn_WP = -Current_Track*turn_distance + Vector2f(-Current_Track.y,Current_Track.x)*_ground_turn_radius;
        potential_turn_direction = 1;
    }


    Vector2f WPFromCentreNormalised = -centre_from_turn_WP.normalized();

    float desiredAngle = asinf( Vector2f(potential_turn_vector.x,potential_turn_vector.y)*WPFromCentreNormalised);
    float heightOffset = tanf(desiredAngle)*centre_from_turn_WP.length();


    potential_turn_centre.clone(turn_WP);
    potential_turn_centre.offset(centre_from_turn_WP.x,centre_from_turn_WP.y);
    potential_turn_centre.set_alt_cm(turn_WP.alt + (heightOffset*100),turn_WP.get_alt_frame());


    if (initial_turn_complete()){//returnValue.length() + _trimspeed >current_loc.get_distance(turn_WP)){
        auto_turn_clockwise = potential_turn_direction;
        auto_turn_centre.clone(potential_turn_centre);
        auto_turn_exit_track = _groundspeed_heading_2;
        if(_use_loiter_vector_alt>0){
            auto_turn_vector = potential_turn_vector;
        }
        else{
            auto_turn_vector = Vector3f(0,0,1.0f);
        }
    }
    else{
        returnValue.normalize();
        auto_turn_exit_track = _groundspeed_heading_1;
    }


    return returnValue;

}

Vector2f AP_L1_Control::turn_distance_special(const struct Location &previous_wp, const struct Location &current_loc, const struct Location &turn_WP, const struct Location &next_WP, const float roll_rate_deg, const float roll_accel_deg, float _trimspeed, float _minspeed, float _current_roll) //const
{
    //Constant radius turn in air frame
    if(_Turn_Frame_Type == 0){
        return turn_distance_air_frame(current_loc,turn_WP,next_WP,roll_accel_deg,roll_accel_deg,_trimspeed,_minspeed,_current_roll);
    }

    return turn_distance_ground_frame(previous_wp,current_loc,turn_WP,next_WP,_trimspeed);

}

/*
    return velocity vector in air frame
 */
Vector2f AP_L1_Control::get_airspeed_from_wind_ground(const Vector2f wind, const Vector2f ground, const float airspeed) const
{
    if (ground.length()<0.1f || airspeed<2.0f){
        return ground;
    }
    Vector2f G = ground.normalized();
    Vector2f WindInTrack = wind;
    WindInTrack = ground * (WindInTrack * ground)/(ground*ground);
    Vector2f C = WindInTrack - wind;
    float magC = C.length();
    Vector2f A = Vector2f();

    if(airspeed>magC){
        A = G*sqrtf(sq(airspeed)- sq(magC)) +C;
    }
    else{
        A = -wind.normalized()*airspeed;
    }
    return A;
}

float AP_L1_Control::loiter_radius(const float radius) const
{
    // prevent an insane loiter bank limit
    float sanitized_bank_limit = constrain_float(_loiter_bank_limit, 0.0f, 89.0f);
    float lateral_accel_sea_level = tanf(radians(sanitized_bank_limit)) * GRAVITY_MSS;

    float nominal_velocity_sea_level;
    if(_spdHgtControl == nullptr) {
        nominal_velocity_sea_level = 0.0f;
    } else {
        nominal_velocity_sea_level =  _spdHgtControl->get_target_airspeed();
    }

    float eas2tas_sq = sq(_ahrs.get_EAS2TAS());

    if (is_zero(sanitized_bank_limit) || is_zero(nominal_velocity_sea_level) ||
        is_zero(lateral_accel_sea_level)) {
        // Missing a sane input for calculating the limit, or the user has
        // requested a straight scaling with altitude. This will always vary
        // with the current altitude, but will at least protect the airframe
        return radius * eas2tas_sq;
    } else {
        float sea_level_radius = sq(nominal_velocity_sea_level) / lateral_accel_sea_level;
        if (sea_level_radius > radius) {
            // If we've told the plane that its sea level radius is unachievable fallback to
            // straight altitude scaling
            return radius * eas2tas_sq;
        } else {
            // select the requested radius, or the required altitude scale, whichever is safer
            return MAX(sea_level_radius * eas2tas_sq, radius);
        }
    }
}

bool AP_L1_Control::reached_loiter_target(void)
{
    return _WPcircle;
}

/**
   prevent indecision in our turning by using our previous turn
   decision if we are in a narrow angle band pointing away from the
   target and the turn angle has changed sign
 */
void AP_L1_Control::_prevent_indecision(float &Nu)
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

void AP_L1_Control::update_waypoint(const struct Location &prev_WP, const struct Location &next_WP, float dist_min)
{
    Location _current_loc;
    Vector3f velocity;
     // Get current position and velocity
    bool canLoiterTurn = false;
    bool inTurnRadius = false;
    bool inLoiterRadius = false;
    bool canFinishLoiter = false;

    next_auto_waypoint.clone(next_WP);
    prev_auto_waypoint.clone(prev_WP);

    divert_allowed = 0;
    turn_around_allowed = 0;

    Vector2f centre_from_current = Vector2f(0,0);
    Vector2f loiter_from_current = Vector2f(0,0);
    if(_ahrs.get_position(_current_loc) && _ahrs.get_velocity_NED(velocity)){

        // do the "Is it sensible to loiter around the turn point" checks
        inTurnRadius = _current_loc.get_distance(auto_turn_centre)< 2.5*_ground_turn_radius;
        centre_from_current = _current_loc.get_distance_NE(auto_turn_centre);

        Vector2f track_direction = prev_WP.get_distance_NE(next_WP).normalized();
        if(next_WP.get_distance_NE(_current_loc) * (-track_direction) > _ground_turn_radius + _ground_turn_early_initiation){
            divert_allowed = 1;
        }

        if (prev_WP.get_distance_NE(_current_loc) * track_direction >_ground_turn_radius * 3.0f && divert_allowed>0){
            turn_around_allowed = 1;
        }


        float anglePositive = (-centre_from_current.x*velocity.y) + (centre_from_current.y*velocity.x);
        if(anglePositive * auto_turn_clockwise >0 && inTurnRadius){
            canLoiterTurn = true;
        }
        else{
            if(!_initial_turn_complete){
                _initial_turn_complete = true;
                gcs().send_text(MAV_SEVERITY_INFO, "Turn Point Out of Range");
            }
        }


        // do the "Is it sensible to loiter around the loiter point" checks
        inLoiterRadius = _current_loc.get_distance(desired_loiter_point)< 2.5*_ground_turn_radius;
        loiter_from_current = _current_loc.get_distance_NE(desired_loiter_point);
        float loiterAnglePositive = (-loiter_from_current.x*velocity.y) + (loiter_from_current.y*velocity.x);
        if(loiterAnglePositive * _loiter_side > 0 && inLoiterRadius){
            canFinishLoiter = true;
        }
        else{
            if(_use_loiter_for_turn){
                _use_loiter_for_turn = false;
                gcs().send_text(MAV_SEVERITY_INFO, "Loiter Centre Calculation Error");
            }
        }

    }

    if(_use_loiter_for_turn && _Turn_Frame_Type==1 && canFinishLoiter){
        update_loiter(desired_loiter_point,_loiter_radius-_ground_turn_correction_factor,_loiter_side);
        _current_nav_type = 2;
    }


    else if (!initial_turn_complete() && _Turn_Frame_Type==1  && canLoiterTurn){

        desired_loiter_point.clone(auto_turn_centre);
        Vector2f offset = -centre_from_current.normalized()*(_ground_turn_radius -    (_loiter_radius *(_loiter_side*auto_turn_clockwise))  ) ;
        desired_loiter_point.offset(offset.x,offset.y);
        if(_use_loiter_vector_alt>0&&_loiter_side*auto_turn_clockwise>0 ){
            loiter_vector = auto_turn_vector;
        }
        else{
            loiter_vector = Vector3f(0,0,1.0f);
            if(desired_loiter_point.alt<prev_WP.alt){
                desired_loiter_point.alt = prev_WP.alt;
            }
        }

        _loiter_exit_track =  atan2f(-centre_from_current.x*auto_turn_clockwise,centre_from_current.y*auto_turn_clockwise);
        update_loiter(auto_turn_centre,_ground_turn_radius-_ground_turn_correction_factor,auto_turn_clockwise);
        _current_nav_type = 1;
    }
    else{
        update_waypoint_strait(prev_WP,next_WP,dist_min);

        Vector2f target_track = prev_WP.get_distance_NE(next_WP);

         _loiter_exit_track = atan2f(target_track.y,target_track.x);
        _current_nav_type = 0;
    }
}

// update L1 control for waypoint navigation
void AP_L1_Control::update_waypoint_strait(const struct Location &prev_WP, const struct Location &next_WP, float dist_min)
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
    if (_ahrs.get_position(_current_loc) == false) {
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

    // Calculate the NE position of the aircraft relative to WP A
    const Vector2f A_air = prev_WP.get_distance_NE(_current_loc);

    // calculate distance to target track, for reporting
    _crosstrack_error = A_air % AB;

    _stoppingDistance = (groundSpeed*groundSpeed)/(2*_emergency_land_deceleration);

    _crosstrack_velo_portion = _groundspeed_vector.normalized() % AB;

    _groundRiskError = _crosstrack_error + (_stoppingDistance * _crosstrack_velo_portion);

    _groundRiskErrorFiltered = ((dt/_ground_risk_filter_tc)*_groundRiskError) + (((_ground_risk_filter_tc-dt)/_ground_risk_filter_tc)*_groundRiskErrorFiltered);



    if(_turn_around_gre_exception){
        if((_crosstrack_error*_crosstrack_velo_portion<0) && abs(_crosstrack_error)<_max_XT){
            _groundRiskErrorFiltered = 0;
        }
        if (abs(_groundRiskError) < _turn_around_reintercept_tollerance){
            _turn_around_gre_exception = false;
            gcs().send_text(MAV_SEVERITY_INFO, "Turn Around Complete");
        }
    }

    _previousCrosstrack_error = _crosstrack_error;

    //Determine if the aircraft is behind a +-135 degree degree arc centred on WP A
    //and further than L1 distance from WP A. Then use WP A as the L1 reference point
    //Otherwise do normal L1 guidance
    float WP_A_dist = A_air.length();
    float alongTrackDist = A_air * AB;

    Vector2f loiterPoint = AB * alongTrackDist + (Vector2f(-AB.y *_loiter_side, AB.x*_loiter_side)*(_loiter_radius));
    Location temp;
    temp.clone(prev_WP);
    temp.offset(loiterPoint.x,loiterPoint.y);
    int32_t altOffset = (next_WP.alt-prev_WP.alt)*_current_loc.line_path_proportion(prev_WP, next_WP);
    temp.set_alt_cm(prev_WP.alt + altOffset,temp.get_alt_frame());
    desired_loiter_point.clone(temp);
    if(_use_loiter_vector_alt>0 && AB_length >1.0f){
        float heightToClimb = (next_WP.alt - prev_WP.alt)/100.0f;
        Vector2f xyComponent = -AB *heightToClimb;
        float vertical_component = AB_length;
        if(heightToClimb<0.0f){
            vertical_component = -AB_length;
        }
        loiter_vector = Vector3f(xyComponent.x,xyComponent.y,vertical_component).normalized();
    }
    else{
        loiter_vector = Vector3f(0,0,1.0f);
    }

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
void AP_L1_Control::update_loiter(const struct Location &center_WP, float radius, int8_t loiter_direction)
{
    divert_allowed = 0;
    turn_around_allowed = 0;

    uint32_t now = AP_HAL::micros();
    float dt = (now - _last_update_waypoint_us) * 1.0e-6f;
    if (dt > 0.1) {
        dt = 0.1;
        _L1_xtrack_i = 0.0f;
        _groundRiskErrorFiltered = 0.0f;
    }
    _last_update_waypoint_us = now;


    if(prev_auto_waypoint.lat != 0 && next_auto_waypoint.lat!=0){
    Vector2f track_direction = prev_auto_waypoint.get_distance_NE(next_auto_waypoint).normalized();
        if(next_auto_waypoint.get_distance_NE(center_WP) * (-track_direction) > _ground_turn_radius + _ground_turn_early_initiation){
            divert_allowed = 1;
        }

        if (prev_auto_waypoint.get_distance_NE(center_WP) * track_direction >_ground_turn_radius * 3.0f && divert_allowed>0){
            turn_around_allowed = 1;
        }
    }

    struct Location _current_loc;

    // scale loiter radius with square of EAS2TAS to allow us to stay
    // stable at high altitude
    //radius = loiter_radius(fabsf(radius));

    // Calculate guidance gains used by PD loop (used during circle tracking)
    float omega = (6.2832f / _L1_period);
    float Kx = omega * omega;
    float Kv = 2.0f * _L1_damping * omega;

    // Calculate L1 gain required for specified damping (used during waypoint capture)
    float K_L1 = 4.0f * _L1_damping * _L1_damping;

    //Get current position and velocity
    if (_ahrs.get_position(_current_loc) == false) {
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
            A_air_unit = Vector2f(cosf(_ahrs.yaw), sinf(_ahrs.yaw));
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

    _stoppingDistance = (groundSpeed*groundSpeed)/(2*_emergency_land_deceleration);

    _crosstrack_velo_portion = loiter_direction*xtrackVelCirc/groundSpeed;

    _groundRiskError = _crosstrack_error + (_stoppingDistance * _crosstrack_velo_portion);

    _groundRiskErrorFiltered = ((dt/_ground_risk_filter_tc)*_groundRiskError) + (((_ground_risk_filter_tc-dt)/_ground_risk_filter_tc)*_groundRiskErrorFiltered);

    if(_turn_around_gre_exception){
        if( abs(_crosstrack_error)<_max_XT){
            _groundRiskErrorFiltered = 0;
        }
    }

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
void AP_L1_Control::update_heading_hold(int32_t navigation_heading_cd)
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

// update L1 control for level flight on current heading
void AP_L1_Control::update_level_flight(void)
{
    // copy to _target_bearing_cd and _nav_bearing
    _target_bearing_cd = _ahrs.yaw_sensor;
    _nav_bearing = _ahrs.yaw;
    _bearing_error = 0;
    _crosstrack_error = 0;

    // Waypoint capture status is always false during heading hold
    _WPcircle = false;

    _latAccDem = 0;

    _data_is_stale = false; // status are correctly updated with current waypoint data
}
