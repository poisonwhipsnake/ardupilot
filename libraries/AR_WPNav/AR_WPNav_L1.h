#pragma once

#include "AR_WPNav.h"

class AR_WPNav_L1 : public AR_WPNav {
public:

    // re-use parent's constructor
    AR_WPNav_L1(AP_AHRS &ahrs,AR_AttitudeControl& atc, AR_PosControl &pos_control)
        : AR_WPNav(atc,pos_control), _ahrs(ahrs)  // Call the AR_WPNav constructor here
    {
        AP_Param::setup_object_defaults(this, var_info);
    }


    bool set_waypoint_speed(float speed) override;


    void init(float speed_max = 0) override;

    // update navigation
    void update(float dt) override;

    // set desired location and (optionally) next_destination
    // next_destination should be provided if known to allow smooth cornering
    bool set_desired_location(const Location &destination, Location next_destination = Location()) override WARN_IF_UNUSED;


        /* Do not allow copies */
    AR_WPNav_L1(const AR_WPNav_L1 &other) = delete;
    AR_WPNav_L1 &operator=(const AR_WPNav_L1&) = delete;

    /* see AP_Navigation.h for the definitions and units of these
     * functions */
    //int32_t nav_roll_cd(void) const ;
    //int32_t nav_roll_cd_special(float _amax, float _rmax, float _trimspeed, float _minspeed, Location current) ;

    float nav_steering_angle(float groundspeed, float wheelbase, float _steering_angle_max, float _steering_angle_max_rate, float _steering_angle_max_accel, float turn_radius) ;
    
    float lateral_acceleration(void) const ;

    void reset(void) ;
    uint8_t turn_around_ok(void) const  {return turn_around_allowed;}
    uint8_t divert_ok(void) const  {return divert_allowed;}

    //uint8_t orbit_ok(void) const  {return orbit_allowed;}

    // return the desired track heading angle(centi-degrees)
    int32_t nav_bearing_cd(void) const ;

    // return the heading error angle (centi-degrees) +ve to left of track
    int32_t bearing_error_cd(void) const ;

    float crosstrack_error(void) const  { return _crosstrack_error; }
    float groundRisk_error(void) const  { return _groundRiskError; }
    float groundRisk_error_flt(void) const { return _groundRiskErrorFiltered; }
    float stopping_distance(void) const  { return _stoppingDistance;}
    float crosstrack_velo_theta(void) const  { return _crosstrack_velo_portion;}
    float nav_bearing(void) const  { return _nav_bearing;}
    float crosstrack_velo(void) const ;

    //virtual bool loiter_for_turn(void) const  { return _loiter_turn_state != LOITER_NONE || !_initial_turn_complete; }
    //virtual bool loiter_for_orbit(void) const  { return _loiter_turn_state >= LOITER_ORBIT; }
    //virtual bool loiter_for_turnaround(void) const  { return _loiter_turn_state == LOITER_TURNAROUND_1 || _loiter_turn_state == LOITER_TURNAROUND_2; }
    //virtual bool loiter_for_turnaround2(void) const  { return _loiter_turn_state == LOITER_TURNAROUND_2; }

    float crosstrack_error_integrator(void) const  { return _L1_xtrack_i; }

    int32_t target_bearing_cd(void) const ;
    //Location get_loiter_location(void) const  {return loiter_point;}
    //Vector3f get_loiter_vector(void) const  {return loiter_vector;}
    //Vector3f get_auto_turn_vector(void) const  {return auto_turn_vector;}
    Location get_auto_turn_centre(void) const  {return auto_turn_centre;}
    //int8_t get_loiter_direction(void) const  {return _loiter_side;}
    //float get_loiter_radius(void) const  {return _loiter_radius;}
    //float get_loiter_exit_angle_remaining(void) const {return (wrap_2PI(auto_turn_exit_track-(atan2f(_ahrs.groundspeed_vector().y,_ahrs.groundspeed_vector().x))+ M_PI) - M_PI) * auto_turn_clockwise;}

    float get_intercept_tolerance (void) const  {return _max_auto_point_distance ;}
    //float turn_distance(float wp_radius) const ;
    //float turn_distance(float groundspeed, float turn_angle) const ;
    //Vector2f turn_distance_special(const struct Location &prev_wp, const struct Location &current_loc,const struct Location &turn_WP, const struct Location &next_WP, const float roll_rate, const float roll_accel, float _trimspeed, float _minspeed, float current_roll) ;//const override;
    Vector2f turn_distance_ground_frame( const struct Location &prev_wp,const struct Location &current_loc,const struct Location &turn_WP, const struct Location &next_WP, float _trimspeed) ;//const;

    /*
    enum LoiterTurnState {
        LOITER_NONE,            // Not in a loiter state
        LOITER_ORBIT,           // Currently in an orbit state
        LOITER_TURNAROUND_1,    // Currently in the first stage of turnaround (270deg turn in the orbit direction)
        LOITER_TURNAROUND_2     // Currently in the second stage of turnaround (90deg turn opposite the orbit direction to reintercept)
    };
    */

    Vector2f get_airspeed_from_wind_ground(const Vector2f wind, const Vector2f ground, float airspeed) const;

    //float loiter_radius (const float loiter_radius) const ;
    void update_waypoint(const struct Location &prev_WP, const struct Location &next_WP, float dist_min = 0.0f) ;
    void update_waypoint_straight(const struct Location &prev_WP, const struct Location &next_WP, float dist_min) ;
    void update_loiter(const struct Location &center_WP, float radius, int8_t loiter_direction) ;
    void update_heading_hold(int32_t navigation_heading_cd) ;
    void ground_risk_exclusion_event_trigger();

    void update_speed_demand(float dt);
    //void update_level_flight(void) ;
    bool reached_loiter_target(void) ;

    int8_t get_current_nav_type(void)  {return _current_nav_type;}
    void set_current_nav_type(int8_t v)  { _current_nav_type = v; }
    bool initial_turn_complete(void) ;
    bool in_turn(void)  {return !_initial_turn_complete; }      // are we in a turn? (or is one pending AUTO mode resume ie. loitered during a turn )
    void start_new_turn(void) ;
    void setup_loiter_to_track(void) ;
    void setup_loiter_to_new_track(Vector2f newTrack, Location LoiterPoint2) ;
    // set the default NAVL1_PERIOD
    void set_default_period(float period) {
        _L1_period.set_default(period);
    }

      float get_auto_nav_bank(){
        return _auto_bank_limit;
    }
    void set_data_is_stale(void)  {
        _data_is_stale = true;
    }
    bool data_is_stale(void) const  {
        return _data_is_stale;
    }

    void set_reverse(bool reverse)  {
        _reverse = reverse;
    }

    // this supports the NAVl1_* user settable parameters
    static const struct AP_Param::GroupInfo var_info[];

private:

    // update distance and bearing from vehicle's current position to destination
    void update_oa_distance_and_bearing_to_destination();

    // object avoidance variables
    bool _oa_active;                // true if we should use alternative destination to avoid obstacles
    Location _origin_oabak;         // backup of _origin so it can be restored when oa completes
    Location _destination_oabak;    // backup of _desitnation so it can be restored when oa completes
    Location _next_destination_oabak; // backup of _next_destination so it can be restored when oa completes
    Location _oa_origin;            // intermediate origin during avoidance
    Location _oa_destination;       // intermediate destination during avoidance
    Location _oa_next_destination;  // intermediate next destination during avoidance
    float _oa_distance_to_destination; // OA (object avoidance) distance from vehicle to _oa_destination in meters
    float _oa_wp_bearing_cd;        // OA adjusted heading to _oa_destination in centi-degrees

     /////New params for L1////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // reference to the AHRS object
    AP_AHRS &_ahrs;

        // lateral acceration in m/s required to fly to the
    // L1 reference point (+ve to right)
    float _latAccDem;

    // L1 tracking distance in meters which is dynamically updated
    float _L1_dist;

    // Status which is true when the vehicle has started circling the WP
    bool _WPcircle;

    // bearing angle (radians) to L1 point
    float _nav_bearing;

    // Have we completed the initial turn to make our heading correct to intercept with the new track?
    // (ie. not in a navigation turn)
    bool _initial_turn_complete;

    // bearing error angle (radians) +ve to left of track
    float _bearing_error;

    // crosstrack error in meters
    float _crosstrack_error;

    float _previousCrosstrack_error;

    float _stoppingDistance;

    float _groundRiskError;

    float _groundRiskErrorFiltered;

    float _crosstrack_velo_portion;

    float _steering_position;
    float _steering_rate;

    AP_Float _emergency_land_deceleration;

    AP_Float _ground_risk_filter_tc;

    AP_Float _ground_risk_exclusion_timeout;

    uint32_t ground_risk_exclusion_event_time;

    float _unsmoothed_bank_angle_cd;
    // target bearing in centi-degrees from last update
    int32_t _target_bearing_cd;

    // L1 tracking loop period (sec)
    AP_Float _L1_period;
    // L1 tracking loop damping ratio
    AP_Float _L1_damping;

    // previous value of cross-track velocity
    float _last_Nu;

    // prevent indecision in waypoint tracking
    void _prevent_indecision(float &Nu);

    // integral feedback to correct crosstrack error. Used to ensure xtrack converges to zero.
    // For tuning purposes it's helpful to clear the integrator when it changes so a _prev is used
    float _L1_xtrack_i = 0;
    AP_Float _L1_xtrack_i_gain;
    AP_Float _auto_bank_limit;
    AP_Float _turn_rate_correction_factor;
    AP_Float _L1_Auto_Period;
    AP_Float _L1_Turn_Exit_Fraction;
    AP_Float _L1_Mid_Turn_Gain;
    AP_Float _L1_Turn_Delay;
    float _L1_xtrack_i_gain_prev = 0;
    uint32_t _last_update_waypoint_us;
    uint32_t _last_nav_angle_update_us;
    uint32_t DebugTimer;
    int32_t previous_roll_cd;
    int32_t previous_roll_update_time;
    AP_Int8 _Turn_Frame_Type;                       // 0 = constant radius in air frame, 1 = constant radius in ground frame"
    //AP_Int8 _loiter_side;
    //AP_Int8 _use_loiter_vector_alt;
    //AP_Float _loiter_radius;
    AP_Float _max_auto_point_distance;
    AP_Float _ground_turn_radius;
    AP_Float _ground_turn_correction_factor;
    AP_Float _ground_turn_early_initiation;
    AP_Float _steering_angle_max_param;
    AP_Float _speed_max_param;
    AP_Float _accel_max;
    AP_Float _decel_max;
    AP_Float _turn_lateral_G;

    AP_Float _steering_angle_velocity_param;
    AP_Float _steering_angle_acceleration_param;
    AP_Float _steering_wheelbase;
    bool _data_is_stale = true;
    //float _loiter_exit_angle;


    uint8_t divert_allowed;
    uint8_t turn_around_allowed;
    //uint8_t orbit_allowed;

    //Vector3f loiter_vector;
    //Vector3f auto_turn_vector;

    int8_t _current_nav_type; // 0: Straight, 1: Turn, 2: Orbit (todo: make enum)

    //Location loiter_point;
    //Location loiter_point_2;
    Location auto_turn_centre;
    Location next_auto_waypoint;
    Location current_auto_waypoint;
    Location prev_auto_waypoint;
    int8_t auto_turn_clockwise = true;
    float auto_turn_exit_track;
    // How many radii can we be away from the centrepoint of a turn to be considered 'in the turn'
    // Actual value should be sqrt(2), making it 2 adds an extra error margin
    const float in_turn_error_scalar = 2;

    AP_Float _loiter_bank_limit;

    bool _reverse = false;

    float current_speed;
    float waypoint_radius;
    float prev_waypoint_radius;
    float waypoint_speed;

    float get_yaw();
    int32_t get_yaw_sensor() const;
};
