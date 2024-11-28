#pragma once

/// @file    AP_L1_Control.h
/// @brief   L1 Control algorithm. This is a instance of an
/// AP_Navigation class

/*
 * Originally written by Brandon Jones 2013
 *
 *  Modified by Paul Riseborough 2013 to provide:
 *  - Explicit control over frequency and damping
 *  - Explicit control over track capture angle
 *  - Ability to use loiter radius smaller than L1 length
 */

#include <AP_Math/AP_Math.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Param/AP_Param.h>
#include <AP_Navigation/AP_Navigation.h>
#include <AP_SpdHgtControl/AP_SpdHgtControl.h>
#include <AP_Common/Location.h>

class AP_L1_Control : public AP_Navigation {
public:
    AP_L1_Control(AP_AHRS &ahrs, const AP_SpdHgtControl *spdHgtControl)
        : _ahrs(ahrs)
        , _spdHgtControl(spdHgtControl)
    {
        AP_Param::setup_object_defaults(this, var_info);
    }

    /* Do not allow copies */
    AP_L1_Control(const AP_L1_Control &other) = delete;
    AP_L1_Control &operator=(const AP_L1_Control&) = delete;

    /* see AP_Navigation.h for the definitions and units of these
     * functions */
    int32_t nav_roll_cd(void) const override;
    int32_t nav_roll_cd_special(float _amax, float _rmax, float _trimspeed, float _minspeed, Location current) override;
    float lateral_acceleration(void) const override;

    void reset(void) override;
    uint8_t turn_around_ok(void) const override {return turn_around_allowed;}
    uint8_t divert_ok(void) const override {return divert_allowed;}

    uint8_t orbit_ok(void) const override {return orbit_allowed;}

    // return the desired track heading angle(centi-degrees)
    int32_t nav_bearing_cd(void) const override;

    // return the heading error angle (centi-degrees) +ve to left of track
    int32_t bearing_error_cd(void) const override;

    float crosstrack_error(void) const override { return _crosstrack_error; }
    float groundRisk_error(void) const override { return _groundRiskError; }
    float groundRisk_error_flt(void) const override{ return _groundRiskErrorFiltered; }
    float stopping_distance(void) const override { return _stoppingDistance;}
    float crosstrack_velo_theta(void) const override { return _crosstrack_velo_portion;}
    float nav_bearing(void) const override { return _nav_bearing;}
    float crosstrack_velo(void) const override;

    virtual bool loiter_for_turn(void) const override { return _loiter_turn_state != LOITER_NONE || !_initial_turn_complete; }
    virtual bool loiter_for_orbit(void) const override { return _loiter_turn_state >= LOITER_ORBIT; }
    virtual bool loiter_for_turnaround(void) const override { return _loiter_turn_state == LOITER_TURNAROUND_1 || _loiter_turn_state == LOITER_TURNAROUND_2; }
    virtual bool loiter_for_turnaround2(void) const override { return _loiter_turn_state == LOITER_TURNAROUND_2; }

    float crosstrack_error_integrator(void) const override { return _L1_xtrack_i; }

    int32_t target_bearing_cd(void) const override;
    Location get_loiter_location(void) const override {return loiter_point;}
    Vector3f get_loiter_vector(void) const override {return loiter_vector;}
    Vector3f get_auto_turn_vector(void) const override {return auto_turn_vector;}
    Location get_auto_turn_centre(void) const override {return auto_turn_centre;}
    int8_t get_loiter_direction(void) const override {return _loiter_side;}
    float get_loiter_radius(void) const override {return _loiter_radius;}
    float get_loiter_exit_angle_remaining(void) const {return (wrap_2PI(auto_turn_exit_track-(atan2f(_ahrs.groundspeed_vector().y,_ahrs.groundspeed_vector().x))+ M_PI) - M_PI) * auto_turn_clockwise;}

    float get_intercept_tolerance (void) const override {return _max_auto_point_distance ;}
    float turn_distance(float wp_radius) const override;
    float turn_distance(float groundspeed, float turn_angle) const override;
    Vector2f turn_distance_special(const struct Location &prev_wp, const struct Location &current_loc,const struct Location &turn_WP, const struct Location &next_WP, const float roll_rate, const float roll_accel, float _trimspeed, float _minspeed, float current_roll) override;//const override;
    Vector2f turn_distance_air_frame( const struct Location &current_loc,const struct Location &turn_WP, const struct Location &next_WP, const float roll_rate, const float roll_accel, float _trimspeed, float _minspeed, float current_roll) ;//const;
    Vector2f turn_distance_ground_frame( const struct Location &prev_wp,const struct Location &current_loc,const struct Location &turn_WP, const struct Location &next_WP, float _trimspeed) ;//const;

    enum LoiterTurnState {
        LOITER_NONE,            // Not in a loiter state
        LOITER_ORBIT,           // Currently in an orbit state
        LOITER_TURNAROUND_1,    // Currently in the first stage of turnaround (270deg turn in the orbit direction)
        LOITER_TURNAROUND_2     // Currently in the second stage of turnaround (90deg turn opposite the orbit direction to reintercept)
    };

    Vector2f get_airspeed_from_wind_ground(const Vector2f wind, const Vector2f ground, float airspeed) const;

    float loiter_radius (const float loiter_radius) const override;
    void update_waypoint(const struct Location &prev_WP, const struct Location &next_WP, float dist_min = 0.0f) override;
    void update_waypoint_straight(const struct Location &prev_WP, const struct Location &next_WP, float dist_min) override;
    void update_loiter(const struct Location &center_WP, float radius, int8_t loiter_direction) override;
    void update_heading_hold(int32_t navigation_heading_cd) override;
    void ground_risk_exclusion_event_trigger();
    void update_level_flight(void) override;
    bool reached_loiter_target(void) override;

    int8_t get_current_nav_type(void) override {return _current_nav_type;}
    void set_current_nav_type(int8_t v) override { _current_nav_type = v; }
    bool initial_turn_complete(void) override;
    bool in_turn(void) override {return !_initial_turn_complete; }      // are we in a turn? (or is one pending AUTO mode resume ie. loitered during a turn )
    void start_new_turn(void) override;
    void setup_loiter_to_track(void) override;
    void setup_loiter_to_new_track(Vector2f newTrack, Location LoiterPoint2) override;
    // set the default NAVL1_PERIOD
    void set_default_period(float period) {
        _L1_period.set_default(period);
    }

      float get_auto_nav_bank(){
        return _auto_bank_limit;
    }
    void set_data_is_stale(void) override {
        _data_is_stale = true;
    }
    bool data_is_stale(void) const override {
        return _data_is_stale;
    }

    // this supports the NAVl1_* user settable parameters
    static const struct AP_Param::GroupInfo var_info[];

    void set_reverse(bool reverse) override {
        _reverse = reverse;
    }

private:
    // reference to the AHRS object
    AP_AHRS &_ahrs;

    // pointer to the SpdHgtControl object
    const AP_SpdHgtControl *_spdHgtControl;

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
    AP_Int8 _loiter_side;
    AP_Int8 _use_loiter_vector_alt;
    AP_Float _loiter_radius;
    AP_Float _max_auto_point_distance;
    AP_Float _ground_turn_radius;
    AP_Float _ground_turn_correction_factor;
    AP_Float _ground_turn_early_initiation;
    bool _data_is_stale = true;
    float _loiter_exit_angle;
    LoiterTurnState _loiter_turn_state = LOITER_NONE;

    uint8_t divert_allowed;
    uint8_t turn_around_allowed;
    uint8_t orbit_allowed;

    Vector3f loiter_vector;
    Vector3f auto_turn_vector;

    int8_t _current_nav_type; // 0: Straight, 1: Turn, 2: Orbit (todo: make enum)

    Location loiter_point;
    Location loiter_point_2;
    Location auto_turn_centre;
    Location next_auto_waypoint;
    Location prev_auto_waypoint;
    int8_t auto_turn_clockwise = true;
    float auto_turn_exit_track;
    // How many radii can we be away from the centrepoint of a turn to be considered 'in the turn'
    // Actual value should be sqrt(2), making it 2 adds an extra error margin
    const float in_turn_error_scalar = 2;

    AP_Float _loiter_bank_limit;

    bool _reverse = false;
    float get_yaw();
    int32_t get_yaw_sensor() const;
};
