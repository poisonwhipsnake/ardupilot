#include "Rover.h"

void ModeHold::update()
{
    float throttle = 0.0f;

    // if vehicle is balance bot, calculate actual throttle required for balancing
    if (rover.is_balancebot()) {
        rover.balancebot_pitch_control(throttle);
    }

    // relax mainsail
    g2.sailboat.relax_sails();

    // update navigation controller to hold the track, but stop the vehicle
    navigate_to_waypoint();
    // hold position - stop motors and center steering
    g2.motors.set_throttle(throttle);
}
