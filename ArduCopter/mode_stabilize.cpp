#include "Copter.h"

/*
 * Init and run calls for stabilize flight mode
 */

// stabilize_run - runs the main stabilize controller
// should be called at 100hz or more
void ModeStabilize::run()
{
    uint32_t now = micros();
    
    // apply simple mode transform to pilot inputs
    update_simple_mode();

    // convert pilot input to lean angles
    float target_roll, target_pitch;
    get_pilot_desired_lean_angles(target_roll, target_pitch, copter.aparm.angle_max, copter.aparm.angle_max);

    float dt = (now - last_run_time)/1000000.0f;
    last_run_time = now;
    // get pilot's desired yaw rate
    float input_yaw_rate =get_pilot_desired_yaw_rate(channel_yaw->norm_input_dz());
    target_encoder_value = target_encoder_value + ((input_yaw_rate/100)*dt);

    AP_WheelEncoder &wheelEncoder2 = AP::wheelencoder();
    
    float target_yaw_rate = copter.g2.EncoderPosHold.update_all(target_encoder_value,wheelEncoder2.get_distance(0),dt);

    if ((now- last_message_time)/1000000 >1.0f ){
        //send mavlink gcs text message
        /*
        copter.gcs().send_text(MAV_SEVERITY_INFO, "EncoderPosHold: %f ",copter.g2.wheel_encoder.get_distance(0));
        copter.gcs().send_text(MAV_SEVERITY_INFO, "TargetRate: %f ",input_yaw_rate);
        copter.gcs().send_text(MAV_SEVERITY_INFO, "Dt: %f ", dt);
        copter.gcs().send_text(MAV_SEVERITY_INFO, "Encoder Target: %f ",copter.g2.EncoderPosHold.get_pid_info().target);
        copter.gcs().send_text(MAV_SEVERITY_INFO, "Encoder Actual: %f ", copter.g2.EncoderPosHold.get_pid_info().actual);
        copter.gcs().send_text(MAV_SEVERITY_INFO, "Output Rate: %f ", target_yaw_rate);
        last_message_time = now;
        */

    }

  
    //Run encoder to rate rate PID
    //float target_yaw_rate = output of rate PID;
    //float target_yaw_rate =get_pilot_desired_yaw_rate(channel_yaw->norm_input_dz());

    if (!motors->armed()) {
        // Motors should be Stopped
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
        AP_WheelEncoder &wheelEncoder = AP::wheelencoder();
        target_encoder_value = wheelEncoder.get_distance(0);

    } else if (copter.ap.throttle_zero
               || (copter.air_mode == AirMode::AIRMODE_ENABLED && motors->get_spool_state() == AP_Motors::SpoolState::SHUT_DOWN)) {
        // throttle_zero is never true in air mode, but the motors should be allowed to go through ground idle
        // in order to facilitate the spoolup block

        // Attempting to Land
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
    } else {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    }

    float pilot_desired_throttle = get_pilot_desired_throttle();

    switch (motors->get_spool_state()) {
    case AP_Motors::SpoolState::SHUT_DOWN:
        // Motors Stopped
        attitude_control->reset_yaw_target_and_rate();
        attitude_control->reset_rate_controller_I_terms();
        pilot_desired_throttle = 0.0f;
        break;

    case AP_Motors::SpoolState::GROUND_IDLE:
        // Landed
        attitude_control->reset_yaw_target_and_rate();
        attitude_control->reset_rate_controller_I_terms_smoothly();
        pilot_desired_throttle = 0.0f;
        break;

    case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
        // clear landing flag above zero throttle
        if (!motors->limit.throttle_lower) {
            set_land_complete(false);
        }
        break;

    case AP_Motors::SpoolState::SPOOLING_UP:
    case AP_Motors::SpoolState::SPOOLING_DOWN:
        // do nothing
        break;
    }

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);

    // output pilot's throttle
    attitude_control->set_throttle_out(pilot_desired_throttle, true, g.throttle_filt);
}
