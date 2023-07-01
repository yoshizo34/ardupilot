#include "Sub.h"
#include "GCS_Mavlink.h"
// stabilize_init - initialise stabilize controller
bool Sub::hdhold_init()
{
    // set target altitude to zero for reporting
    pos_control.set_pos_target_z_cm(0);
    HDHOLD_state = HDHOLD_last = HDhold_init;   
    gcs().send_text( MAV_SEVERITY_DEBUG, "mode heading hold init." );
    return true;
}

// stabilize_run - runs the main stabilize controller
// should be called at 100hz or more
void Sub::hdhold_run()
{
    float target_roll, target_pitch;
    last_pilot_heading = ahrs.yaw_sensor;
    target_heading = g.set_heading;
    gcs().send_text( MAV_SEVERITY_DEBUG, "ahrs.yaw_sensor: %i(deg)",    last_pilot_heading );

    switch(HDHOLD_state){
        case HDhold_init:
            gcs().send_text( MAV_SEVERITY_DEBUG, "hdhold_init." );
            if (!motors.armed()) {
                // To-Do: add some initialisation of position controllers
                motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
                // Sub vehicles do not stabilize roll/pitch/yaw when disarmed
                attitude_control.set_throttle_out(0,true,g.throttle_filt);
                attitude_control.relax_attitude_controllers();
                circle_nav.init();
                return;
            }
            
            HDHOLD_state = HDhold_turn;
        break;

        case HDhold_turn:
            gcs().send_text( MAV_SEVERITY_DEBUG, "hdhold_turn." );
            motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
            attitude_control.input_euler_angle_roll_pitch_yaw(0, 0, target_heading, true);
            if (last_pilot_heading == target_heading){
                HDHOLD_state = HDhold_done;
            }
        break;

        case HDhold_done:
            gcs().send_text( MAV_SEVERITY_DEBUG, "hdhold_done." );
            get_pilot_desired_lean_angles(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_roll, target_pitch, aparm.angle_max);
            attitude_control.input_euler_angle_roll_pitch_yaw(target_roll, target_pitch, target_heading, true);
            attitude_control.set_throttle_out(channel_throttle->norm_input(), false, g.throttle_filt);
            motors.set_forward(channel_forward->norm_input());
            motors.set_lateral(channel_lateral->norm_input());
        break;

    }

}
