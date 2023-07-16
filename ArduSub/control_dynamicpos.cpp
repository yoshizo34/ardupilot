#include "Sub.h"

/*
 * Init and run calls for survey flight mode
 */

// survey_init - initialise survey controller
bool Sub::DP_init()
{

    // initialize speeds and accelerations
    pos_control.set_max_speed_accel_xy(wp_nav.get_default_speed_xy(), wp_nav.get_wp_acceleration());
    pos_control.set_correction_speed_accel_xy(wp_nav.get_default_speed_xy(), wp_nav.get_wp_acceleration());
    pos_control.set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    pos_control.set_correction_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    pos_control.init_z_controller();
    pos_control.init_xy_controller();

    return true;
}

void Sub::DP_run()
{
    motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    pos_control.set_pos_target_xy_cm(500.0f,0.0f);
    pos_control.set_pos_target_z_cm(-1000.0f);
    pos_control.update_xy_controller();
    pos_control.update_z_controller();
    float lateral_out, forward_out;
    translate_pos_control_rp(lateral_out, forward_out);
    motors.set_lateral(lateral_out);
    motors.set_forward(forward_out);
    
}
