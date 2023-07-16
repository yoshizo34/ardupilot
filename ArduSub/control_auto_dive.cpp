#include "Sub.h"

/*
 * control_auto.cpp
 *  Contains the mission, waypoint navigation and NAV_CMD item implementation
 *
 *  While in the auto flight mode, navigation or do/now commands can be run.
 *  Code in this file implements the navigation commands
 */

// auto_init - initialise auto controller
bool Sub::auto_dive_init()
{
    if (!position_ok()) {
        return false;
    }
        // initialize speeds and accelerations
    pos_control.set_max_speed_accel_xy(wp_nav.get_default_speed_xy(), wp_nav.get_wp_acceleration());
    pos_control.set_correction_speed_accel_xy(wp_nav.get_default_speed_xy(), wp_nav.get_wp_acceleration());
    pos_control.set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    pos_control.set_correction_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // initialise circle controller including setting the circle center based on vehicle speed
    circle_nav.init();

    pos_control.init_z_controller();
    pos_control.init_xy_controller();
       // start in position control mode
    return true;

}

// auto_run - runs the appropriate auto controller
// according to the current auto_mode
// should be called at 100hz or more
void Sub::auto_dive_run()
{
        // there is a pressure sensor for reading depth instead of barometer.
    float target_yaw_rate = 0;
    read_barometer();
    float current_depth = barometer.get_altitude();

    // when vehicle is not armed....
     if (!motors.armed()) {
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        // Sub vehicles do not stabilize roll/pitch/yaw when disarmed
        attitude_control.set_throttle_out(0,true,g.throttle_filt);
        attitude_control.relax_attitude_controllers();
        // initialise velocity controller
        pos_control.init_z_controller();
        pos_control.init_xy_controller();
        return;
    }

    // Set target depth(cm)  
    target_Depth = -10000;
    // set motors to full range
    motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // when motors are spooling up.... running the attitude control 
    if (motors.get_spool_state() != AP_Motors::SpoolState::THROTTLE_UNLIMITED) {
        pos_control.relax_velocity_controller_xy();
        pos_control.update_xy_controller();
        pos_control.relax_z_controller(0.0f);  
        pos_control.update_z_controller();
        attitude_control.reset_yaw_target_and_rate();
        attitude_control.reset_rate_controller_I_terms();
        attitude_control.input_thrust_vector_rate_heading(pos_control.get_thrust_vector(), 0.0);
        target_Depth = inertial_nav.get_position_z_up_cm() + -1000;
        return;
    }

    // target depth 
    if (target_Depth < current_depth) {
        pos_control.set_pos_target_z_from_climb_rate_cm(Auto_DIVE_Define_descent_rate);
        pos_control.update_z_controller();
    // process pilot inputs
    // get pilot's desired yaw rate
    target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
    if (!is_zero(target_yaw_rate)) {
        circle_pilot_yaw_override = true;
        }

        failsafe_terrain_set_status(circle_nav.update());

        ///////////////////////
        // update xy outputs //

        float lateral_out, forward_out;
        translate_circle_nav_rp(lateral_out, forward_out);

        // Send to forward/lateral outputs
        motors.set_lateral(lateral_out);
        motors.set_forward(forward_out);

         if (circle_pilot_yaw_override) {
            attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_yaw_rate);
        } else {
            attitude_control.input_euler_angle_roll_pitch_yaw(channel_roll->get_control_in(), channel_pitch->get_control_in(), circle_nav.get_yaw(), true);
        }

    }else{
        control_depth();
        return;
    }
    

    
}
