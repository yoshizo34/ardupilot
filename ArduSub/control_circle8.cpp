#include "Sub.h"

/*
 * control_circle8.cpp - init and run calls for circle flight mode
 */

// circle8_init - initialise circle8 controller flight mode
bool Sub::circle8_init()
{
    if (!position_ok()) {
        return false;
    }

    gcs().send_text(MAV_SEVERITY_INFO, "mode circle8 init.");

    circle_pilot_yaw_override = false;

    // initialize speeds and accelerations
    pos_control.set_max_speed_accel_xy(wp_nav.get_default_speed_xy(), wp_nav.get_wp_acceleration());
    pos_control.set_correction_speed_accel_xy(wp_nav.get_default_speed_xy(), wp_nav.get_wp_acceleration());
    pos_control.set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    pos_control.set_correction_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // initialise circle controller including setting the circle center based on vehicle speed
    circle_nav.init();

#if 1
    // current location saveed.
    LocBeginC8 = current_loc; 
    Circle8State = Circle8_init;

    Circle8CenterPos = circle_nav.get_center();
    Circle8NextCenterPos = Circle8CenterPos;
    Circle8NextCenterPos.rotate( ROTATION_YAW_180 );

    Circle8Radius = circle_nav.get_radius() / 100.0;    //  cmなのでmに変換

    gcs().send_text(MAV_SEVERITY_INFO, "center: ( %f, %f )", Circle8CenterPos.x, Circle8CenterPos.y );
    gcs().send_text(MAV_SEVERITY_INFO, "2nd center: ( %f, %f )", Circle8NextCenterPos.x, Circle8NextCenterPos.y );
    gcs().send_text(MAV_SEVERITY_INFO, "radius: %f, %f(m) )", circle_nav.get_radius(), Circle8Radius );

#endif

    return true;
}

// circle8_run - runs the circle8 flight mode
// should be called at 100hz or more
void Sub::circle8_run()
{
    float target_yaw_rate = 0;
    float target_climb_rate = 0;

    // update parameters, to allow changing at runtime
    pos_control.set_max_speed_accel_xy(wp_nav.get_default_speed_xy(), wp_nav.get_wp_acceleration());
    pos_control.set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // if not armed set throttle to zero and exit immediately
    if (!motors.armed()) {
        // To-Do: add some initialisation of position controllers
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        // Sub vehicles do not stabilize roll/pitch/yaw when disarmed
        attitude_control.set_throttle_out(0,true,g.throttle_filt);
        attitude_control.relax_attitude_controllers();
        circle_nav.init();
        return;
    }

#if 1
{
    float Distance = ( float ) current_loc.get_distance( LocBeginC8 );
//    gcs().send_text(MAV_SEVERITY_INFO, "mode circle8 run.");
//    gcs().send_text(MAV_SEVERITY_INFO, "distance: %f", Distance );

    switch( Circle8State ){
        case Circle8_init:
            Circle8State = Circle8_wait1stHalf;
            gcs().send_text(MAV_SEVERITY_INFO, "state: init -> wait1sthalf" );
        break;

        case Circle8_wait1stHalf:
            if( Distance >  20.0 - 0.5 )
            {
                Circle8State = Circle8_wait1stDone;
                gcs().send_text(MAV_SEVERITY_INFO, "state: wait1sthalf -> wait1stdone" );
            }
        break;

        case Circle8_wait1stDone:
            if( Distance < 0.5 )
            {
                Circle8State = Circle8_wait2ndHalf;

                Location NewCenter( Circle8NextCenterPos, LocBeginC8.get_alt_frame() );
                circle_nav.set_center( NewCenter );
                LocBeginC8 = current_loc; 

                gcs().send_text(MAV_SEVERITY_INFO, "state: wait1stdone -> wait2ndHalf" );
            }
        break;

        case Circle8_wait2ndHalf:
            if( Distance > 20.0 - 0.5 )
            {
                Circle8State = Circle8_wait2ndDone;
                gcs().send_text(MAV_SEVERITY_INFO, "state: wait2ndHalf -> wait2ndDone" );
            }
        break;

        case Circle8_wait2ndDone:
            if( Distance < 0.5 )
            {
                Circle8State = Circle8_wait1stHalf;

                Location NewCenter( Circle8CenterPos, LocBeginC8.get_alt_frame() );
                circle_nav.set_center( NewCenter );

                gcs().send_text(MAV_SEVERITY_INFO, "state: wait1stdone -> wait1stHalf" );
            }
        break;

        default:
        break;

    }

}
#endif

    // process pilot inputs
    // get pilot's desired yaw rate
    target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
    if (!is_zero(target_yaw_rate)) {
        circle_pilot_yaw_override = true;
    }

    // get pilot desired climb rate
    target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());

    // set motors to full range
    motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // run circle controller
    failsafe_terrain_set_status(circle_nav.update());

    ///////////////////////
    // update xy outputs //

    float lateral_out, forward_out;
    translate_circle_nav_rp(lateral_out, forward_out);

    // Send to forward/lateral outputs
    motors.set_lateral(lateral_out);
    motors.set_forward(forward_out);

    // call attitude controller
    if (circle_pilot_yaw_override) {
        attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_yaw_rate);
    } else {
        attitude_control.input_euler_angle_roll_pitch_yaw(channel_roll->get_control_in(), channel_pitch->get_control_in(), circle_nav.get_yaw(), true);
    }

    // update altitude target and call position controller
    pos_control.set_pos_target_z_from_climb_rate_cm(target_climb_rate);
    pos_control.update_z_controller();
}
