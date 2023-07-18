#include "Sub.h"

/*
 * control_circle_descent - init and run calls for circle descent flight mode
 */

// circle_descent_init - initialise circle descent controller flight mode
bool ModeCircling_Descent::init(bool ignore_checks){

        if (!sub.position_ok()) {
        return false;
    }

    sub.circle_pilot_yaw_override = false;

    // initialize speeds and accelerations
    position_control->set_max_speed_accel_xy(sub.wp_nav.get_default_speed_xy(), sub.wp_nav.get_wp_acceleration());
    position_control->set_correction_speed_accel_xy(sub.wp_nav.get_default_speed_xy(), sub.wp_nav.get_wp_acceleration());
    position_control->set_max_speed_accel_z(-sub.get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    position_control->set_correction_speed_accel_z(-sub.get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // initialise circle controller including setting the circle center based on vehicle speed
    sub.circle_nav.init();

    return true;
}

// circle_run - runs the circle flight mode
// should be called at 100hz or more
void ModeCircling_Descent::run()
{   
    float target_yaw_rate = 0;
    float lateral_out, forward_out;
    bool tel_alt;
    float current_depth = sub.barometer.get_altitude()*100;

    
    switch(circle_descent_state){
        case CIRCLE_DESCENT_init:

            gcs().send_text( MAV_SEVERITY_DEBUG, "Initialise circle descent mode");
            sub.wp_nav.get_vector_NEU(sub.current_loc,CircleStartPoint,tel_alt);

            // update parameters, to allow changing at runtime
            position_control->set_max_speed_accel_xy(sub.wp_nav.get_default_speed_xy(), sub.wp_nav.get_wp_acceleration());
            position_control->set_max_speed_accel_z(-sub.get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
            circle_rate = g.c_descent_rate;
            // set to next state
            sub.circle_nav.set_rate(circle_rate);
            sub.circle_nav.set_radius_cm(g.c_descent_radius);
            circle_descent_state = CIRCLE_DESCENT_run;
            gcs().send_text( MAV_SEVERITY_DEBUG, "START CIRCLE FLY");
        break;

        case CIRCLE_DESCENT_run:
            start_heading = ahrs.yaw_sensor;
            // if not armed set throttle to zero and exit immediately
            if (!motors.armed()) {
            // To-Do: add some initialisation of position controllers
                motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
                // Sub vehicles do not stabilize roll/pitch/yaw when disarmed
                attitude_control->set_throttle_out(0,true,g.throttle_filt);
                attitude_control->relax_attitude_controllers();
                sub.circle_nav.init();
                return;
            }

            // process pilot inputs
            // get pilot's desired yaw rate
            target_yaw_rate = sub.get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
            if (!is_zero(target_yaw_rate)) {
                sub.circle_pilot_yaw_override = true;
            }

            // set motors to full range
            motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

            // run circle controller
            sub.failsafe_terrain_set_status(sub.circle_nav.update());
            CircleCenterPoint=sub.circle_nav.get_center();



            ///////////////////////
            // update xy outputs //
            sub.translate_circle_nav_rp(lateral_out, forward_out);

            // Send to forward/lateral outputs
            motors.set_lateral(lateral_out);
            motors.set_forward(forward_out);

            // call attitude controller
            if (sub.circle_pilot_yaw_override) {
                attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_yaw_rate);
            } else {
                attitude_control->input_euler_angle_roll_pitch_yaw(channel_roll->get_control_in(), channel_pitch->get_control_in(), sub.circle_nav.get_yaw(), true);
            }

            // update altitude target and call position controller
            position_control->set_pos_target_z_from_climb_rate_cm(0.0f);
            position_control->update_z_controller();
            //for debug
            //gcs().send_text( MAV_SEVERITY_DEBUG, "angle_total:%f",sub.circle_nav.get_angle_total());
            if (sub.circle_nav.get_angle_total() > 2*M_PI || sub.circle_nav.get_angle_total() < -2*M_PI){
                gcs().send_text( MAV_SEVERITY_DEBUG, "FIN CIRCLE FLY DESCENDING");
                circle_descent_state = CIRCLE_DESCENT_elev;
                descent_z = current_depth+g.c_descent_pitch;
                gcs().send_text(MAV_SEVERITY_DEBUG, "BAROMETER:%fcm",current_depth);
                position_control->set_pos_target_z_cm(descent_z);
            }

        break;

        case CIRCLE_DESCENT_elev:
            gcs().send_text(MAV_SEVERITY_DEBUG, "BAROMETER:%fcm, target depth:%fcm",current_depth,descent_z);
            attitude_control->input_euler_angle_roll_pitch_yaw(0,0,start_heading,false);
            position_control->set_pos_target_xy_cm(CircleStartPoint.x,CircleStartPoint.y);
            sub.translate_pos_control_rp(lateral_out, forward_out);
            motors.set_lateral(lateral_out);
            motors.set_forward(forward_out);
            position_control->update_z_controller();

            if (current_depth < descent_z && current_depth > g.c_descent_max){
                circle_rate = circle_rate *-1;
                sub.circle_nav.init(CircleCenterPoint,false,circle_rate);
                circle_descent_state = CIRCLE_DESCENT_run;
            } else if(current_depth < g.c_descent_max){
                gcs().send_text(MAV_SEVERITY_DEBUG, "CURRENT DEPTH:%fm, FINISH FLY.",current_depth);
                circle_descent_state = CIRCLE_DESCENT_fin;
            }
        break;

        case CIRCLE_DESCENT_fin:
            attitude_control->input_euler_angle_roll_pitch_yaw(0,0,start_heading,false);
            position_control->init_xy_controller_stopping_point();
            sub.translate_pos_control_rp(lateral_out, forward_out);
            motors.set_lateral(lateral_out);
            motors.set_forward(forward_out);
            position_control->set_pos_target_z_from_climb_rate_cm(0);
            position_control->update_z_controller();
        break;

    }


}