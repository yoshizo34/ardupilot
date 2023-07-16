#include "Sub.h"

/*
 * Init and run calls for survey flight mode
 */



// survey_init - initialise survey controller
bool Sub::survey_init(bool ignore_checks)
{
    if (!position_ok() && !ignore_checks) {
        return false;
    }

    // initialize speeds and accelerations
    pos_control.set_max_speed_accel_xy(wp_nav.get_default_speed_xy(), wp_nav.get_wp_acceleration());
    pos_control.set_correction_speed_accel_xy(wp_nav.get_default_speed_xy(), wp_nav.get_wp_acceleration());
    pos_control.set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    pos_control.set_correction_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    pos_control.init_xy_controller();
    pos_control.init_z_controller();
    pos_control.set_pos_target_z_cm(0);
    survey_state = SURVEY_init;
    gcs().send_text( MAV_SEVERITY_DEBUG, "mode survey init." );
    return true;
}

void Sub::survey_run()
{
    // control_survey parameter 
    // g.survey_angle -> same survey_course
    // g.survey_length -> same survey_length
    // g.survey_descent -> interval of depth

    // survey setting
    // survey course -> direction of the vehicle proceed
    // survey length -> distance of the vehicle proceed
    // s_course_rad -> radius of survey course

    
    int survey_course = g.survey_angle; //cdeg
    int survey_length = g.survey_length; //cm
    float s_course_rad = (survey_course/100) * M_PI /180; //rad  

    float target_yaw_rate,dst;
    bool tel_alt;
    
    read_barometer();
    float current_depth = barometer.get_altitude(); //m
    float descent = g.survey_descent; //m

    switch(survey_state){

        case SURVEY_init:
            // set initial position 
            // It set current position to start position in relative position from EKF origin
            wp_nav.get_vector_NEU(current_loc,start_point,tel_alt);
            // It set current depth to start position z
            start_point.z = current_depth;

            // Calculate vehicle heading
            // Vehicle turns to perpendicular direction of survey line
            if(survey_course - 9000 <0){
                vehicle_heading = (survey_course - 9000) + 36000;
            } else{
                vehicle_heading = survey_course - 9000;
            }
            // Read current vehicle heading
            last_pilot_heading = ahrs.yaw_sensor;
            
            // disarm process
            if (!motors.armed()) {
                // To-Do: add some initialisation of position controllers
                motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
                // Sub vehicles do not stabilize roll/pitch/yaw when disarmed
                attitude_control.set_throttle_out(0,true,g.throttle_filt);
                attitude_control.relax_attitude_controllers();
                return;
            }

            // arm process
            gcs().send_text( MAV_SEVERITY_DEBUG, "Turn in the direction of structures." );
            motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

            // Spooling up process
            if (motors.get_spool_state() != AP_Motors::SpoolState::THROTTLE_UNLIMITED) {
                    pos_control.relax_velocity_controller_xy();
                    pos_control.update_xy_controller();
                    pos_control.relax_z_controller(0.0f);  
                    pos_control.update_z_controller();
                    attitude_control.reset_yaw_target_and_rate();
                    attitude_control.reset_rate_controller_I_terms();
                    attitude_control.input_thrust_vector_rate_heading(pos_control.get_thrust_vector(), 0.0);
                    return;
            }

            // set roll and pitch to zero
            // set vehicle course to desirable course
            attitude_control.input_euler_angle_roll_pitch_yaw(0, 0, vehicle_heading, true);

            // 
            if(last_pilot_heading == vehicle_heading){
                wp_nav.wp_and_spline_init();
                target_point.x=start_point.x+(survey_length) * cosf(s_course_rad);
                target_point.y=start_point.y+(survey_length) * sinf(s_course_rad);
                target_point.z=current_depth;

                wp_nav.set_wp_destination(target_point,false);

                gcs().send_text( MAV_SEVERITY_DEBUG, "finish turn. SURVEY start. %d" ,survey_state);
                gcs().send_text( MAV_SEVERITY_DEBUG,"start position x:%f, y:%f, z:%f",start_point.x,start_point.y,start_point.z);
                gcs().send_text( MAV_SEVERITY_DEBUG,"target position x:%f, y:%f, z:%f",target_point.x,target_point.y,target_point.z);
                trip_state = TRIP_outward;
                survey_state = SURVEY_run;
            }

            control_depth();

        break;

        case SURVEY_run:

            wp_nav.update_wpnav();
            float lateral_out,forward_out;
            translate_wpnav_rp(lateral_out, forward_out);      
            motors.set_lateral(lateral_out);
            motors.set_forward(forward_out);
            control_depth();
            dst = wp_nav.get_wp_distance_to_destination();
            target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
            attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(channel_roll->get_control_in(), channel_pitch->get_control_in(),target_yaw_rate);

            if(dst<5.0){
                
                target_depth = current_depth + descent;
                gcs().send_text( MAV_SEVERITY_DEBUG, "reached out way point. Descending start...");
                survey_state = SURVEY_elev;
            }
        break;
        
        case SURVEY_elev:
            

            gcs().send_text( MAV_SEVERITY_DEBUG, "Descending. current depth:%f(m)", current_depth);
            if(target_depth <g.survey_maxdepth){
                gcs().send_text( MAV_SEVERITY_DEBUG, "reached out MAX depth. Finish survey...");
                survey_state = SURVEY_fin;
            }

            if (target_depth < current_depth) {
                pos_control.set_pos_target_z_from_climb_rate_cm(Auto_DIVE_Define_descent_rate);
                pos_control.update_z_controller();
            }else{


                if (trip_state==TRIP_outward){
                    target_point = start_point;
                    target_point.z=target_depth*100;
                    trip_state=TRIP_return;
                    gcs().send_text( MAV_SEVERITY_DEBUG,"start position x:%f, y:%f, z:%f",start_point.x,start_point.y,start_point.z);
                    gcs().send_text( MAV_SEVERITY_DEBUG, "target point.z:%f",target_point.z);
                

                } else if(trip_state==TRIP_return) {
                    target_point.x=start_point.x + (survey_length) * cosf(s_course_rad);
                    target_point.y=start_point.y +(survey_length) * sinf(s_course_rad);
                    target_point.z=target_depth*100;
                    trip_state=TRIP_outward;
                    gcs().send_text( MAV_SEVERITY_DEBUG, "target point.z:%f",target_point.z);
                    gcs().send_text( MAV_SEVERITY_DEBUG,"start position x:%f, y:%f, z:%f",start_point.x,start_point.y,start_point.z);
                    
                }
                
                wp_nav.set_wp_destination(target_point,false);
                pos_control.set_pos_target_z_cm(target_point.z);
                pos_control.update_z_controller();
                survey_state = SURVEY_run;
                    gcs().send_text( MAV_SEVERITY_DEBUG, "target point.z:%f",target_point.z);
            }


        break;

        case SURVEY_fin:
            attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(channel_roll->get_control_in(), channel_pitch->get_control_in(),channel_yaw->get_control_in());
            control_depth();
        break;
    }

    
}
