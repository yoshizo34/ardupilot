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
    pos_control.set_pos_target_z_cm(0);
    survey_state = SURVEY_init;
    gcs().send_text( MAV_SEVERITY_DEBUG, "mode survey init." );
    return true;
}

void Sub::survey_run()
{
    // set to position control mode
    int survey_course = 31500;
    float x_cm = 0.0;
    float y_cm = 300.0;
    // initialise waypoint controller
    switch(survey_state){

        case SURVEY_init:

            last_pilot_heading = ahrs.yaw_sensor;
                if (!motors.armed()) {
                    // To-Do: add some initialisation of position controllers
                    motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
                    // Sub vehicles do not stabilize roll/pitch/yaw when disarmed
                    attitude_control.set_throttle_out(0,true,g.throttle_filt);
                    attitude_control.relax_attitude_controllers();
                    circle_nav.init();
                    return;
                }
                pos_control.init_xy_controller();

            //calculate vehicle heading
            if(survey_course - 9000 <0){
                vehicle_heading = (survey_course - 9000) + 36000;
            } else{
                vehicle_heading = survey_course - 9000;
            }
            gcs().send_text( MAV_SEVERITY_DEBUG, "Turn in the direction of structures." );
            motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
            attitude_control.input_euler_angle_roll_pitch_yaw(0, 0, vehicle_heading, true);
            if(last_pilot_heading == vehicle_heading){
                survey_state = SURVEY_run;
            }
            control_depth();
        break;

        case SURVEY_run:

            gcs().send_text( MAV_SEVERITY_DEBUG, "finis turn. Survey start." );

        break;
        case SURVEY_elev:
        break;
        case SURVEY_fin:
        break;



    }

    attitude_control.input_euler_angle_roll_pitch_yaw(0, 0, vehicle_heading, true);
    pos_control.set_pos_target_xy_cm(x_cm,y_cm);
    float lateral_out,forward_out;
    translate_pos_control_rp(lateral_out,forward_out);
    motors.set_lateral(lateral_out);
    motors.set_forward(forward_out);
}
