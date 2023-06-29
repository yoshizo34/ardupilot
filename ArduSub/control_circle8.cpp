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

    gcs().send_text( MAV_SEVERITY_DEBUG, "mode circle8 init." );

    circle_pilot_yaw_override = false;

    // initialize speeds and accelerations
    pos_control.set_max_speed_accel_xy(wp_nav.get_default_speed_xy(), wp_nav.get_wp_acceleration());
    pos_control.set_correction_speed_accel_xy(wp_nav.get_default_speed_xy(), wp_nav.get_wp_acceleration());
    pos_control.set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    pos_control.set_correction_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // initialise circle controller including setting the circle center based on vehicle speed
    circle_nav.init();

    // current location saveed.
    LocBeginC8          = current_loc;      //  始点の座標を現在の座標で初期化
    Circle8State        = Circle8LastState = Circle8_init;     //  ステートマシンを初期状態へ

    Circle8CenterPos1st = circle_nav.get_center();      //  1つ目の円の中心をナビから取得
    Circle8CenterPos2nd = Circle8CenterPos1st;          //  ２つ目の円の中心は１つ目の円の中心をヨー方向に180°回転
    Circle8CenterPos2nd.rotate( ROTATION_YAW_180 );

    Circle8Radius = circle_nav.get_radius() / 100.0;    //  cmなのでmに変換
    Circle8SetClimb_rate = 0;       //  上昇率をクリア

    gcs().send_text( MAV_SEVERITY_DEBUG, "1st center: ( %f, %f )",  Circle8CenterPos1st.x, Circle8CenterPos1st.y );
    gcs().send_text( MAV_SEVERITY_DEBUG, "2nd center: ( %f, %f )",  Circle8CenterPos2nd.x, Circle8CenterPos2nd.y );
    gcs().send_text( MAV_SEVERITY_DEBUG, "radius: %f(cm), %f(m)",   circle_nav.get_radius(), Circle8Radius );
    gcs().send_text( MAV_SEVERITY_DEBUG, "distance judge: %f(m)",   Circle8_Define_DistanceJudge );
    gcs().send_text( MAV_SEVERITY_DEBUG, "climb rate: %f(cm/s)",    Circle8_Define_Crimb_rate );

    return true;
}

// circle8_run - runs the circle8 flight mode
// should be called at 100hz or more
void Sub::circle8_run()
{
    float target_yaw_rate = 0;
    // float target_climb_rate = 0;

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

    float   Distance    = ( float ) current_loc.get_distance( LocBeginC8 );     //  始点の座標からの距離を取得(m)
    Vector3f DistanceNED = current_loc.get_distance_NED( LocBeginC8 );      //  始点座標からのNEDを取得

    switch( Circle8State ){
        case Circle8_init:      //  初期処理
            Circle8State = Circle8_wait1stHalf;     //  次のステート
            Circle8SetClimb_rate = Circle8_Define_Crimb_rate * -1.0;       //  定義値で下降
        break;

        case Circle8_wait1stHalf:       //  初期位置～1つ目の円の半分まで待ち。
            if( Distance >  ( Circle8Radius * 2.0 ) - Circle8_Define_DistanceJudge )     //  初期位置から2r離れたので折り返し地点と判断
            {
                Circle8State = Circle8_wait1stDone;     //  次のステート
                Circle8SetClimb_rate = Circle8_Define_Crimb_rate;       //  定義値で上昇
            }
        break;

        case Circle8_wait1stDone:       //  １つ目の円が終わるまで待ち。
            if( Distance < Circle8_Define_DistanceJudge )        //  初期位置にほぼ入れば到達。
            {
                Circle8State = Circle8_wait2ndHalf;     //  次のステート
                Circle8SetClimb_rate = Circle8_Define_Crimb_rate * -1.0;       //  定義値で下降

                //  ナビを2つ目の円の中心点で初期化、回転方向を逆に
                circle_nav.init( Circle8CenterPos2nd, circle_nav.center_is_terrain_alt(), circle_nav.get_rate_current() * -1.0 );
             }
        break;

        case Circle8_wait2ndHalf:       //  ２つ目の円の初期位置～円の半分まで待ち
            if( Distance > ( Circle8Radius * 2.0 ) - Circle8_Define_DistanceJudge )      //  初期位置から2r離れたので折り返し地点
            {
                Circle8State = Circle8_wait2ndDone;     //  次のステート
                Circle8SetClimb_rate = Circle8_Define_Crimb_rate;       //  定義値で上昇
             }
        break;

        case Circle8_wait2ndDone:       //  ２つ目の円が終わるまで待ち
            if( Distance < Circle8_Define_DistanceJudge )        //  初期位置に到達
            {
                Circle8State = Circle8_wait1stHalf;     //  次のステート

                Circle8SetClimb_rate = Circle8_Define_Crimb_rate * -1.0;       //  定義値で下降

                //  ナビを１つ目の円の中心点で初期化、回転方向を逆に
                circle_nav.init( Circle8CenterPos1st, circle_nav.center_is_terrain_alt(), circle_nav.get_rate_current() * -1.0 );
            }
        break;

        default:    //  enumなのでここには来ない…はず
        break;
    }

    if( Circle8LastState != Circle8State )      //  前回とステートが違う
    {
        gcs().send_text( MAV_SEVERITY_DEBUG, "state: %d -> %d, distance=%f, depth=%f", 
                Circle8LastState, Circle8State, Distance, DistanceNED.z );
    }

    Circle8LastState = Circle8State;    //  前回ステート更新

    // process pilot inputs
    // get pilot's desired yaw rate
    target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
    if (!is_zero(target_yaw_rate)) {
        circle_pilot_yaw_override = true;
    }

    // get pilot desired climb rate
    // target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());

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
    pos_control.set_pos_target_z_from_climb_rate_cm( Circle8SetClimb_rate );
    pos_control.update_z_controller();
}
