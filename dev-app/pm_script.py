from pymavlink import mavutil
import time

# 接続
master = mavutil.mavlink_connection("tcp:127.0.0.1:5762")
master.wait_heartbeat() 

# while True:
#     try:
#         print(master.recv_match().to_dict())
#     except:
#         pass
#     time.sleep(1.0)


# # モード
# mode = 'STABILIZE'

# mode_id = master.mode_mapping()[ mode ]

# master.mav.set_mode_send( 
#     master.target_system,
#     mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
#     mode_id
# )

# # ARM
# master.arducopter_arm()
# master.motors_armed_wait()
# print("ARMED")

# time.sleep(10)

# #DISARM
# master.arducopter_disarm()
# master.motors_disarmed_wait()
# print("DISARMED")

# パラメータ読み込み
master.mav.param_request_list_send(
    master.target_system, master.target_component
)

while True:
    time.sleep(0.01)
    try:
        message = master.recv_match(type='PARAM_VALUE', blocking=True).to_dict()
        print('name: %s\tvalue: %d' % (message['param_id'],
                                        message[ 'param_value']))
    except Exception as error:
        print(error)
        sys.exit(0)