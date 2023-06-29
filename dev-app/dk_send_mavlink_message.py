from dronekit import connect, VehicleMode
from pymavlink import mavutil

import time
import math

vehicle = connect('tcp:127.0.0.1:5762', wait_ready=True, timeout=60 )
print("Connected.")

### 座標指定の移動
vehicle.mode = VehicleMode("GUIDED")
vehicle.wait_for_mode("GUIDED")
print("mode = GUIDED")

vehicle.armed = True
vehicle.arm()
print("ARMED.")

# 離陸
try:
    vehicle.wait_for_armable()
    vehicle.wait_for_mode("GUIDED")
    vehicle.arm()

    time.sleep(1)

    vehicle.wait_simple_takeoff( 10, timeout=20)

except TimeoutError as takeoffError:
    print("Takeoff is timeout.")

######################################################

msg = vehicle.message_factory.command_long_encode(
    0, 1,                                       # ターゲットシステム、コンポーネント
    mavutil.mavlink.MAV_CMD_CONDITION_YAW,      # コマンド
    0,                                          
    90,                                         # param1 : 角度指定(deg)
    0,                                          # param2 : 速度指定(deg/s)
    1,                                          # param3 : 回転方向, -1:CCW, 1:CW
    0,                                          # param4 : 指定, 1:Relative, 0:Absolute
    0, 0, 0)                                    # param5 ～ 7 : CONDITION_YAWの場合未指定

vehicle.send_mavlink( msg )
