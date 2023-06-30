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

# MAVLink メッセージを生成
msg = vehicle.message_factory.set_position_target_local_ned_encode(
    0,                                          # ブートからの時間（未使用
    0, 0,                                       # ターゲットシステム、コンポーネント
    mavutil.mavlink.MAV_FRAME_LOCAL_NED,        # フレーム
    0b0000111111000111,                         # タイプマスク
    0, 0, 0,                                    # X, Y, Z 位置（未使用
    1.0, 1.0, -0.2,                             # 速度 (m/s)
    0, 0, 0,                                    # X, Y, Z 加速度（未サポート
    0, 0)                                       # ヨー、ヨーレート

for x in range( 0, 100):
    vehicle.send_mavlink(msg)
    time.sleep(1)