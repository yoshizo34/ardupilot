from dronekit import connect, VehicleMode
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

# ロール、ピッチ、ヨーからクォータニオンを計算する
def to_quaternion(roll = 0.0, pitch = 0.0, yaw = 0.0):
    """
    Convert degrees to quaternions
    """
    t0 = math.cos(math.radians(yaw * 0.5))
    t1 = math.sin(math.radians(yaw * 0.5))
    t2 = math.cos(math.radians(roll * 0.5))
    t3 = math.sin(math.radians(roll * 0.5))
    t4 = math.cos(math.radians(pitch * 0.5))
    t5 = math.sin(math.radians(pitch * 0.5))

    w = t0 * t2 * t4 + t1 * t3 * t5
    x = t0 * t3 * t4 - t1 * t2 * t5
    y = t0 * t2 * t5 + t1 * t3 * t4
    z = t1 * t2 * t4 - t0 * t3 * t5

    return [w, x, y, z]

# MAVLinkメッセージを生成
msg = vehicle.message_factory.set_attitude_target_encode(
    0,                                          # ブートからの時間（未使用
    0, 0,                                       # ターゲットシステム、コンポーネント  
    0b00000000 if False else 0b00000100,        # ヨーレート
    to_quaternion(20.0, -20.0, 180.0),          # クォータニオン角度
    0, 0, math.radians(0),                      # ヨー、ピッチ、レート
    0.5)                                        # スラスト


for x in range(0, 100):
    vehicle.send_mavlink( msg )
    time.sleep(0.05)

