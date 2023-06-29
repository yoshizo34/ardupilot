from dronekit import connect, VehicleMode, LocationGlobalRelative
import time

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

# 目標の緯度経度、高度設定
aLocation = LocationGlobalRelative( -34.364114, 149.166022, 20)

# 移動
vehicle.simple_goto( aLocation )
time.sleep(100)


# # ホームロケーション
# # vehicle.home_location に値がセットされるまで download を繰り返す
# while not vehicle.home_location:
#     cmds = vehicle.commands
#     cmds.download()
#     cmds.wait_ready()

#     if not vehicle.home_location:
#         print("Waiting for home location ...")

# ホームロケーションの取得官僚
print("Home location: %s" % vehicle.home_location )

# # RTL時の高さパラメータ
# print("Param: %s" % vehicle.parameters['RTL_ALT'])

# # RTLパラメータのセット
# vehicle.parameters['RTL_ALT'] = 1500

# # RTL時の高さパラメータ
# print("Param: %s" % vehicle.parameters['RTL_ALT'])

# # パラメータ全表示
# for key, value in vehicle.parameters.items():
#     print("Key:%s, Values:%s" % (key, value))

