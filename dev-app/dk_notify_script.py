from dronekit import connect, VehicleMode
import time

# 他のスクリプトを同時に動かしたいので 5762 ではなく 5763 を使用, 待ちが面倒なので wait～ を false
vehicle = connect('tcp:127.0.0.1:5763', wait_ready=False, timeout=60 )
print("Connected.")

# # コールバック関数
# def location_callback(self, attr_name, value):
#     print("Location(Global):", value)

# # コールバック関数の追加
# vehicle.add_attribute_listener('location.global_frame', location_callback)

# # コールバック関数が解除されるまでの10秒間、コールバック関数のprintが表示される
# time.sleep(10)

# #登録したコールバック関数を解除する
# vehicle.remove_attribute_listener('location.global_frame', location_callback)

@vehicle.on_attribute('location.global_frame')
def location_callback(self, attr_name, value):
    print("Location (Global):", value)

time.sleep(10)
