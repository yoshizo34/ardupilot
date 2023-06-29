from dronekit import connect, VehicleMode
import time

vehicle = connect('tcp:127.0.0.1:5762', wait_ready=True, timeout=60 )
print("Connected.")

# 非同期
# vehicle.mode = VehicleMode("GUIDED")
# vehicle.armed = True
# vehicle.groundspeed = 3.2


# 同期
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
