from dronekit import connect, VehicleMode
from pymavlink import mavutil

import time
import math

vehicle = connect('tcp:127.0.0.1:5762', wait_ready=True, timeout=60 )
print("Connected.")

while not vehicle.home_location:
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()

    if not vehicle.home_location:
        print("Waiting for home location ...")

print("Home location: %s" % vehicle.home_location)
