from dronekit import *
import time
import math
import argparse
import dronekit_sitl
from pymavlink import mavutil
import cv2
import numpy as np


the_connection = mavutil.mavlink_connection('/dev/ttyUSB0')
vehicle = connect('/dev/ttyUSB0', wait_ready=True)


print("Vehicle state:")
print("Global Location: %s" % Vehicle.location.global_frame)
print("Global Location (relative altitude): %s" % Vehicle.location.global_relative_frame)
print("Local Location: %s" % Vehicle.location.local_frame)
print("Attitude: %s" % Vehicle.attitude)
print("Battery: %s" % Vehicle.battery)
print("Last Heartbeat: %s" % Vehicle.last_heartbeat)
print("Heading: %s" % Vehicle.heading)
print("Groundspeed: %s" % Vehicle.groundspeed)
print("Airspeed: %s" % Vehicle.airspeed)
print("Is Armable?: %s" % Vehicle.is_armable)
print("Armed: %s" % Vehicle.armed)
print("Mode: %s" % Vehicle.mode.name)


cap = cv2.VideoCapture(0)
cap.resolution = (640, 480)
cap.framerate = 32
ret, frame = cap.read()
time.sleep(0.5)

color = "undefined"

parser = argparse.ArgumentParser(description="commands")
parser.add_argument("--connect", help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()
connection_sting = args.connect

sitl = None

if not connection_sting:
    sitl = dronekit_sitl.start_default()
    connection_sting = sitl.connection_string()

vehicle = connect(connection_sting, wait_ready=True)
vehicle = connect()


def arm_and_takeoff(aTargetAltitude):
    print("Предполетные проверки")
    while not vehicle.is_armable:
        print("Ждем дрон...")
        time.sleep(1)
    print("Запускаем двигатели")

    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print("Ждем моторы...")
        time.sleep(1)
    print("Взлет!")
    vehicle.simple_takeoff(aTargetAltitude)

    while True:
        print("Текущая высота: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Поднялись на %d метров" % vehicle.location.global_relative_frame.alt)
            break
        time.sleep(1)

arm_and_takeoff(10)
vehicle.airspeed = 3


def image_callback(cap):
    global color, frame, apd
    kernal = np.ones((5, 5), "uint8")
    img_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2_HSV)[119:120, 159:160]
    print(img_hsv[119][159])

    blue_low = (94, 80, 2)
    blue_high = (120, 255, 255)

    blue_mask = cv2.inRange(img_hsv, blue_low, blue_high)
    blue_mask = cv2.dilate(blue_mask, kernal)

    blue_thresh = cv2.inRange(img_hsv, blue_low, blue_high)

    if blue_thresh[0][0] == 255:
        color = "blue"
    else:
        color = "undefined"

    contours = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[0]

    for cont in contours:
        sm = cv2.arcLength(cont, True)
        apd = cv2.approxPolyDP(cont, 0.02 * sm, True)

        if len(apd) == 4:
            cv2.drawContours(frame, [apd], -1, (0, 255, 0), 4)

def recv_match(self, condition=None, type=None, blocking=False, timeout=None):
    msg = the_connection.recv_match(type='SYS_STATUS', condition='SYS_STATUS.mode==2 and SYS_STATUS.nav_mode==4', blocking=True)

def set_velocity_body(vehicle, vx, vy, vz):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000111111000111,  # -- BITMASK
        0, 0, 0,  # -- POSITION
        vx, vy, vz,  # -- VELOCITY
        0, 0, 0,  # -- ACCELERATIONS
        0, 0)
    vehicle.send_mavlink(msg)
    vehicle.flush()


arm_and_takeoff(20)
a_location = LocationGlobalRelative(-34.3254, -78.3546, 20)
vehicle.simple_goto(a_location)
vehicle.groundspeed = 5


def distance_to_current_waypoint():
    nextwaypoint = vehicle.commands.next
    if nextwaypoint == 0:
        return None
    missionitem = vehicle.commands[nextwaypoint-1]
    lat = missionitem.x
    lon = missionitem.y
    alt = missionitem.z
    while not is_arrived(lat, lon, alt):
        time.sleep(3)

def is_arrived(lat, lon, alt, precision=0.3):
    veh_loc = vehicle.location.global_relative_frame
    diff_lat_m = (lat - veh_loc.lat) * 1.113195e5
    diff_lon_m = (lon - veh_loc.lon) * 1.113195e5
    diff_alt_m = alt - veh_loc.alt
    dist_xyz = math.sqrt(diff_lat_m ** 2 + diff_lon_m ** 2 + diff_alt_m ** 2)

    if dist_xyz < precision:
        print("Прибыли на место")
        return True
    else:
        print("Еще не долетели")
        return False

time.sleep(10)

if color == "blue" & len(apd) == 4:
    arm_and_takeoff(0)

print("Returning to Launch")
vehicle.mode = VehicleMode("RTL")

print("Close vehicle object")
vehicle.close()

if sitl is not None:
    sitl.stop()

cv2.imshow("Frame", frame)
cv2.waitKey(0)
