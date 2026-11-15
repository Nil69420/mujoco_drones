import math
import sys
import time

from pymavlink import mavutil

TARGET_NORTH_M = 10.0
TARGET_EAST_M = 5.0
TAKEOFF_M = 5.0
TOLERANCE_M = 1.0
EARTH_RADIUS_M = 6378137.0
EKF_POS_HORIZ_ABS = 16
CMD_ARM = 400
CMD_TAKEOFF = 22
CMD_LAND = 21
CMD_REPOSITION = 192


def wait_for(link, kinds, seconds):
    return link.recv_match(type=kinds, blocking=True, timeout=seconds)


def latest(link, kind, seconds=2.0):
    msg = None
    end = time.time() + seconds
    while time.time() < end:
        got = link.recv_match(type=kind, blocking=True, timeout=0.5)
        if got is not None:
            msg = got
    return msg


def main():
    link = mavutil.mavlink_connection(sys.argv[1], source_system=255)
    link.wait_heartbeat(timeout=180)
    deadline = time.time() + 240
    ready = False
    last_flags = None
    while time.time() < deadline and not ready:
        msg = wait_for(link, ["ESTIMATOR_STATUS"], 2)
        if msg is not None:
            last_flags = msg.flags
        ready = msg is not None and bool(msg.flags & EKF_POS_HORIZ_ABS)
    if not ready:
        print("FAIL: estimator never got a global position (last estimator flags %s)" % last_flags)
        return 1
    home = latest(link, "HOME_POSITION", 3.0)
    if home is None:
        print("FAIL: no home position")
        return 1
    home_lat, home_lon, home_alt = home.latitude / 1e7, home.longitude / 1e7, home.altitude / 1e3
    armed = False
    for _ in range(40):
        link.mav.command_long_send(link.target_system, link.target_component, CMD_ARM, 0, 1, 0, 0, 0, 0, 0, 0)
        time.sleep(1.0)
        beat = latest(link, "HEARTBEAT", 1.5)
        if beat is not None and beat.base_mode & 128:
            armed = True
            break
    if not armed:
        print("FAIL: could not arm")
        return 1
    link.mav.command_long_send(
        link.target_system, link.target_component, CMD_TAKEOFF, 0, 0, 0, 0, float("nan"),
        home_lat, home_lon, home_alt + TAKEOFF_M,
    )
    time.sleep(20)
    pos = latest(link, "LOCAL_POSITION_NED", 1.5)
    altitude = -pos.z
    print("takeoff altitude %.2f" % altitude)
    if abs(altitude - TAKEOFF_M) > TOLERANCE_M:
        print("FAIL: takeoff altitude")
        return 1
    lat = home_lat + math.degrees(TARGET_NORTH_M / EARTH_RADIUS_M)
    lon = home_lon + math.degrees(TARGET_EAST_M / (EARTH_RADIUS_M * math.cos(math.radians(home_lat))))
    link.mav.command_int_send(
        link.target_system, link.target_component, mavutil.mavlink.MAV_FRAME_GLOBAL, CMD_REPOSITION, 0, 0,
        -1, 1, 0, float("nan"), int(lat * 1e7), int(lon * 1e7), home_alt + TAKEOFF_M,
    )
    time.sleep(30)
    pos = latest(link, "LOCAL_POSITION_NED", 1.5)
    error = math.sqrt((pos.x - TARGET_NORTH_M) ** 2 + (pos.y - TARGET_EAST_M) ** 2 + (-pos.z - TAKEOFF_M) ** 2)
    print("waypoint error %.2f m" % error)
    if error > TOLERANCE_M:
        print("FAIL: waypoint error")
        return 1
    link.mav.command_long_send(link.target_system, link.target_component, CMD_LAND, 0, 0, 0, 0, 0, 0, 0, 0)
    end = time.time() + 90
    landed = False
    while time.time() < end and not landed:
        pos = latest(link, "LOCAL_POSITION_NED", 1.0)
        beat = latest(link, "HEARTBEAT", 1.0)
        landed = beat is not None and not (beat.base_mode & 128)
    print("landed %s altitude %.2f" % (landed, -pos.z))
    if not landed:
        print("FAIL: did not land")
        return 1
    print("PASS")
    return 0


sys.exit(main())
