import sys
import time

from pymavlink import mavutil

ENDPOINT = sys.argv[1]
TARGET = (10.0, 5.0, -5.0)
HOLD_TOLERANCE_M = 0.5
EKF_POS_HORIZ_ABS = 16


def wait_for(link, kinds, seconds):
    return link.recv_match(type=kinds, blocking=True, timeout=seconds)


def main():
    link = mavutil.mavlink_connection(ENDPOINT, source_system=255)
    link.wait_heartbeat(timeout=120)
    link.mav.request_data_stream_send(
        link.target_system, link.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1
    )
    deadline = time.time() + 180
    fix, ekf_gps = 0, False
    while time.time() < deadline and not (fix >= 3 and ekf_gps):
        msg = wait_for(link, ["GPS_RAW_INT", "EKF_STATUS_REPORT"], 2)
        if msg is None:
            continue
        if msg.get_type() == "GPS_RAW_INT":
            fix = msg.fix_type
        else:
            ekf_gps = bool(msg.flags & EKF_POS_HORIZ_ABS)
    if not (fix >= 3 and ekf_gps):
        print("FAIL: no GPS fix in the EKF")
        return 1
    link.set_mode_apm("GUIDED")
    time.sleep(1)
    armed = False
    for _ in range(30):
        link.mav.command_long_send(
            link.target_system, link.target_component, 400, 0, 1, 21196, 0, 0, 0, 0, 0
        )
        time.sleep(1)
        beat = link.recv_match(type="HEARTBEAT", blocking=True, timeout=3)
        if beat is not None and beat.base_mode & 128:
            armed = True
            break
    if not armed:
        print("FAIL: could not arm")
        return 1
    link.mav.command_long_send(
        link.target_system, link.target_component, 22, 0, 0, 0, 0, 0, 0, 0, 5
    )
    end = time.time() + 25
    altitude = 0.0
    while time.time() < end:
        pos = wait_for(link, ["LOCAL_POSITION_NED"], 2)
        if pos is not None:
            altitude = -pos.z
    if abs(altitude - 5.0) > HOLD_TOLERANCE_M:
        print("FAIL: takeoff altitude %.2f" % altitude)
        return 1
    end = time.time() + 35
    while time.time() < end:
        link.mav.set_position_target_local_ned_send(
            0, link.target_system, link.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, 0b110111111000,
            TARGET[0], TARGET[1], TARGET[2], 0, 0, 0, 0, 0, 0, 0, 0,
        )
        pos = wait_for(link, ["LOCAL_POSITION_NED"], 2)
        time.sleep(0.2)
    error = ((pos.x - TARGET[0]) ** 2 + (pos.y - TARGET[1]) ** 2 + (pos.z - TARGET[2]) ** 2) ** 0.5
    print("waypoint error %.2f m" % error)
    if error > HOLD_TOLERANCE_M:
        print("FAIL: waypoint error")
        return 1
    link.set_mode_apm("LAND")
    end = time.time() + 90
    landed = False
    while time.time() < end and not landed:
        pos = wait_for(link, ["LOCAL_POSITION_NED"], 2)
        beat = link.recv_match(type="HEARTBEAT", blocking=False)
        if beat is not None and not (beat.base_mode & 128):
            landed = True
    print("landed %s altitude %.2f" % (landed, -pos.z))
    if not landed:
        print("FAIL: did not land")
        return 1
    print("PASS")
    return 0


sys.exit(main())
