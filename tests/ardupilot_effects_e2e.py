import statistics
import sys
import time

from pymavlink import mavutil

ENDPOINT = sys.argv[1]
EKF_POS_HORIZ_ABS = 16


def main():
    link = mavutil.mavlink_connection(ENDPOINT, source_system=255)
    link.wait_heartbeat(timeout=120)
    link.mav.request_data_stream_send(
        link.target_system, link.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1
    )
    deadline = time.time() + 240
    fix, ekf_gps = 0, False
    while time.time() < deadline and not (fix >= 3 and ekf_gps):
        msg = link.recv_match(type=["GPS_RAW_INT", "EKF_STATUS_REPORT"], blocking=True, timeout=2)
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
    end = time.time() + 20
    while time.time() < end:
        link.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=2)
    zacc, altitude = [], []
    end = time.time() + 20
    while time.time() < end:
        msg = link.recv_match(type=["RAW_IMU", "LOCAL_POSITION_NED"], blocking=True, timeout=2)
        if msg is None:
            continue
        if msg.get_type() == "RAW_IMU":
            zacc.append(msg.zacc)
        else:
            altitude.append(-msg.z)
    if len(zacc) < 100 or len(altitude) < 50:
        print("FAIL: too few samples")
        return 1
    print("HOVER zacc_mg %.1f alt_m %.3f alt_std_m %.4f" % (
        statistics.mean(zacc), statistics.mean(altitude), statistics.pstdev(altitude)))
    return 0


sys.exit(main())
