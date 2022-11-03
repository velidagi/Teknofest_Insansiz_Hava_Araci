from doctest import FAIL_FAST
from dronekit import Command, connect, VehicleMode, LocationGlobalRelative
import time
from pymavlink import mavutil

connection_string = "127.0.0.1:14550"

iha = connect(connection_string, wait_ready=True, timeout=100)


def arm_ol_ve_yuksel(hedef_yukseklik):
    while iha.is_armable == False:
        print("Arm ici gerekli sartlar saglanamadi.")
        time.sleep(1)
    print("Iha su anda armedilebilir")

    iha.mode = VehicleMode("GUIDED")
    while iha.mode == 'GUIDED':
        print('Guided moduna gecis yapiliyor')
        time.sleep(1.5)

    print("Guided moduna gecis yapildi")
    iha.armed = True
    while iha.armed is False:
        print("Arm icin bekleniliyor")
        time.sleep(1)

    print("Ihamiz arm olmustur")

    iha.simple_takeoff(hedef_yukseklik)
    while iha.location.global_relative_frame.alt <= hedef_yukseklik * 0.94:
        print("Su anki yukseklik{}".format(iha.location.global_relative_frame.alt))
        time.sleep(0.5)
    print("Takeoff gerceklesti")


def condition_yaw(heading, relative=False):
    if relative:
        is_relative = 1  # yaw relative to direction of travel
    else:
        is_relative = 0  # yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = iha.message_factory.command_long_encode(
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # command
        0,  # confirmation
        heading,  # param 1, yaw in degrees
        0,  # param 2, yaw speed deg/s
        1,  # param 3, direction -1 ccw, 1 cw
        is_relative,  # param 4, relative offset 1, absolute angle 0
        0, 0, 0)  # param 5 ~ 7 not used
    # send command to vehicle
    iha.send_mavlink(msg)


def goto_position_target_relative_ned(north, east, down=0):
    msg = iha.message_factory.set_position_target_local_ned_encode(
        0,  # time_boot_ms (not used)
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,  # frame
        0b0000111111111000,  # type_mask (only positions enabled)
        north, east, down,
        0, 0, 0,  # x, y, z velocity in m/s  (not used)
        0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    # send command to iha
    iha.send_mavlink(msg)


condition_yaw(0, relative=True)


def ileri():
    goto_position_target_relative_ned(10, 0)
    condition_yaw(0, relative=True)


def geri():
    goto_position_target_relative_ned(-10, 0)
    condition_yaw(0, relative=True)


def sag():
    goto_position_target_relative_ned(0, 10)
    condition_yaw(0, relative=True)


def sol():
    goto_position_target_relative_ned(0, -10)
    condition_yaw(0, relative=True)


arm_ol_ve_yuksel(15)

a_location = LocationGlobalRelative(-35.35699099, 149.16382687, 10)

iha.simple_goto(a_location, airspeed=5.0)

time.sleep(15)
iha.airspeed = 2
time.sleep(5)
curentgps = iha.location.global_frame

iha.airspeed = 2
iha.simple_goto(curentgps, airspeed=0.5)

time.sleep(5)
sag()
time.sleep(4)
iha.airspeed = 10
sag()
time.sleep(4)
iha.airspeed = 1
sag()
time.sleep(4)

ileri()
time.sleep(4)

geri()
time.sleep(4)

iha.mode = VehicleMode("RTL")