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


def gorev_1():
    global komut
    komut = iha.commands
    komut.clear()
    time.sleep(1)

    komut.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                      mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 0, 0, 10))

    komut.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                      mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, 0, 0, 0, 20, 0, 0, 0, 0, 0))

    # waypoint
    komut.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                      mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, -35.35699099, 149.16382687, 20))
    # RTL
    komut.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                      mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0, 0, 0, 0, 0, 0, 0))

    komut.upload()
    print("comands uploading..")


arm_ol_ve_yuksel(10)
gorev_1()

iha.mode = VehicleMode("AUTO")
time.sleep(1)

