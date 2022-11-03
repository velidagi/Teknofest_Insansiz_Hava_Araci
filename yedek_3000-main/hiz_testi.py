from dronekit import Command, connect, VehicleMode, LocationGlobalRelative
import time
from pymavlink import mavutil



connection_string="127.0.0.1:14550"

iha=connect(connection_string,wait_ready=True,timeout=100)

def arm_ol_ve_yuksel(hedef_yukseklik):
	while iha.is_armable==False:
		print("Arm ici gerekli sartlar saglanamadi.")
		time.sleep(1)
	print("Iha su anda armedilebilir")
	
	iha.mode=VehicleMode("GUIDED")
	while iha.mode=='GUIDED':
		print('Guided moduna gecis yapiliyor')
		time.sleep(1.5)

	print("Guided moduna gecis yapildi")
	iha.armed=True
	while iha.armed is False:
		print("Arm icin bekleniliyor")
		time.sleep(1)

	print("Ihamiz arm olmustur")
	
	iha.simple_takeoff(hedef_yukseklik)
	while iha.location.global_relative_frame.alt<=hedef_yukseklik*0.94:
		print("Su anki yukseklik{}".format(iha.location.global_relative_frame.alt))
		time.sleep(0.5)
	print("Takeoff gerceklesti")

waypoint1_x = -35.36175960
waypoint1_y = 149.16501012

waypoint2_x = -35.36088375
waypoint2_y = 149.16400500

waypoint3_x = -35.36193926
waypoint3_y = 149.16293103

waypoint4_x = -35.36360337 
waypoint4_y = 149.16295587

waypoint5_x = -35.36449041
waypoint5_y = 149.16407114

waypoint6_x = -35.3632622
waypoint6_y = 149.1652384

def gorev_ekle():

    global komut
    komut = iha.commands
    komut.clear()
    time.sleep(1)

    komut.add(Command(0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 
    mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,0,0,0,0,0,0,0,0,14))
    #speed 20
    komut.add(Command(0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 
    mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,0,0,0,40,0,0,0,0,0))
    #waypoint 2
    komut.add(Command(0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 
    mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,0,0,0,0,0,0,waypoint1_x,waypoint1_y,20))
    #waypoint 3
    komut.add(Command(0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 
    mavutil.mavlink.MAV_CMD_NAV_SPLINE_WAYPOINT,0,0,0,0,0,0,waypoint2_x,waypoint2_y,20))
    #waypoint 4
    komut.add(Command(0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 
    mavutil.mavlink.MAV_CMD_NAV_SPLINE_WAYPOINT,0,0,0,0,0,0,waypoint3_x,waypoint3_y,20))
    

    komut.add(Command(0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 
    mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,0,0,0,0,0,0, waypoint4_x ,waypoint4_y,20))
    
    komut.add(Command(0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 
    mavutil.mavlink.MAV_CMD_NAV_SPLINE_WAYPOINT,0,0,0,0,0,0, waypoint5_x ,waypoint5_y,20))

    komut.add(Command(0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 
    mavutil.mavlink.MAV_CMD_NAV_SPLINE_WAYPOINT,0,0,0,0,0,0, waypoint6_x ,waypoint6_y,20))

    #RTL
    komut.add(Command(0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 
    mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,0,0,0,0,0,0,0,0,0))


    #komut.add(Command(0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 
    #mavutil.mavlink.MAV_CMD_NAV_LAND,0,0,0,0,0,0,-35.36279566,149.16513097,0))

    komut.upload()
    print("comands uploading..")

arm_ol_ve_yuksel(15)

gorev_ekle()

iha.mode = VehicleMode("AUTO")


while True:
    
    print(iha.location.global_relative_frame)
    print(" Heading: %s" % iha.heading)
    print (iha.attitude)
    time.sleep(2.5)