from dronekit import Command, connect, VehicleMode, LocationGlobalRelative
import time
from pymavlink import mavutil
import numpy as np
import cv2
import time
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

arm_ol_ve_yuksel(15)
def condition_yaw(heading, relative=False):
    if relative:
        is_relative=1 #yaw relative to direction of travel
    else:
        is_relative=0 #yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = iha.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        1,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    iha.send_mavlink(msg)

def goto_position_target_relative_ned(north, east, down=0):

    msg = iha.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, # frame
        0b0000111111111000, # type_mask (only positions enabled)
        north, east, down,
        0, 0, 0, # x, y, z velocity in m/s  (not used)
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    # send command to iha
    iha.send_mavlink(msg)
def ileri():
    goto_position_target_relative_ned(5, 0)
    condition_yaw(0, relative=True)
    
def geri():
    goto_position_target_relative_ned(-5, 0)
    condition_yaw(0, relative=True)
    

def sag():
    goto_position_target_relative_ned(0, 5)
    condition_yaw(0, relative=True)
    

def sol():
    goto_position_target_relative_ned(0, -5)
    condition_yaw(0, relative=True)
    
def alcal():
    goto_position_target_relative_ned(0, 0,15)
    condition_yaw(0, relative=True)
    

def baslat():
	
	cap = cv2.VideoCapture(0)
	prev_frame_time = 0
	new_frame_time = 0
	z = 0
	while True:

		# Goruntuyu frame olarak tek tek okuma
		ret,frame=cap.read()
		vertical ,horizontal,temp= frame.shape
		# Goruntunun Merkezini aliyoruz
		orgin_x = horizontal // 2
		orgin_y = vertical // 2
		# Goruntunun Merkezine Nokta Koyuyoruz
		cv2.circle(frame, (orgin_x, orgin_y), radius=5, color=(0, 0, 0), thickness=-1)

		hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
		#lower red de hepsinin 114 oldugu bir ihtimal de vardi sanki
		lower_red = np.array([114, 113, 144], np.uint8)
		upper_red = np.array([180, 255, 255], np.uint8)
		"""lower_red = np.array([114, 71, 133], np.uint8)
		upper_red = np.array([180, 255, 255], np.uint8)"""
		"""lower_red = np.array([0, 50, 50], np.uint8)
		upper_red = np.array( [5, 255, 255], np.uint8)"""
		mask = cv2.inRange(hsv, lower_red, upper_red)
		red = cv2.bitwise_and(frame,frame,mask=mask)
		masked = cv2.medianBlur(mask,9)
		circles = cv2.HoughCircles(masked, cv2.HOUGH_GRADIENT, 3, frame.shape[0] / 0.1, param1=220, param2=20,minRadius=80,maxRadius=500)
		if circles is not None:
			circles = np.uint16(np.around(circles))

			for i in circles[0, :]:
				circle_x = i[0]
				circle_y = i[1]
				cv2.circle(frame, (i[0], i[1]), i[2], (255, 255, 0), 2)
				cv2.circle(frame, (i[0], i[1]), 2, (255, 255, 0), 2)
				#print("x=",i[0],"y=",i[1])

			# IHA'ya bir yonde ucurmak icin ornek olarak alacagimiz oranlamalar
			left_ = (circle_x - orgin_x) / 20
			right_ = (circle_x - orgin_x) / 20
			forward_ = (orgin_y - circle_y) / 20
			back_ = (circle_y - orgin_y) / 20

			if ((100 > orgin_x - circle_x > 0) and (100 > circle_y - orgin_y > 0)):
				z = z + 1
				if (z == 10):
                	print("Alçal")
                	z = 0
			elif ((orgin_x - circle_x > 0) and (circle_y - orgin_y > 0)):
				z = z + 1
				if (z == 100):
					sol()
					geri()
					z = 0
			elif ((orgin_x - circle_x > 0) and (circle_y - orgin_y < 0)):
				z = z + 1
				if (z == 100):
					sol()
					ileri()
					z = 0
			elif ((orgin_x - circle_x < 0) and (circle_y - orgin_y > 0)):
				z = z + 1
				if (z == 100):
					sag()
					geri()
					z = 0
			elif ((orgin_x - circle_x < 0) and (circle_y - orgin_y < 0)):
				z = z + 1
				if (z == 100):
					sag()
					ileri()
					z = 0

		"""if((10>orgin_x - circle_x > 0) and (10>circle_y - orgin_y > 0) ):
				print("Alcal")
			elif ((orgin_x - circle_x > 0) and (circle_y - orgin_y > 0)):
				print(str(left_)+" Sol ve " + str(back_)+   " Geriye git")
			elif ((orgin_x - circle_x > 0) and (circle_y - orgin_y < 0)):
				print(str(left_)+" Sol ve "+str(forward_) + " ileri git")
			elif ((orgin_x - circle_x < 0) and (circle_y - orgin_y > 0)):
				print(str(right_)+" Sag ve " + str(back_)+ " geriye git")
			elif ((orgin_x - circle_x < 0) and (circle_y - orgin_y < 0)):
				print( str(right_)+ " Sag ve " +str(forward_)+" ileri git")"""

		"""	# Roll-Pitch yapmasi icin IHA'nin goruntu merkez koordinatlari ile cemberin merkez koordinatlari arasindaki pixel uzakligi
			if(orgin_x-circle_x>0):
				print("Sola dogru "+ str(orgin_x-circle_x) + " pixel ilerle")
				if(circle_y-orgin_y>0):
					print("Asagi dogru " + str(circle_y-orgin_y) + " pixel ilerle")
				elif(circle_y-orgin_y<0):
					print("Yukari dogru " + str(orgin_y-circle_y) + " pixel ilerle")
			elif (orgin_x - circle_x < 0):
				print("Saga dogru " + str(orgin_x - circle_x) + " pixel ilerle")
				if(circle_y-orgin_y>0):
					print("Asagi dogru " + str(circle_y-orgin_y) + " pixel ilerle")
				elif(circle_y-orgin_y<0):
					print("Yukari dogru " + str(orgin_y-circle_y) + " pixel ilerle")
			elif(orgin_x == circle_x or orgin_y == circle_y):
				print('ihaniz tam kirmizi noktanin merkezi uzerinde ')"""

		# FPS ekrana yazdirilacak font
		font = cv2.FONT_HERSHEY_SIMPLEX
		# frame saymayi bitirecegimiz kisim
		new_frame_time = time.time()

		fps = 1/(new_frame_time-prev_frame_time)
		prev_frame_time = new_frame_time

		# fps degerini integer degiskenine cevirme
		fps = int(fps)
		fps = str(fps)

		# FPS degerlerini ekrana yansitma
		cv2.putText(red, fps, (7, 70), font, 3, (100, 255, 0), 3, cv2.LINE_AA)

		# Goruntuyu pencerede acmak icin
		cv2.imshow("red",red)
		cv2.imshow('frame', frame)

		# cikis icin q tusuna basmak icin
		if cv2.waitKey(1) & 0xFF == ord('q'):
			break

	cap.release()
	# Pencereleri Yok etme
	cv2.destroyAllWindows()
baslat()


