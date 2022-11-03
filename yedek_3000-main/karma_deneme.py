from dronekit import Command, connect, VehicleMode, LocationGlobalRelative
import time
from pymavlink import mavutil
import numpy as np
import cv2
import time
connection_string="127.0.0.1:14550"


iha=connect(connection_string,wait_ready=True,timeout=100)
global toplamx
global toplamy
global toplampx

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


def ileri(absdeger):
    goto_position_target_relative_ned(absdeger/5, 0)
    condition_yaw(0, relative=True)
    
def geri(absdeger):
    goto_position_target_relative_ned(-(absdeger/5), 0)
    condition_yaw(0, relative=True)
    

def sag(absdeger):
    goto_position_target_relative_ned(0, absdeger/5)
    condition_yaw(0, relative=True)
    

def sol(absdeger):
    goto_position_target_relative_ned(0, -(absdeger/5))
    condition_yaw(0, relative=True)
    
def alcal():
    goto_position_target_relative_ned(0, 0,1)
    condition_yaw(0, relative=True)
    

def baslat():
	
	radius = 1
	cap = cv2.VideoCapture(0)
	circle_x = 0
	circle_y = 0
	z = 0
	dizix = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
	diziy = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
	px_mt_list = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
	toplamx = 0
	toplamy = 0
	toplampx = 1
	font = cv2.FONT_HERSHEY_SIMPLEX
	sol_basic=int((abs(orgin_x - ortalamax) * ortalamapx))
	ileri_basic=int(abs((orgin_y - ortalamay) * ortalamapx))
	while cap.isOpened():
		ret, frame = cap.read()

		vertical, horizontal, temp = frame.shape
		# Goruntunun Merkezini alıyoruz
		orgin_x = horizontal // 2
		orgin_y = vertical // 2
		# Goruntunun Merkezine Nokta Koyuyoruz
		cv2.circle(frame, (orgin_x, orgin_y), radius=5, color=(0, 0, 0), thickness=-1)

		# BGR to HSV Donuşumu
		HSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

		# blur çeşitleri
		# blur = cv2.medianBlur(HSV, 3)
		# blur = cv2.GaussianBlur(HSV, (3, 3), 0)
		blur = cv2.GaussianBlur(HSV, (5, 5), 2, 2)
		# blur = cv2.medianBlur(HSV, 5)
		# Kırmızının alt ve ust degerlerinin tanımlanması
		# lower = np.array([136, 87, 111], np.uint8)
		# upper = np.array([180, 255, 255], np.uint8)
		lower = np.array([146, 92, 0])
		upper = np.array([179, 255, 255])

		# alt ve ust degerlerden hanggisine denk geldigini tespit etmesi
		Red_mask = cv2.inRange(blur, lower, upper)
		result = cv2.bitwise_and(frame, frame, mask=Red_mask)

		# uçunucu input "minDist" oluyor ve bu da bulunan çemberrlerin merkezi arasındaki minimum mesafesi oluyormuş
		# eger fazla çember tespiti yapıyorsa bu ayarı yuksek tutarak daha az çember tespit etmesi saglanabilir
		circles = cv2.HoughCircles(Red_mask, cv2.HOUGH_GRADIENT, 1, Red_mask.shape[0] / 8, param1=300, param2=13,
								   minRadius=15, maxRadius=150)

		# Maskeleme yapılan goruntude çember varsa tespiti yapılıp çevresine çember çizilir
		if circles is not None:
			circles = np.round(circles[0, :]).astype("int")
			cv2.circle(frame, center=(circles[0, 0], circles[0, 1]), radius=circles[0, 2], color=(0, 255, 0),
					   thickness=2)
			cv2.circle(frame, center=(circles[0, 0], circles[0, 1]), radius=1, color=(0, 255, 0), thickness=2)
			circle_x = circles[0, 0]
			circle_y = circles[0, 1]
			radius = circles[0, 2]

			reference_object = 5
			px_mt_ratio = reference_object / radius  # Burada referans cismin boyutunu toplam pixel uzunluguna bolup 1cm veya metrenin kaç pixel oldugunu buluyoruz
			px_mt_list[z] = px_mt_ratio
			# print(px_mt_ratio)
			# print(abs(orgin_x-circle_x)*px_mt_ratio)
			# print(abs(orgin_y-circle_y)*px_mt_ratio)

			# Tespit edilen daireye x ve y ekseninde çizgi çizer
			cv2.line(frame, (orgin_x, orgin_y), (circle_x, orgin_y), (255, 0, 0), 3)
			cv2.line(frame, (orgin_x, orgin_y), (orgin_x, circle_y), (255, 0, 0), 3)

			if ((100 > orgin_x - circle_x > 0) and (100 > circle_y - orgin_y > 0)):
				z = z + 1
				if (z == 10):
					print("Alcaliyor")
					alcal()
					z = 0
				try:
					uzaklik = round(sol_basic, 2)
					cv2.putText(frame, str(uzaklik), (circle_x, orgin_y), font, 1, (255, 255, 255), 1, cv2.LINE_AA)
					uzakliky = round((abs(orgin_y - ortalamay) * ortalamapx), 2)
					cv2.putText(frame, str(uzakliky), (orgin_x, circle_y), font, 1, (255, 255, 255), 1, cv2.LINE_AA)
				except:
					pass
			elif ((orgin_x - circle_x > 0) and (circle_y - orgin_y > 0)):
				dizix[z] = circle_x
				diziy[z] = circle_y
				z = z + 1
				print(str(circle_x) + " ORJİN " + str(circle_y))
				if (z == 10):
					ortalamax, ortalamay, ortalamapx = ort(dizix, diziy, px_mt_list)
					print(str(ortalamax) + " " + str(ortalamay))
					print(str(sol_basic) + " CM Sol ve " + str(
						ileri_basic) + " CM Geriye gidiliyor")
					
					sol(sol_basic)
					geri(int(ileri_basic))
					# print(str((abs(orgin_x - circle_x) * px_mt_ratio)) + " CM Sol ve " + str(abs((orgin_y - circle_y) * px_mt_ratio)) + " CM Geriye git")
					dizix = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
					diziy = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
					px_mt_list = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
					toplamx = 0
					toplamy = 0
					toplampx = 0
					z = 0

				try:
					uzaklik = round(sol_basic, 2)
					cv2.putText(frame, str(uzaklik), (circle_x, orgin_y), font, 1, (255, 255, 255), 1, cv2.LINE_AA)
					uzakliky = round((abs(orgin_y - ortalamay) * ortalamapx), 2)
					cv2.putText(frame, str(uzakliky), (orgin_x, circle_y), font, 1, (255, 255, 255), 1, cv2.LINE_AA)
				except:
					pass


			elif ((orgin_x - circle_x > 0) and (circle_y - orgin_y < 0)):
				dizix[z] = circle_x
				diziy[z] = circle_y
				z = z + 1
				if (z == 10):
					ortalamax, ortalamay, ortalamapx = ort(dizix, diziy, px_mt_list)
					print(str(ortalamax) + " " + str(ortalamay))
					print(str(sol_basic) + "CM Sol ve " + str(
						int(ileri_basic)) + "CM ileri git")
					sol(sol_basic)
					ileri(int(ileri_basic))
					# print(str((abs(orgin_x - circle_x) * px_mt_ratio)) + "CM Sol ve " + str(abs((orgin_y - circle_y) * px_mt_ratio)) + "CM ileri git")
					dizix = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
					diziy = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
					px_mt_list = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
					toplamx = 0
					toplamy = 0
					toplampx = 0
					z = 0

				try:
					uzaklik = round(sol_basic, 2)
					cv2.putText(frame, str(uzaklik), (circle_x, orgin_y), font, 1, (255, 255, 255), 1, cv2.LINE_AA)
					uzakliky = round((abs(orgin_y - ortalamay) * ortalamapx), 2)
					cv2.putText(frame, str(uzakliky), (orgin_x, circle_y), font, 1, (255, 255, 255), 1, cv2.LINE_AA)
				except:
					pass


			elif ((orgin_x - circle_x < 0) and (circle_y - orgin_y > 0)):
				dizix[z] = circle_x
				diziy[z] = circle_y
				z = z + 1
				if (z == 10):
					ortalamax, ortalamay, ortalamapx = ort(dizix, diziy, px_mt_list)
					print(str(ortalamax) + " " + str(ortalamay))
					print(str(int(sol_basic)) + "CM sag ve " + str(
						ileri_basic) + "CM geriye git")
					sag(int(sol_basic))
					geri(int(ileri_basic))

					# print(str((abs(orgin_x - circle_x) * px_mt_ratio)) + "CM sag ve " + str(abs((orgin_y - circle_y) * px_mt_ratio)) + "CM geriye git")
					dizix = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
					diziy = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
					px_mt_list = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
					toplamx = 0
					toplamy = 0
					toplampx = 0
					z = 0

				try:
					uzaklik = round(sol_basic, 2)
					cv2.putText(frame, str(uzaklik), (circle_x, orgin_y), font, 1, (255, 255, 255), 1, cv2.LINE_AA)
					uzakliky = round((abs(orgin_y - ortalamay) * ortalamapx), 2)
					cv2.putText(frame, str(uzakliky), (orgin_x, circle_y), font, 1, (255, 255, 255), 1, cv2.LINE_AA)
				except:
					pass


			elif ((orgin_x - circle_x < 0) and (circle_y - orgin_y < 0)):
				dizix[z] = circle_x
				diziy[z] = circle_y
				z = z + 1
				if (z == 10):
					ortalamax, ortalamay, ortalamapx = ort(dizix, diziy, px_mt_list)
					print(str(ortalamax) + " " + str(ortalamay))
					print(str(int(sol_basic)) + "CM sag ve " + str(
						ileri_basic) + "CM ileri git")
					sag(int(sol_basic))
					ileri(int(ileri_basic))
					# print(str((abs(orgin_x - circle_x) * px_mt_ratio)) + "CM sag ve " + str(abs((orgin_y - circle_y) * px_mt_ratio)) + "CM ileri git")
					dizix = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
					diziy = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
					px_mt_list = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
					toplamx = 0
					toplamy = 0
					toplampx = 0
					z = 0

				try:
					uzaklik = round(sol_basic, 2)
					cv2.putText(frame, str(uzaklik), (circle_x, orgin_y), font, 1, (255, 255, 255), 1, cv2.LINE_AA)
					uzakliky = round((abs(orgin_y - ortalamay) * ortalamapx), 2)
					cv2.putText(frame, str(uzakliky), (orgin_x, circle_y), font, 1, (255, 255, 255), 1, cv2.LINE_AA)
				except:
					pass

		cv2.imshow("And", result)

		cv2.imshow("Mask", Red_mask)
		cv2.imshow("Tracking Red Color", frame)

		if cv2.waitKey(1) & 0xFF == ord('q'):
			break

	cap.release()
	# Pencereleri Yok etme
	cv2.destroyAllWindows()
	
def ort(dizix, diziy, px_mt_list):
    ortalamax = 0
    ortalamay = 0
    print("deneme___ dizi silindi")

    return dizix[5], diziy[1], px_mt_list[5]

	
baslat()


