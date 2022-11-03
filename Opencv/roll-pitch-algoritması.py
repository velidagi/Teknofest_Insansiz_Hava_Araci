import cv2
import time
import numpy as np
cap = cv2.VideoCapture("")
frameCounter = 0
sayi = 0
while 1:


    ret, frame = cap.read()
    frame = cv2.flip(frame, 1)
    frame = cv2.resize(frame, (640, 480))
    vertical, horizontal, temp = frame.shape
    #Görüntünün Merkezini orjin olarak alıyoruz
    orgin_x = horizontal // 2
    orgin_y = vertical // 2
    cv2.namedWindow("frame", cv2.WINDOW_NORMAL)
    blur = cv2.GaussianBlur(frame, (7, 7), 0)
    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
    lower_red = np.array([123, 64, 132])
    upper_red = np.array([255, 255, 255])
    mask = cv2.inRange(hsv, lower_red, upper_red)
    # hedef disindakileri sil
    mask1 = cv2.erode(mask, None, iterations=1)
    mask2 = cv2.dilate(mask1, None, iterations=4)
    mask_copy = mask2.copy()

    # konturlerin yerleri tesit ediliyor
    contours, _ = cv2.findContours(mask_copy, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cv2.rectangle(frame, (230, 150), (410, 330), (125, 0, 20), 4)
    cv2.circle(frame, (320, 240), (2), (0, 0, 255), 4)

    # eger kontur varsa
    if len(contours) > 0:
        areas = [cv2.contourArea(c) for c in contours]
        max_index = np.argmax(areas)
        ctr = contours[max_index]

        # konturu  cevreleyen cemberin kordinati ve yari capi
        (circle_x, circle_y), radius = cv2.minEnclosingCircle(ctr)

        if radius >= 20:  # draw circle if radius>40 px

            cv2.circle(frame, (int(circle_x), int(circle_y)), int(radius), (0, 255, 255), 4)
            cv2.circle(frame, (int(circle_x), int(circle_y)), int(4), (0, 255, 255), 4, -1)
            cv2.line(frame, (320, 240), (int(circle_x), int(circle_y)), (0, 0, 255), 3, lineType=cv2.LINE_4)

            # Döner Kanatı bir yöne doğru uçurmak için aldığımız pixel değerlerinden belirli bir oranda oraya yönelebilmek için yaptığımız oranlamalar
            left_ = abs(circle_x - orgin_x) / 20
            right_ = (circle_x - orgin_x) / 20
            forward_ = (orgin_y - circle_y) / 20
            back_ = abs(circle_y - orgin_y) / 20
            #ROLL PITCH Yaparak Kırmızı daireyi ortalayıp alçalmak için yazılan if else bloğu
            if ((10 > orgin_x - circle_x > 0) and (10 > circle_y - orgin_y > 0)):
                frameCounter = frameCounter + 1
                cv2.putText(frame, "alcaliyor", (200, 335), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1.0, (0, 0, 255), 2)
                if (frameCounter % 10 == 0):
                    print("alcal")
            elif ((orgin_x - circle_x > 0) and (circle_y - orgin_y > 0)):
                frameCounter = frameCounter + 1
                cv2.putText(frame, "sol ve geriye", (200, 335), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1.0, (0, 0, 255), 2)
                if (frameCounter % 10 == 0):
                    print(str(left_) + " Sol ve " + str(back_) + " geriye")
            elif ((orgin_x - circle_x > 0) and (circle_y - orgin_y < 0)):
                frameCounter = frameCounter + 1
                cv2.putText(frame, "sol ve ileriye", (200, 335), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1.0, (0, 0, 255), 2)
                if (frameCounter % 10 == 0):
                    print(str(left_) + " Sol ve " + str(forward_) + " ileri")
            elif ((orgin_x - circle_x < 0) and (circle_y - orgin_y > 0)):
                frameCounter = frameCounter + 1
                cv2.putText(frame, "sag ve geriye", (200, 335), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1.0, (0, 0, 255), 2)
                if (frameCounter % 10 == 0):
                    print(str(right_) + " Sag ve " + str(back_) + " geriye")
            elif ((orgin_x - circle_x < 0) and (circle_y - orgin_y < 0)):
                frameCounter = frameCounter + 1
                cv2.putText(frame, "sag ve ileri", (200, 335), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1.0, (0, 0, 255), 2)
                if (frameCounter % 10 == 0):
                    print(str(right_) + " Sag ve " + str(forward_) + " ileri")
        else:
            print("bekliyorum")
            cv2.putText(frame, "bekliyorum", (220, 160), cv2.FONT_HERSHEY_PLAIN, 1.0, (0, 240, 0), 2)

    cv2.imshow("camera", frame)

    if cv2.waitKey(100) == 27:
        break
