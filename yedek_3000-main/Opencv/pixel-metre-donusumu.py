import cv2
import numpy as np

radius = 1
cap = cv2.VideoCapture("")
circle_x = 0
circle_y = 0
frameCounter = 0
dizix = []
diziy = []
px_mt_list = []
toplamx = 0
toplamy = 0
toplampx = 1
font = cv2.FONT_HERSHEY_SIMPLEX

def ort(dizix, diziy, px_mt_list, toplamx=toplamx, toplamy=toplamy, toplampx=toplampx):

    for i in range(0, len(dizix)-1):
        toplamx += dizix[i]

    ortalamax = toplamx / len(dizix)

    for i in range(0, len(diziy)-1):
        toplamy += diziy[i]

    ortalamay = toplamy / len(diziy)

    for i in range(0, len(px_mt_list)-1):
        toplamy += px_mt_list[i]

    ortalamapx = toplampx / len(px_mt_list)

    return ortalamax, ortalamay, ortalamapx

while cap.isOpened():
    ret, frame = cap.read()

    vertical, horizontal, temp = frame.shape
    #Görüntünün Merkezini orjin olarak alıyoruz
    orgin_x = horizontal // 2
    orgin_y = vertical // 2
    cv2.circle(frame, (orgin_x, orgin_y), radius=5, color=(0, 0, 0), thickness=-1)

    HSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    blur = cv2.GaussianBlur(HSV, (5, 5), 2, 2)
    lower = np.array([146, 92, 0])
    upper = np.array([179, 255, 255])

    Red_mask = cv2.inRange(blur, lower, upper)
    result = cv2.bitwise_and(frame, frame, mask=Red_mask)
    contours, _ = cv2.findContours(Red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # Maskeleme yapılan görüntüde çember varsa tespiti yapılıp çevresine çember çizilir
    if len(contours) > 0:

        for ctr in contours:
            # konturu  cevreleyen çemberin kordinati ve yarı çapı
            (circle_x, circle_y), radius = cv2.minEnclosingCircle(ctr)
            circle_x = int(circle_x)
            circle_y = int(circle_y)
            radius = int(radius)

            if radius >= 20:
                cv2.circle(frame, (int(circle_x), int(circle_y)), int(radius), (0, 255, 255), 4)
                cv2.circle(frame, (int(circle_x), int(circle_y)), int(4), (0, 255, 255), 4, -1)
                cv2.line(frame, (320, 240), (int(circle_x), int(circle_y)), (0, 0, 255), 3, lineType=cv2.LINE_4)
                cv2.putText(frame, "hedef tespit edildi", (20, 20), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1.0, (0, 240, 0),2)

                #Görüntüdeki referans objenin gerçek yarı çapı
                reference_object = 5
                px_mt_ratio = reference_object / radius  # Burada referans cismin boyutunu toplam pixel uzunluğuna bölüp 1cm veya metrenin kaç pixel olduğunu buluyoruz
                px_mt_list.append(px_mt_ratio)

                # Tespit edilen daireye x ve y ekseninde çizgi çizer
                cv2.line(frame, (orgin_x, orgin_y), (int(circle_x), orgin_y), (255, 0, 0), 3)
                cv2.line(frame, (orgin_x, orgin_y), (orgin_x, int(circle_y)), (255, 0, 0), 3)


                if ((orgin_x - circle_x > 0) and (circle_y - orgin_y > 0)):
                    dizix.append(circle_x)
                    diziy.append(circle_y)
                    frameCounter = frameCounter + 1
                    if (frameCounter % 10 == 0):
                        ortalamax, ortalamay, ortalamapx = ort(dizix, diziy, px_mt_list)
                        #görüntüdeki değerler ile farklıdır. Uçuş sırasında koordinat hesabı yapılırken ortalaması
                        #kullanılarak oluşabilecek sapmaları minimuma indirebilmek için kullanılmıştır
                        print(str((abs(orgin_x - ortalamax) * ortalamapx)) + " Sol ve " + str(abs((orgin_y - ortalamay) * ortalamapx)) + " geriye")
                        dizix.clear()
                        diziy.clear()
                        px_mt_list.clear()

                    try:
                        uzaklik = round((abs(orgin_x - circle_x) * px_mt_ratio), 2)
                        cv2.putText(frame, str(uzaklik), (circle_x, orgin_y), font, 1, (255, 255, 255), 1, cv2.LINE_AA)
                        uzakliky = round(abs((orgin_y - circle_y) * px_mt_ratio), 2)
                        cv2.putText(frame, str(uzakliky), (orgin_x, circle_y), font, 1, (255, 255, 255), 1, cv2.LINE_AA)
                    except:
                        pass

                elif ((orgin_x - circle_x > 0) and (circle_y - orgin_y < 0)):
                    dizix.append(circle_x)
                    diziy.append(circle_y)
                    frameCounter = frameCounter + 1
                    if (frameCounter % 10 == 0):
                        ortalamax, ortalamay, ortalamapx = ort(dizix, diziy, px_mt_list)
                        print(str((abs(orgin_x - ortalamax) * ortalamapx)) + "Sol ve " + str(abs((orgin_y - ortalamay) * ortalamapx)) + " ileri")
                        dizix.clear()
                        diziy.clear()
                        px_mt_list.clear()

                    try:
                        uzaklik = round((abs(orgin_x - circle_x) * px_mt_ratio), 2)
                        cv2.putText(frame, str(uzaklik), (circle_x, orgin_y), font, 1, (255, 255, 255), 1, cv2.LINE_AA)
                        uzakliky = round(abs((orgin_y - circle_y) * px_mt_ratio), 2)
                        cv2.putText(frame, str(uzakliky), (orgin_x, circle_y), font, 1, (255, 255, 255), 1, cv2.LINE_AA)
                    except:
                        pass

                elif ((orgin_x - circle_x < 0) and (circle_y - orgin_y > 0)):
                    dizix.append(circle_x)
                    diziy.append(circle_y)
                    frameCounter = frameCounter + 1
                    if (frameCounter % 10 == 0):
                        ortalamax, ortalamay, ortalamapx = ort(dizix, diziy, px_mt_list)
                        print(str((abs(orgin_x - ortalamax) * ortalamapx)) + "Sağ ve " + str(abs((orgin_y - ortalamay) * ortalamapx)) + " geriye")
                        dizix.clear()
                        diziy.clear()
                        px_mt_list.clear()

                    try:
                        uzaklik = round((abs(orgin_x - circle_x) * px_mt_ratio), 2)
                        cv2.putText(frame, str(uzaklik), (circle_x, orgin_y), font, 1, (255, 255, 255), 1, cv2.LINE_AA)
                        uzakliky = round(abs((orgin_y - circle_y) * px_mt_ratio), 2)
                        cv2.putText(frame, str(uzakliky), (orgin_x, circle_y), font, 1, (255, 255, 255), 1, cv2.LINE_AA)
                    except:
                        pass

                elif ((orgin_x - circle_x < 0) and (circle_y - orgin_y < 0)):
                    dizix.append(circle_x)
                    diziy.append(circle_y)
                    frameCounter = frameCounter + 1
                    if (frameCounter % 10 == 0):
                        ortalamax, ortalamay, ortalamapx = ort(dizix, diziy, px_mt_list)
                        print(str((abs(orgin_x - ortalamax) * ortalamapx)) + "Sağ ve " + str(abs((orgin_y - ortalamay) * ortalamapx)) + "ileri")
                        dizix.clear()
                        diziy.clear()
                        px_mt_list.clear()
                    try:
                        uzaklik = round((abs(orgin_x - circle_x) * px_mt_ratio), 2)
                        cv2.putText(frame, str(uzaklik), (circle_x, orgin_y), font, 1, (255, 255, 255), 1, cv2.LINE_AA)
                        uzakliky = round(abs((orgin_y - circle_y) * px_mt_ratio), 2)
                        cv2.putText(frame, str(uzakliky), (orgin_x, circle_y), font, 1, (255, 255, 255), 1, cv2.LINE_AA)
                    except:
                        pass
    else:

        cv2.putText(frame, "hedef araniyor...", (20, 20), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1.0, (0, 0, 240),2)

    cv2.imshow("Alan Taramasi", frame)

    if cv2.waitKey(100) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
