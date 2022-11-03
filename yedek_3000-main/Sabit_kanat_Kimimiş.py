import cv2
import numpy as np

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


def ort(dizix, diziy, px_mt_list, toplamx=toplamx, toplamy=toplamy, toplampx=toplampx):
    ortalamax = 0
    ortalamay = 0
    for i in range(0, len(dizix)):
        toplamx += dizix[i]

    ortalamax = toplamx / len(dizix)

    for i in range(0, len(diziy)):
        toplamy += diziy[i]

    ortalamay = toplamy / len(diziy)

    for i in range(0, len(px_mt_list)):
        toplamy += px_mt_list[i]

    ortalamapx = toplampx / len(px_mt_list)

    return ortalamax, ortalamay, ortalamapx


while cap.isOpened():
    ret, frame = cap.read()

    vertical, horizontal, temp = frame.shape
    # Görüntünün Merkezini alıyoruz
    orgin_x = horizontal // 2
    orgin_y = vertical // 2
    # Görüntünün Merkezine Nokta Koyuyoruz
    cv2.circle(frame, (orgin_x, orgin_y), radius=5, color=(0, 0, 0), thickness=-1)

    # BGR to HSV Dönüşümü
    HSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # blur çeşitleri
    # blur = cv2.medianBlur(HSV, 3)
    # blur = cv2.GaussianBlur(HSV, (3, 3), 0)
    blur = cv2.GaussianBlur(HSV, (5, 5), 2, 2)
    #blur = cv2.medianBlur(HSV, 5)
    # Kırmızının alt ve üst değerlerinin tanımlanması
    # lower = np.array([136, 87, 111], np.uint8)
    # upper = np.array([180, 255, 255], np.uint8)
    lower = np.array([146, 92, 0])
    upper = np.array([179, 255, 255])

    # alt ve üst değerlerden hanggisine denk geldiğini tespit etmesi
    Red_mask = cv2.inRange(blur, lower, upper)
    result = cv2.bitwise_and(frame, frame, mask=Red_mask)

    # üçünücü input "minDist" oluyor ve bu da bulunan çemberrlerin merkezi arasındaki minimum mesafesi oluyormuş
    # eğer fazla çember tespiti yapıyorsa bu ayarı yüksek tutarak daha az çember tespit etmesi sağlanabilir
    circles = cv2.HoughCircles(Red_mask, cv2.HOUGH_GRADIENT, 1, Red_mask.shape[0] / 8, param1=300, param2=13,
                               minRadius=15, maxRadius=150)

    # Maskeleme yapılan görüntüde çember varsa tespiti yapılıp çevresine çember çizilir
    if circles is not None:
        circles = np.round(circles[0, :]).astype("int")
        cv2.circle(frame, center=(circles[0, 0], circles[0, 1]), radius=circles[0, 2], color=(0, 255, 0), thickness=2)
        cv2.circle(frame, center=(circles[0, 0], circles[0, 1]), radius=1, color=(0, 255, 0), thickness=2)
        circle_x = circles[0, 0]
        circle_y = circles[0, 1]
        radius = circles[0, 2]

        reference_object = 5
        px_mt_ratio = reference_object / radius  # Burada referans cismin boyutunu toplam pixel uzunluğuna bölüp 1cm veya metrenin kaç pixel olduğunu buluyoruz
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
                print("Alçal")
                z = 0
            try:
                uzaklik = round((abs(orgin_x - ortalamax) * ortalamapx), 2)
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
                print(str((abs(orgin_x - ortalamax) * ortalamapx)) + " CM Sol ve " + str(
                    abs((orgin_y - ortalamay) * ortalamapx)) + " CM Geriye git")
                # print(str((abs(orgin_x - circle_x) * px_mt_ratio)) + " CM Sol ve " + str(abs((orgin_y - circle_y) * px_mt_ratio)) + " CM Geriye git")
                dizix = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
                diziy = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
                px_mt_list = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
                toplamx = 0
                toplamy = 0
                toplampx = 0
                z = 0

            try:
                uzaklik = round((abs(orgin_x - ortalamax) * ortalamapx), 2)
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
                print(str((abs(orgin_x - ortalamax) * ortalamapx)) + "CM Sol ve " + str(
                    abs((orgin_y - ortalamay) * ortalamapx)) + "CM ileri git")
                # print(str((abs(orgin_x - circle_x) * px_mt_ratio)) + "CM Sol ve " + str(abs((orgin_y - circle_y) * px_mt_ratio)) + "CM ileri git")
                dizix = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
                diziy = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
                px_mt_list = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
                toplamx = 0
                toplamy = 0
                toplampx = 0
                z = 0

            try:
                uzaklik = round((abs(orgin_x - ortalamax) * ortalamapx), 2)
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
                print(str((abs(orgin_x - ortalamax) * ortalamapx)) + "CM Sağ ve " + str(
                    abs((orgin_y - ortalamay) * ortalamapx)) + "CM geriye git")
                # print(str((abs(orgin_x - circle_x) * px_mt_ratio)) + "CM Sağ ve " + str(abs((orgin_y - circle_y) * px_mt_ratio)) + "CM geriye git")
                dizix = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
                diziy = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
                px_mt_list = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
                toplamx = 0
                toplamy = 0
                toplampx = 0
                z = 0

            try:
                uzaklik = round((abs(orgin_x - ortalamax) * ortalamapx), 2)
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
                print(str((abs(orgin_x - ortalamax) * ortalamapx)) + "CM Sağ ve " + str(
                    abs((orgin_y - ortalamay) * ortalamapx)) + "CM ileri git")
                # print(str((abs(orgin_x - circle_x) * px_mt_ratio)) + "CM Sağ ve " + str(abs((orgin_y - circle_y) * px_mt_ratio)) + "CM ileri git")
                dizix = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
                diziy = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
                px_mt_list = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
                toplamx = 0
                toplamy = 0
                toplampx = 0
                z = 0

            try:
                uzaklik = round((abs(orgin_x - ortalamax) * ortalamapx), 2)
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
cv2.destroyAllWindows()