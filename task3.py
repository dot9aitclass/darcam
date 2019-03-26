import numpy as np
import cv2
import time

# from yuvtrack import Tracker
n = int(input("Enter camera index: "))
y, u, v = 151, 83, 109
resx = 640 / 2
resy = 480 / 2
print(y, u, v)
cap = cv2.VideoCapture(n)
radius = 0
x = 0
y = 0
while True:
    area = []
    ret, frame = cap.read()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2YUV)
    # print(y,u,v)
    lower_color = np.array([0, u - 30, v - 30])
    upper_color = np.array([255, u + 30, v + 30])
    mask = cv2.inRange(hsv, lower_color, upper_color)
    erode = cv2.erode(mask, None, iterations=1)
    dilate = cv2.dilate(erode, None, iterations=1)
    img, contours, hierarchy = cv2.findContours(
        dilate, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    for cnt in contours:
        area.append(cv2.contourArea(cnt))
    # cv2.drawContours(frame,contours,-1,(0,0,255),3)
    if len(area) != 0:
        m = max(area)
    for cnt in contours:
        if cv2.contourArea(cnt) >= m:
            # global radius,x,y
            (x, y), radius = cv2.minEnclosingCircle(cnt)
            break

    cv2.circle(dilate, (int(resx), int(resy)), int(radius + 20), (255, 255, 255), 2)
    if x - resx < 0 and abs(x - resx) > radius:
        print("Go Left")
    elif abs(x - resx) > radius and x - resx > 0:
        print("Go Right")
    else:
        print("Go straight")
    if abs(y - resy) > radius and y - resy < 0:
        print("Look up")
    if abs(y - resy) > radius and y - resy > 0:
        print("Look Down")
    # center = (int(x),int(y))
    # sobframe=cv2.Sobel(frame,cv2.CV_64F,1,0,ksize=5)
    # if cen1[0]-center[0]<-50:
    #     print ("wrong way,move towards right")
    # if cen1[0]-center[0]>50:
    #     print ("wrong way,move towards left")
    cv2.imshow("frame", dilate)
    # cen1=center
    if cv2.waitKey(30) == 27:
        break
cv2.destroyAllWindows()
cap.release()
