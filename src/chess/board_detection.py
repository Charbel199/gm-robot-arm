import cv2
import numpy as np
import math
# Load the image
image1 = cv2.imread('../assets/chessboard_bw3.jpg')
lwr = np.array([0, 0, 131])
upr = np.array([179, 40, 255])


# Extract chess-board lines
hsv = cv2.cvtColor(image1, cv2.COLOR_BGR2HSV)
msk = cv2.inRange(hsv, lwr, upr)
krn = cv2.getStructuringElement(cv2.MORPH_RECT, (50, 30))
dlt = cv2.dilate(msk, krn, iterations=5)
res =  cv2.bitwise_and(dlt, msk)
res = np.uint8(res)
res = cv2.Canny(res, 38, 38*3)
cv2.imshow('res',res)


lines= cv2.HoughLines(res, 1, math.pi/180.0, 130)


# Show lines
a,b,c = lines.shape
for i in range(a):
    rho = lines[i][0][0]
    theta = lines[i][0][1]
    a = math.cos(theta)
    b = math.sin(theta)
    x0, y0 = a*rho, b*rho
    pt1 = ( int(x0+1000*(-b)), int(y0+1000*(a)) )
    pt2 = ( int(x0-1000*(-b)), int(y0-1000*(a)) )
    cv2.line(image1, pt1, pt2, (0, 0, 255), 2, cv2.LINE_AA)


cv2.imshow('image1',image1)
while(1):
    if cv2.waitKey(33) == ord('q'):
        break
