import cv2
import numpy as np

# Load the image
img = cv2.imread('../assets/chessboard_bw.jpg')

n_inner_corners = (7,7)
# Color-segmentation to get binary mask
# BW Ranges
lwr = np.array([0, 0, 107])
upr = np.array([179, 45, 238])

# Extract chess-board
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
msk = cv2.inRange(hsv, lwr, upr)
krn = cv2.getStructuringElement(cv2.MORPH_RECT, (50, 30))
dlt = cv2.dilate(msk, krn, iterations=5)
res = 255 - cv2.bitwise_and(dlt, msk)
res = np.uint8(res)

# Find where black squares touch
ret, corners = cv2.findChessboardCorners(res, n_inner_corners,
                                         flags=cv2.CALIB_CB_ADAPTIVE_THRESH +
                                               cv2.CALIB_CB_FAST_CHECK +
                                               cv2.CALIB_CB_NORMALIZE_IMAGE)
if ret:
    print(len(corners))
    fnl = cv2.drawChessboardCorners(img, n_inner_corners, corners, ret)
    cv2.imshow("Chessboard corners", fnl)
else:
    print("No Checkerboard Found")

while(1):
    if cv2.waitKey(33) == ord('q'):
        break