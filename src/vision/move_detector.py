from skimage.metrics import structural_similarity as compare_ssim
import cv2
import imutils
from src.utils.cv_utils import concat_images, get_hsv_canny, get_center_points, reduce_points, \
    corner_points_to_2d_matrix, corner_points_to_squares
import numpy as np


class MoveDetector():
    def __init__(self) -> None:
        self.previous_image = None
        self.current_image = None

    def set_prev_image(self, image):
        self.previous_image = image

    def set_curr_image(self, image):
        self.current_image = image


if __name__ == "__main__":
    # load the two input images
    imageA = cv2.imread('../assets/moves/move1-0.png')
    imageB = cv2.imread('../assets/moves/move1-1.png')
    # convert the images to grayscale
    grayA = cv2.cvtColor(imageA, cv2.COLOR_BGR2GRAY)
    grayB = cv2.cvtColor(imageB, cv2.COLOR_BGR2GRAY)

    (score, diff) = compare_ssim(grayA, grayB, full=True)
    diff = (diff * 255).astype("uint8")
    print("SSIM: {}".format(score))

    thresh = cv2.threshold(diff, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)[1]
    cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)

    # cv2.imshow("Move 0", grayA)
    # cv2.imshow("Move 1", grayB)
    # cv2.imshow("diff", diff)
    # cv2.imshow("thresh", thresh)
    # for c in cnts:
    #     (x, y, w, h) = cv2.boundingRect(c)
    #     cv2.rectangle(imageA, (x, y), (x + w, y + h), (0, 0, 255), 2)
    #     cv2.rectangle(imageB, (x, y), (x + w, y + h), (0, 0, 255), 2)

    lwr_b = np.array([0, 69, 0])
    upr_b = np.array([179, 245, 255])
    lwr_w = np.array([0, 0, 146])
    upr_w = np.array([179, 66, 234])

    res1 = get_hsv_canny(imageA, lwr_b, upr_b)
    res2 = get_hsv_canny(imageA, lwr_w, upr_w)
    res = cv2.bitwise_or(res1, res2)
    center_points, corner_points = get_center_points(res, draw_points=True, image=imageA)
    corner_points = reduce_points(corner_points)
    center_points = reduce_points(center_points)
    assert len(center_points) == 64
    assert len(corner_points) == 81

    matrix_2d = corner_points_to_2d_matrix(corner_points)
    squares = corner_points_to_squares(matrix_2d, with_text=True, image=imageA)

    images_to_show = [grayA, grayB, diff, thresh, imageA, imageB, res1, res2, res]
    images_titles = ["Move 0", "Move 1", "diff", "thresh", "imageA", "imageB", "res1b", "res2w", "restot"]
    final_image = concat_images(images_to_show, images_titles)
    cv2.imshow("final_image", final_image)
    while (1):
        if cv2.waitKey(33) == ord('q'):
            break
