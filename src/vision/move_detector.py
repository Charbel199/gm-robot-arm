from skimage.metrics import structural_similarity as compare_ssim
import cv2
import imutils
from src.utils.cv_utils import concat_images, get_hsv_canny, get_center_points, reduce_points, \
    corner_points_to_2d_matrix, corner_points_to_squares, get_each_square_diff, get_move_made,get_images_diff
import numpy as np

chessboard_map = [['a8', 'a7', 'a6', 'a5', 'a4', 'a3', 'a2', 'a1'],
                  ['b8', 'b7', 'b6', 'b5', 'b4', 'b3', 'b2', 'b1'],
                  ['c8', 'c7', 'c6', 'c5', 'c4', 'c3', 'c2', 'c1'],
                  ['d8', 'd7', 'd6', 'd5', 'd4', 'd3', 'd2', 'd1'],
                  ['e8', 'e7', 'e6', 'e5', 'e4', 'e3', 'e2', 'e1'],
                  ['f8', 'f7', 'f6', 'f5', 'f4', 'f3', 'f2', 'f1'],
                  ['g8', 'g7', 'g6', 'g5', 'g4', 'g3', 'g2', 'g1']]


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
    imageA = cv2.imread('../assets/moves/move2-1.png')
    imageB = cv2.imread('../assets/moves/move2-2.png')

    if imageA.shape != imageB.shape:
        print(imageA.shape)
        print(imageB.shape)
        up_width = imageA.shape[1]
        up_height = imageA.shape[0]
        up_points = (up_width, up_height)
        imageB = cv2.resize(imageB, up_points, interpolation=cv2.INTER_LINEAR)

    print(imageA.shape)
    print(imageB.shape)
    imageA_copy = imageA.copy()
    imageB_copy = imageB.copy()

    lwr_b = np.array([0, 69, 0])
    upr_b = np.array([179, 245, 255])
    lwr_w = np.array([0, 0, 146])
    upr_w = np.array([179, 66, 234])

    # For image A
    res1A = get_hsv_canny(imageA, lwr_b, upr_b)
    res2A = get_hsv_canny(imageA, lwr_w, upr_w)
    resA = cv2.bitwise_or(res1A, res2A)
    center_points, corner_points, processed_image = [], [], []
    default_lower_area = 4000
    while len(center_points) != 64:
        center_points, corner_points, processed_image = get_center_points(resA, lower_area=default_lower_area)
        if len(center_points) != 64:
            default_lower_area -= 500
    print(f"Best default lower area {default_lower_area}")
    center_points, corner_points, processed_image = get_center_points(resA, lower_area=2000, draw_points=True,
                                                                      draw_contours=True, image=imageA_copy)
    corner_points = reduce_points(corner_points)
    center_points = reduce_points(center_points)
    print(len(center_points))
    assert len(center_points) == 64
    assert len(corner_points) == 81
    matrix_2dA = corner_points_to_2d_matrix(corner_points)
    squaresA = corner_points_to_squares(matrix_2dA, with_text=True, image=imageA_copy)

    # For image B
    res1B = get_hsv_canny(imageB, lwr_b, upr_b)
    res2B = get_hsv_canny(imageB, lwr_w, upr_w)
    resB = cv2.bitwise_or(res1B, res2B)
    center_points, corner_points, processed_image = [], [], []
    default_lower_area = 4000
    while len(center_points) != 64:
        center_points, corner_points, processed_image = get_center_points(resB, lower_area=default_lower_area)
        if len(center_points) != 64:
            default_lower_area -= 500
    print(f"Best default lower area {default_lower_area}")
    center_points, corner_points, processed_image = get_center_points(resB, lower_area=2000, draw_points=True,
                                                                      draw_contours=True, image=imageB_copy)
    corner_points = reduce_points(corner_points)
    center_points = reduce_points(center_points)
    print(len(center_points))
    assert len(center_points) == 64
    assert len(corner_points) == 81
    matrix_2dB = corner_points_to_2d_matrix(corner_points)
    print(matrix_2dB)
    squaresB = corner_points_to_squares(matrix_2dB, with_text=True, image=imageB_copy)

    squares_with_differences = get_each_square_diff(imageA, imageB, squaresA, squaresB, threshold=0.4, show_box=True,
                                                    image=imageA_copy)

    move_index = get_move_made(squares_with_differences, matrix_2dA)
    for move in move_index:
        print(f"Move {chessboard_map[move[0]][move[1]]}")


    images_to_show = [imageA_copy, imageB_copy, res1A, res2A, resA, res1B, res2B, resB, processed_image]
    images_titles = ["imageA", "imageB", "res1A-b", "res2A-w", "resA-tot", "res1B-b", "res2B-w", "resB-tot",
                     "processed_image"]
    final_image = concat_images(images_to_show, images_titles)
    cv2.imshow("final_image", final_image)

    while (1):
        if cv2.waitKey(33) == ord('q'):
            break
