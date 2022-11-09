from skimage.metrics import structural_similarity as compare_ssim
import cv2
import imutils
from src.utils.cv_utils import concat_images, get_image_information, get_each_square_diff, get_move_made
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
    empty_image = cv2.imread('src/assets/moves/empty_lichess2.png')
    imageA = cv2.imread('src/assets/moves/lichess2_1.png')
    imageB = cv2.imread('src/assets/moves/lichess2_2.png')

    if imageA.shape != imageB.shape:
        print(f"Reshaping imageA from {imageA.shape} to {imageB.shape}")
        up_width = imageA.shape[1]
        up_height = imageA.shape[0]
        up_points = (up_width, up_height)
        imageB = cv2.resize(imageB, up_points, interpolation=cv2.INTER_LINEAR)

    # Lichess White and Blue board
    hsv_min_b = np.array([0, 69, 0])
    hsv_max_b = np.array([179, 245, 255])
    hsv_min_w = np.array([0, 0, 146])
    hsv_max_w = np.array([179, 66, 234])

    # Lounge brown and white board
    # hsv_min_b = np.array([0, 0, 122])
    # hsv_max_b = np.array([74, 87, 255])
    # hsv_min_w = np.array([7, 0, 0])
    # hsv_max_w = np.array([93, 255, 124])

    image_write = imageA.copy()
    squares, matrix_2d, calibration_b, calibration_w, calibration_bw, calibraion_processed = get_image_information(
        empty_image, image_write, hsv_min_b, hsv_max_b,
        hsv_min_w, hsv_max_w)

    # Difference between snapshots
    squares_with_differences, squares_differences_images = get_each_square_diff(imageA, imageB, squares, threshold=0.8,
                                                                                show_box=True,
                                                                                image=image_write)

    move_index = get_move_made(squares_with_differences, matrix_2d)
    for move in move_index:
        print(f"Move {chessboard_map[move[0]][move[1]]}")


    # Show images
    images_to_show = [image_write, calibration_b, calibration_w, calibration_bw, calibraion_processed,
                      squares_differences_images]
    images_titles = ['image_copy', 'calibration_b', 'calibration_w', 'calibration_bw', 'calibraion_processed',
                     'squares_differences_images']
    final_image = concat_images(images_to_show, images_titles)
    cv2.imshow("final_image", final_image)

    while (1):
        if cv2.waitKey(33) == ord('q'):
            break
