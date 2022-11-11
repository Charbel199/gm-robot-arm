from skimage.metrics import structural_similarity as compare_ssim
import cv2
import imutils
from src.utils.cv_utils import concat_images, get_image_information, get_each_square_diff, get_move_made, get_hsv_filter, sort_array_of_points, get_four_corners
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
    # empty_image = cv2.imread('src/assets/moves/empty_lichess2.png')
    # imageA = cv2.imread('src/assets/moves/lichess2_1.png')
    # imageB = cv2.imread('src/assets/moves/lichess2_2.png')

    empty_image = cv2.imread('src/assets/moves/real_board/Move_Empty.jpeg')
    imageA = cv2.imread('src/assets/moves/real_board/Move_Initial.jpeg')
    imageB = cv2.imread('src/assets/moves/real_board/Move1.jpeg')

    image_to_warp = imageB.copy()
    # Red markers
    hsv_min_marker = np.array([0, 177, 240])
    hsv_max_marker = np.array([178, 255, 255])
    res = get_hsv_filter(image_to_warp, hsv_min_marker, hsv_max_marker)
    kernel = np.ones((3, 3), np.uint8)
    res = cv2.dilate(res,kernel)
    contours, hierarchy = cv2.findContours(res, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    main_ctnrs = []
    centers = []
    for c in contours:
        print("AREA ",cv2.contourArea(c))
        if cv2.contourArea(c) >30:
            main_ctnrs.append(c)
            (x, y, w, h) = cv2.boundingRect(c)
            centers.append((x,y))
            cv2.rectangle(image_to_warp, (x, y), (x + w, y + h), (127, 0, 255), 2)

    centers = get_four_corners(centers)
    # Show keypoints
    ii=np.zeros_like(image_to_warp)

    print(centers)
    pt_A = [centers[0][0],centers[0][1]]
    pt_B = [centers[1][0],centers[1][1]]
    pt_C = [centers[2][0],centers[2][1]]
    pt_D = [centers[3][0],centers[3][1]]
    cv2.circle(image_to_warp, pt_A, 5, (255, 0, 0), 1)
    cv2.circle(image_to_warp, pt_B, 5, (0, 255, 0), 1)
    cv2.circle(image_to_warp, pt_C, 5, (0, 0, 255), 1)
    cv2.circle(image_to_warp, pt_D, 5, (255, 244, 0), 1)
    width, height = 600,600
    # to calculate the transformation matrix
    input_pts = np.float32([pt_A, pt_D, pt_C, pt_B])
    output_pts = np.float32([[0, 0],
                             [width, 0],
                             [width, height],
                             [0, height]])

    # Compute the perspective transform M
    M = cv2.getPerspectiveTransform(input_pts, output_pts)

    # Apply the perspective transformation to the image
    out = cv2.warpPerspective(image_to_warp, M, (width,height))


    cv2.imshow("Keypoints", out)

    while (1):
        if cv2.waitKey(33) == ord('q'):
            break

    if imageA.shape != imageB.shape:
        print(f"Reshaping imageA from {imageA.shape} to {imageB.shape}")
        up_width = imageA.shape[1]
        up_height = imageA.shape[0]
        up_points = (up_width, up_height)
        imageB = cv2.resize(imageB, up_points, interpolation=cv2.INTER_LINEAR)

    # Lichess White and Blue board
    # hsv_min_b = np.array([0, 69, 0])
    # hsv_max_b = np.array([179, 245, 255])
    # hsv_min_w = np.array([0, 0, 146])
    # hsv_max_w = np.array([179, 66, 234])

    # Lounge brown and white board
    # hsv_min_b = np.array([0, 0, 122])
    # hsv_max_b = np.array([74, 87, 255])
    # hsv_min_w = np.array([7, 0, 0])
    # hsv_max_w = np.array([93, 255, 124])

    # Lounge black and white board
    hsv_min_b = np.array([0, 0, 0])
    hsv_max_b = np.array([27, 241, 90])
    hsv_min_w = np.array([0, 0, 84])
    hsv_max_w = np.array([179, 255, 255])

    image_write = imageA.copy()
    squares, matrix_2d, calibration_b, calibration_w, calibration_bw, calibraion_processed = get_image_information(
        empty_image, image_write, hsv_min_b, hsv_max_b,
        hsv_min_w, hsv_max_w)

    # Difference between snapshots
    # squares_with_differences, squares_differences_images = get_each_square_diff(imageA, imageB, squares, threshold=0.8,
    #                                                                             show_box=True,
    #                                                                             image=image_write)
    #
    # move_index = get_move_made(squares_with_differences, matrix_2d)
    # for move in move_index:
    #     print(f"Move {chessboard_map[move[0]][move[1]]}")


    # Show images
    images_to_show = [image_write, calibration_b, calibration_w, calibration_bw, calibraion_processed
                      ]
    images_titles = ['image_copy', 'calibration_b', 'calibration_w', 'calibration_bw', 'calibraion_processed',
                     'squares_differences_images']
    final_image = concat_images(images_to_show, images_titles)
    cv2.imshow("final_image", final_image)

    while (1):
        if cv2.waitKey(33) == ord('q'):
            break
