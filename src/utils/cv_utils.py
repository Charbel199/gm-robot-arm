from skimage.metrics import structural_similarity as compare_ssim
import cv2
import imutils
import numpy as np
import math
from sewar.full_ref import mse, rmse, psnr, uqi, ssim, ergas, scc, rase, sam, msssim, vifp
from logger.log import LoggerService

logger = LoggerService.get_instance()


# Draw bounding boxes around contours
def draw_contour_bounding_box(image, contours, color=(0, 0, 255), thickness=2):
    for c in contours:
        (x, y, w, h) = cv2.boundingRect(c)
        cv2.rectangle(image, (x, y), (x + w, y + h), color, thickness)


# Get contours (It is better to have thresholded input)
def get_contours(image):
    cnts = cv2.findContours(image.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    return cnts


# Concat images and titles in a single window
def concat_images(images, titles, force_row_size=None, with_titles=True, width=None, height=None, fontScale=1):
    if width is None:
        width = images[0].shape[1]
    if height is None:
        height = images[0].shape[0]

    font = cv2.FONT_HERSHEY_SIMPLEX
    position = (15, 35)
    fontColor = (0, 255, 0)
    thickness = 1
    lineType = 1
    for i, image in enumerate(images):
        # Make all images 3 channels
        images[i] = np.stack((image,) * 3, axis=-1) if len(image.shape) < 3 else image
        if images[i].shape != (height, width, 3):
            up_points = (width, height)
            images[i] = cv2.resize(images[i], up_points, interpolation=cv2.INTER_AREA)
        image = images[i]

        if with_titles:
            cv2.putText(image, titles[i],
                        position,
                        font,
                        fontScale,
                        fontColor,
                        thickness,
                        lineType)

    num_images = len(images)
    s = int(math.sqrt(num_images)) if force_row_size is None else force_row_size

    # Append empty images
    empty_images = s * math.ceil(num_images / s) - num_images
    [images.append(np.zeros_like(images[0])) for _ in range(empty_images)]

    def join_images(ims, horizontal=1):
        return np.concatenate(tuple(ims), axis=1 if horizontal else 0)

    split_arrays = np.array_split(images, s)
    vert_arrays = []
    for arr in split_arrays:
        vert_arrays.append(join_images(arr, horizontal=1))

    final_image = join_images(vert_arrays, horizontal=0)

    return final_image


# Get HSV filter
def get_hsv_filter(image, lower, upper):
    # Extract chess_handler-board lines
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    msk = cv2.inRange(hsv, lower, upper)
    krn = cv2.getStructuringElement(cv2.MORPH_RECT, (50, 30))
    dlt = cv2.dilate(msk, krn, iterations=5)
    res = cv2.bitwise_and(dlt, msk)
    res = np.uint8(res)
    return res


# Get canny image based on hsv lower and upper bounds
def get_hsv_canny(image, lower, upper):
    # Extract chess_handler-board lines
    res = get_hsv_filter(image, lower, upper)
    res1 = cv2.Canny(res, 38, 38 * 3)
    return res1


# Get center points of chessboard squares based on canny image
def get_center_points(canny, lower_area=4500, upper_area=5650, draw_points=False, draw_contours=False, image=None):
    processed_img = cv2.GaussianBlur(canny, (5, 5), 0)
    _, processed_img = cv2.threshold(processed_img, 30, 255, cv2.THRESH_BINARY)
    # find contours
    contours, hierarchy = cv2.findContours(processed_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    center_points = []
    corner_points = []
    for c in contours:
        area = cv2.contourArea(c)
        if lower_area < area < upper_area:
            (x, y, w, h) = cv2.boundingRect(c)
            corner_1 = (x, y)
            corner_2 = (x, y + h)
            corner_3 = (x + w, y)
            corner_4 = (x + w, y + h)
            corner_points.append(corner_1)
            corner_points.append(corner_2)
            corner_points.append(corner_3)
            corner_points.append(corner_4)
            center_point = (int(x + w / 2), int(y + h / 2))
            center_points.append(center_point)
            if draw_points:
                cv2.circle(image, center_point, 2, (0, 255, 0))
                cv2.circle(image, corner_1, 3, (34, 127, 127))
                cv2.circle(image, corner_2, 3, (34, 127, 127))
                cv2.circle(image, corner_3, 3, (34, 127, 127))
                cv2.circle(image, corner_4, 3, (34, 127, 127))
            if draw_contours:
                cv2.rectangle(image, (x, y), (x + w, y + h), (255, 0, 255), 2)
    return center_points, corner_points, processed_img


# Get distance between points
def get_distance_between_points(point1, point2):
    # d=???((x2 ??? x1)?? + (y2 ??? y1)??).
    x1, y1 = point1
    x2, y2 = point2
    return math.sqrt(math.pow(x2 - x1, 2) + math.pow(y2 - y1, 2))


# Reduce points if close to each other
def reduce_points(points):
    points_sorted = sorted(points, key=lambda x: (x[0], x[1]))
    groups = []
    while len(points_sorted) != 0:
        current_point = points_sorted.pop()
        group = [current_point]
        points_to_remove = []
        for p in points_sorted:
            if get_distance_between_points(current_point, p) < 25:
                group.append(p)
                points_to_remove.append(p)
        for p in points_to_remove:
            points_sorted.remove(p)
        groups.append(group)
    points = [g[0] for g in groups]
    points = sorted(points, key=lambda x: (x[0], x[1]))
    return points


# Chessboard corner points to 2d matrix
def corner_points_to_2d_matrix(points, size=9):
    matrix = [points[i:i + size] for i in range(0, len(points), size)]
    for i, _ in enumerate(matrix):
        matrix[i] = sort_array_of_points(matrix[i])
    return matrix


# Cheessboard corner points to squares/corners coordinates
def corner_points_to_squares(points, size=8, with_text=False, image=None):
    counter = 0
    font = cv2.FONT_HERSHEY_SIMPLEX
    fontScale = 0.5
    fontColor = (0, 255, 0)
    thickness = 2
    lineType = 1

    squares = []
    for i in range(size):
        for j in range(size):
            square = [points[i][j], points[i + 1][j], points[i + 1][j + 1], points[i][j + 1]]
            squares.append(square)
            if with_text:
                cv2.putText(image, str(counter),
                            square[3],
                            font,
                            fontScale,
                            fontColor,
                            thickness,
                            lineType)
                counter += 1
    return squares


def is_piece_found(img, threshold=200):
    cnts = cv2.findContours(img.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    if cnts:
        max_area = max([cv2.contourArea(c) for c in cnts])
        if max_area < threshold:
            return 0
    else:
        return 0
    return 1



def get_pieces_diff(imgA, imgB, hsv_white_pieces_min, hsv_white_pieces_max,
                    hsv_black_pieces_min, hsv_black_pieces_max):
    imgA_copy = imgA.copy()
    imgA_copy = cv2.cvtColor(imgA_copy, cv2.COLOR_BGR2HSV)
    imgA_filter_white = cv2.inRange(imgA_copy, hsv_white_pieces_min, hsv_white_pieces_max)
    imgA_filter_black = cv2.inRange(imgA_copy, hsv_black_pieces_min, hsv_black_pieces_max)

    is_A_white = is_piece_found(imgA_filter_white) != 0
    is_A_black = is_piece_found(imgA_filter_black) != 0
    is_A_empty = is_piece_found(imgA_filter_white) == 0 and is_piece_found(imgA_filter_black) == 0

    imgBcopy = imgB.copy()
    imgBcopy = cv2.cvtColor(imgBcopy, cv2.COLOR_BGR2HSV)
    imgB_filter_white = cv2.inRange(imgBcopy, hsv_white_pieces_min, hsv_white_pieces_max)
    imgB_filter_black = cv2.inRange(imgBcopy, hsv_black_pieces_min, hsv_black_pieces_max)
    is_B_white = is_piece_found(imgB_filter_white) != 0
    is_B_black = is_piece_found(imgB_filter_black) != 0
    is_B_empty = is_piece_found(imgB_filter_white) == 0 and is_piece_found(imgB_filter_black) == 0

    if (is_A_white and is_B_white) or (is_A_black and is_B_black) or (is_A_empty and is_B_empty):
        return 0
    return 1


# Get difference between each chessboard square
def get_each_square_diff(imageA, imageB, squares, threshold=0.6, show_box=False, image=None, **kwargs):
    squares_with_differences = []
    squares_to_show = []
    squares_to_show_titles = []

    scores = []
    for i in range(len(squares)):
        s = squares[i]
        x1, y1 = s[0]
        x2, y2 = s[2]

        square1 = imageA[y1:y2, x1:x2]
        square2 = imageB[y1:y2, x1:x2]

        # score = get_images_diff_histograms(square1, square2)

        score = get_pieces_diff(square1, square2,
                                kwargs['hsv_white_pieces_min'],
                                kwargs['hsv_white_pieces_max'],
                                kwargs['hsv_black_pieces_min'],
                                kwargs['hsv_black_pieces_max'])
        squares_to_show.append(square1)
        squares_to_show.append(square2)
        squares_to_show_titles.append(f"{i} {str(score)} A")
        squares_to_show_titles.append(f"{i} {str(score)} B")
        scores.append(score)

    scores, squares = zip(*sorted(zip(scores, squares), reverse=True))

    main_indices = [i for i in range(len(scores)) if scores[i] == 1]
    main_scores = [scores[i] for i in main_indices]
    main_squares = [squares[i] for i in main_indices]

    for i, score in enumerate(main_scores):

        s = main_squares[i]
        x1, y1 = s[0]
        x2, y2 = s[2]

        square1 = imageA[y1:y2, x1:x2]
        square2 = imageB[y1:y2, x1:x2]
        squares_with_differences.append(s)

        if show_box:
            cv2.rectangle(image, (x1, y1), (x2, y2), (0, 0, 255), 2)

    squares_differences_images = concat_images(squares_to_show, squares_to_show_titles, with_titles=False)

    return squares_with_differences, squares_differences_images


# Deduce move made based on square differences
def get_squares_changed(squares_with_differences, matrix_2d):
    move_index = []
    for d in squares_with_differences:
        def index_2d(data, search):
            for i, e in enumerate(data):
                try:
                    return i, e.index(search)
                except ValueError:
                    pass

        i, j = index_2d(matrix_2d, d[0])
        move_index.append([i, j])
    return move_index


# Get squares and coordinates from image
def get_image_information(image, image_write, hsv_black_squares_min, hsv_black_squares_max, hsv_white_squares_min, hsv_white_squares_max, board_percentage=0.8):
    # Get HSV canny for White and Black squares and OR them
    res_b = get_hsv_canny(image, hsv_black_squares_min, hsv_black_squares_max)
    res_w = get_hsv_canny(image, hsv_white_squares_min, hsv_white_squares_max)
    res_bw = cv2.bitwise_or(res_b, res_w)

    # Estimate minium area of square
    image_area = image.shape[0] * image.shape[1]
    min_square_area = image_area * board_percentage / 64
    max_square_area = min_square_area * 1.3
    logger.debug(f"Square area between {min_square_area} and {max_square_area}")

    # Get center points, corner points and processed image
    center_points, corner_points, processed_image = get_center_points(res_bw, lower_area=min_square_area,
                                                                      upper_area=max_square_area,
                                                                      draw_points=True,
                                                                      draw_contours=True, image=image_write)
    # Reduce points
    corner_points = reduce_points(corner_points)
    center_points = reduce_points(center_points)
    logger.debug(f"Number of center points: {len(center_points)}, Number of corner points {len(corner_points)}")
    # assert len(center_points) == 64
    # assert len(corner_points) == 81
    squares = None
    matrix_2d = None
    if len(corner_points) == 81:
        matrix_2d = corner_points_to_2d_matrix(corner_points)
        squares = corner_points_to_squares(matrix_2d, with_text=True, image=image_write)

    return squares, matrix_2d, res_b, res_w, res_bw, processed_image


# Sort array of points based on x then y
def sort_array_of_points(points):
    # TODO: Doesnt work if not similar x for resorting in y
    points = sorted(points, key=lambda x: x[0])
    points = sorted(points, key=lambda x: x[1])
    return points


def get_four_corners(points):
    points_x = sorted(points, key=lambda x: x[0])
    top_points = points_x[0:2]
    bottom_points = points_x[2:4]
    sorted_top_points = sorted(top_points, key=lambda x: x[1])
    sorted_bottom_points = sorted(bottom_points, key=lambda x: x[1])
    point_1 = sorted_top_points[0]
    point_2 = sorted_top_points[1]
    point_3 = sorted_bottom_points[1]
    point_4 = sorted_bottom_points[0]
    return [point_1, point_2, point_3, point_4]
