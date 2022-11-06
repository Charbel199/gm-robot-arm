from skimage.metrics import structural_similarity as compare_ssim
import cv2
import imutils
import numpy as np
import math


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
def concat_images(images, titles, with_titles=True):
    font = cv2.FONT_HERSHEY_SIMPLEX
    position = (15, 35)
    fontScale = 1
    fontColor = (0, 255, 0)
    thickness = 2
    lineType = 1
    for i, image in enumerate(images):
        # Make all images 3 channels
        images[i] = np.stack((image,) * 3, axis=-1) if len(image.shape) < 3 else image
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
    s = int(math.sqrt(num_images))

    # Append empty images
    empty_images = num_images % s
    [images.append(np.zeros_like(images[0])) for _ in range(empty_images)]

    def join_images(ims, horizontal=1):
        return np.concatenate(tuple(ims), axis=1 if horizontal else 0)

    split_arrays = np.array_split(images, s)
    vert_arrays = []
    for arr in split_arrays:
        vert_arrays.append(join_images(arr, horizontal=1))

    final_image = join_images(vert_arrays, horizontal=0)

    return final_image


# Get canny image based on hsv lower and upper bounds
def get_hsv_canny(image, lower, upper):
    # Extract chess-board lines
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    msk = cv2.inRange(hsv, lower, upper)
    krn = cv2.getStructuringElement(cv2.MORPH_RECT, (50, 30))
    dlt = cv2.dilate(msk, krn, iterations=5)
    res = cv2.bitwise_and(dlt, msk)
    res = np.uint8(res)
    res1 = cv2.Canny(res, 38, 38 * 3)
    return res1


# Get center points of chessboard squares based on canny image
def get_center_points(canny, lower_area=4500, upper_area=5650, draw_points=False, draw_contours=False, image=None):
    processed_img = cv2.GaussianBlur(canny, (5, 5), 0)
    _, processed_img = cv2.threshold(processed_img, 50, 255, cv2.THRESH_BINARY)
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
    # d=√((x2 – x1)² + (y2 – y1)²).
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


# Get difference between grayscale images
def get_images_diff(grayA, grayB, lower_area=1500, upper_area=5650, with_box=False, image=None):
    (score, diff) = compare_ssim(grayA, grayB, full=True)
    diff = (diff * 255).astype("uint8")
    thresh = cv2.threshold(diff, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)[1]
    cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    final_cnts = []
    for c in cnts:
        area = cv2.contourArea(c)
        if lower_area < area < upper_area:
            final_cnts.append(c)
            if with_box:
                (x, y, w, h) = cv2.boundingRect(c)
                cv2.rectangle(image, (x, y), (x + w, y + h), (0, 0, 255), 2)
    return score, diff, thresh, cnts


# Get difference between each chessboard square
def get_each_square_diff(imageA, imageB, squaresA, squaresB, threshold=0.6, show_box=False, image=None):
    grayA = cv2.cvtColor(imageA, cv2.COLOR_BGR2GRAY)
    grayB = cv2.cvtColor(imageB, cv2.COLOR_BGR2GRAY)
    squares_with_differences = []
    for i in range(len(squaresA)):
        sA = squaresA[i]
        sB = squaresB[i]
        x1, y1 = sA[0]
        x2, y2 = sA[2]
        square1 = grayA[y1:y2, x1:x2]
        x1, y1 = sB[0]
        x2, y2 = sB[2]
        square2 = grayB[y1:y2, x1:x2]

        print(f"Comparing {square1.shape} to {square2.shape}")
        if square1.shape != square2.shape:
            up_width = square2.shape[1]
            up_height = square2.shape[0]
            up_points = (up_width, up_height)
            square1 = cv2.resize(square1, up_points, interpolation=cv2.INTER_LINEAR)
        print(f"Comparing {square1.shape} to {square2.shape}")

        score, diff, thresh, cnts = get_images_diff(square1, square2, with_box=False, image=None)
        print(f"Score is {score}")
        if score < threshold:
            cv2.imshow(str(score), square1)
            cv2.imshow(str(score + 1), square2)
            squares_with_differences.append(sA)
            if show_box:
                cv2.rectangle(image, (x1, y1), (x2, y2), (0, 0, 255), 2)
    return squares_with_differences


# Deduce move made based on square differences
def get_move_made(squares_with_differences, matrix_2d):
    move_index = []
    for d in squares_with_differences:
        def index_2d(data, search):
            for i, e in enumerate(data):
                try:
                    return i, e.index(search)
                except ValueError:
                    pass

        i, j = index_2d(matrix_2d, d[0])
        move_index.append((i, j))
    return move_index


# Sort array of points based on x then y
def sort_array_of_points(points):
    points = sorted(points, key=lambda x: x[0])
    points = sorted(points, key=lambda x: x[1])
    return points
