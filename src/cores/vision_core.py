import cv2
from utils.cv_utils import concat_images, get_image_information, get_each_square_diff, get_squares_changed, \
    get_hsv_filter, get_four_corners
from logger.log import LoggerService
import numpy as np
from typing import List

logger = LoggerService.get_instance()


class VisionCore:
    def __init__(self,
                 hsv_black_squares_min, hsv_black_squares_max,
                 hsv_white_squares_min, hsv_white_squares_max,
                 hsv_markers_min, hsv_markers_max,
                 hsv_white_pieces_min, hsv_white_pieces_max,
                 hsv_black_pieces_min, hsv_black_pieces_max,
                 use_camera=False):
        logger.info(f'Launching Vision Core')
        self.temp_camera_image = None
        self.debug_images = []
        self.images = []
        self.use_camera = use_camera
        self.is_calibrated = False
        self.captured_initial_board_layout = False

        self.empty_board_image = None
        self.last_user_squares_change = None
        self.calibrated_image = None
        self.images_to_show = None

        self.height, self.width = 600, 600
        self.image_buffer_size = 6

        self.squares = None
        self.cached_marker_centers = None
        self.cached_squares_differences_images = None
        self.cached_image_last_move = None
        self.matrix_2d = None

        self.hsv_black_squares_min = hsv_black_squares_min
        self.hsv_black_squares_max = hsv_black_squares_max
        self.hsv_white_squares_min = hsv_white_squares_min
        self.hsv_white_squares_max = hsv_white_squares_max
        self.hsv_markers_min = hsv_markers_min
        self.hsv_markers_max = hsv_markers_max
        self.hsv_white_pieces_min = hsv_white_pieces_min
        self.hsv_white_pieces_max = hsv_white_pieces_max
        self.hsv_black_pieces_min = hsv_black_pieces_min
        self.hsv_black_pieces_max = hsv_black_pieces_max

        # empty_image = cv2.imread('assets/moves/real_board/Move_Empty2.jpeg')
        # initial_image = cv2.imread('assets/moves/real_board/Move_Initial.jpeg')
        # image1 = cv2.imread('assets/moves/real_board/Move1.jpeg')
        # image2 = cv2.imread('assets/moves/real_board/Move2.jpeg')
        # image3 = cv2.imread('assets/moves/real_board/Move3.jpeg')
        # image4 = cv2.imread('assets/moves/real_board/Move4.jpeg')
        # image5 = cv2.imread('assets/moves/real_board/Move5.jpeg')
        #
        # self.fake_images = [empty_image, initial_image, image1, image2, image3, image4, image5]

        empty_image = cv2.imread('assets/moves/real_board/Move_Empty2.jpeg')
        initial_image = cv2.imread('assets/moves/real_board/colored/Move_Initial.jpeg')
        image1 = cv2.imread('assets/moves/real_board/colored/Move1.jpeg')
        image2 = cv2.imread('assets/moves/real_board/colored/Move2.jpeg')
        image3 = cv2.imread('assets/moves/real_board/colored/Move3.jpeg')
        image4 = cv2.imread('assets/moves/real_board/colored/Move4.jpeg')
        image5 = cv2.imread('assets/moves/real_board/colored/Move5.jpeg')
        image6 = cv2.imread('assets/moves/real_board/colored/Move_BeforeCastle.jpeg')
        image7 = cv2.imread('assets/moves/real_board/colored/Move_AfterCastle.jpeg')
        image8 = cv2.imread('assets/moves/real_board/colored/Move_AfterCastle_EnPassant.jpeg')
        real_board = cv2.imread('assets/moves/real_board/colored/Move_real.jpg')

        self.fake_images = [empty_image, initial_image, image1, image2, image3, image4, image5, image6, image7, image8]
        # self.fake_images = [real_board, real_board, real_board]

        self.chessboard_map = [['a8', 'a7', 'a6', 'a5', 'a4', 'a3', 'a2', 'a1'],
                               ['b8', 'b7', 'b6', 'b5', 'b4', 'b3', 'b2', 'b1'],
                               ['c8', 'c7', 'c6', 'c5', 'c4', 'c3', 'c2', 'c1'],
                               ['d8', 'd7', 'd6', 'd5', 'd4', 'd3', 'd2', 'd1'],
                               ['e8', 'e7', 'e6', 'e5', 'e4', 'e3', 'e2', 'e1'],
                               ['f8', 'f7', 'f6', 'f5', 'f4', 'f3', 'f2', 'f1'],
                               ['g8', 'g7', 'g6', 'g5', 'g4', 'g3', 'g2', 'g1'],
                               ['h8', 'h7', 'h6', 'h5', 'h4', 'h3', 'h2', 'h1']]

    def preprocess_image(self, image):
        res = get_hsv_filter(image, self.hsv_markers_min, self.hsv_markers_max)
        res = cv2.dilate(res, np.ones((3, 3), np.uint8))
        contours, hierarchy = cv2.findContours(res, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        marker_centers = []
        for c in contours:
            if cv2.contourArea(c) > 30:
                (x, y, w, h) = cv2.boundingRect(c)
                marker_centers.append([int(x + w / 2), int(y + h / 2)])
        logger.debug(f"Found {len(marker_centers)} markers")
        if len(marker_centers) == 4:
            marker_centers = get_four_corners(marker_centers)
            self.cached_marker_centers = marker_centers
        else:
            marker_centers = self.cached_marker_centers

        # to calculate the transformation matrix
        input_pts = np.float32(marker_centers)
        output_pts = np.float32([[0, 0],
                                 [0, self.height],
                                 [self.width, self.height],
                                 [self.width, 0]])

        # Compute the perspective transform
        transform = cv2.getPerspectiveTransform(input_pts, output_pts)

        # Apply the perspective transformation to the image
        out = cv2.warpPerspective(image, transform, (self.width, self.height))

        return out

    def visualize_all_images(self):
        try:
            self.images_to_show = [self.calibrated_image]
            self.images_to_show.extend(self.images.copy())
            if any(image is None for image in self.images_to_show):
                self.images_to_show = []
        except Exception:
            self.images_to_show = []

    def visualize_last_move(self):
        try:
            self.images_to_show = [self.calibrated_image]
            self.images_to_show.append(self.images[-2].copy())
            self.images_to_show.append(self.cached_image_last_move)
            self.images_to_show.append(self.cached_squares_differences_images)
            if any(image is None for image in self.images_to_show):
                self.images_to_show = []
        except Exception:
            self.images_to_show = []

    def capture_image(self):
        logger.info("Capturing image")
        if not self.use_camera:
            image = self.fake_images.pop(0)
        else:
            image = self.temp_camera_image
        image = self.preprocess_image(image)
        return image

    def update_images(self):
        logger.info("Updating images")
        if len(self.images) > self.image_buffer_size:
            self.images.pop(0)
        self.images.append(self.capture_image())

    def capture_initial_chessboard_layout(self):
        if self.captured_initial_board_layout:
            logger.info("Already captured initial board layout")
            return
        logger.info("Capturing initial board layout")
        self.update_images()  # Called when pieces are set
        self.captured_initial_board_layout = True
        logger.info("Done capturing initial board layout")

    def calibrate(self):
        if self.is_calibrated:
            logger.info("Already calibrated")
            return

        logger.info("Calibrating board")
        self.empty_board_image = self.capture_image()  # Capture empty board and set it as
        image_write = self.empty_board_image.copy()

        self.squares, self.matrix_2d, calibration_b, calibration_w, calibration_bw, calibration_processed = get_image_information(
            self.empty_board_image, image_write, self.hsv_black_squares_min, self.hsv_black_squares_max,
            self.hsv_white_squares_min, self.hsv_white_squares_max, board_percentage=0.7)
        self.debug_images = [calibration_w]
        self.debug_images.append(calibration_b)
        self.debug_images.append(calibration_bw)
        self.debug_images.append(calibration_processed)
        self.calibrated_image = image_write
        self.is_calibrated = True
        logger.info("Done calibrating board")

    def get_user_squares_changed(self) -> List:
        logger.info("Detecting user move")
        if self.images[-1].shape != self.images[-2].shape:
            # TODO: Warp and crop
            logger.debug(f"Reshaping self.current_image from {self.images[-1].shape} to {self.images[-2].shape}")
            up_width = self.images[-1].shape[1]
            up_height = self.images[-1].shape[0]
            up_points = (up_width, up_height)
            self.images[-2] = cv2.resize(self.images[-2], up_points, interpolation=cv2.INTER_LINEAR)

        # Difference between snapshots
        image_write = self.images[-1].copy()
        squares_with_differences, squares_differences_images = get_each_square_diff(self.images[-2],
                                                                                    self.images[-1],
                                                                                    self.squares,
                                                                                    threshold=0.4,
                                                                                    show_box=True,
                                                                                    image=image_write,
                                                                                    hsv_white_pieces_min=self.hsv_white_pieces_min,
                                                                                    hsv_white_pieces_max=self.hsv_white_pieces_max,
                                                                                    hsv_black_pieces_min=self.hsv_black_pieces_min,
                                                                                    hsv_black_pieces_max=self.hsv_black_pieces_max,
                                                                                    )

        self.cached_squares_differences_images = squares_differences_images
        self.cached_image_last_move = image_write
        squares_index = get_squares_changed(squares_with_differences, self.matrix_2d)
        last_user_squares_change = []
        for square in squares_index:
            last_user_squares_change.append(f"{self.chessboard_map[square[0]][square[1]]}")

        self.last_user_squares_change = last_user_squares_change
        logger.info(f"User changed {last_user_squares_change}")
        return last_user_squares_change
