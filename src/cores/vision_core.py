import cv2
from src.utils.cv_utils import concat_images, get_image_information, get_each_square_diff, get_move_made, \
    get_hsv_filter, get_four_corners
from src.logger.log import LoggerService
import numpy as np

logger = LoggerService.get_instance()


class VisionCore:
    def __init__(self,
                 hsv_min_b,
                 hsv_max_b,
                 hsv_min_w,
                 hsv_max_w,
                 hsv_min_marker,
                 hsv_max_marker):
        self.images = []

        self.empty_board_image = None
        self.last_user_move = None
        self.calibrated_image = None
        self.images_to_show = None

        self.height, self.width = 600, 600
        self.image_buffer_size = 6

        self.squares = None
        self.cached_marker_centers = None
        self.matrix_2d = None

        self.hsv_min_b = hsv_min_b
        self.hsv_max_b = hsv_max_b
        self.hsv_min_w = hsv_min_w
        self.hsv_max_w = hsv_max_w
        self.hsv_min_marker = hsv_min_marker
        self.hsv_max_marker = hsv_max_marker

        # empty_image = cv2.imread('src/assets/moves/empty_lichess2.png')
        # imageA = cv2.imread('src/assets/moves/lichess2_1.png')
        # imageB = cv2.imread('src/assets/moves/lichess2_2.png')
        # self.fake_images = [empty_image, imageA, imageB]

        empty_image = cv2.imread('src/assets/moves/real_board/Move_Empty2.jpeg')
        initial_image = cv2.imread('src/assets/moves/real_board/Move_Initial.jpeg')
        image1 = cv2.imread('src/assets/moves/real_board/Move1.jpeg')
        image2 = cv2.imread('src/assets/moves/real_board/Move2.jpeg')
        image3 = cv2.imread('src/assets/moves/real_board/Move3.jpeg')
        image4 = cv2.imread('src/assets/moves/real_board/Move4.jpeg')
        image5 = cv2.imread('src/assets/moves/real_board/Move5.jpeg')

        self.fake_images = [empty_image, initial_image, image1, image2, image3, image4, image5]

        self.chessboard_map = [['a8', 'a7', 'a6', 'a5', 'a4', 'a3', 'a2', 'a1'],
                               ['b8', 'b7', 'b6', 'b5', 'b4', 'b3', 'b2', 'b1'],
                               ['c8', 'c7', 'c6', 'c5', 'c4', 'c3', 'c2', 'c1'],
                               ['d8', 'd7', 'd6', 'd5', 'd4', 'd3', 'd2', 'd1'],
                               ['e8', 'e7', 'e6', 'e5', 'e4', 'e3', 'e2', 'e1'],
                               ['f8', 'f7', 'f6', 'f5', 'f4', 'f3', 'f2', 'f1'],
                               ['g8', 'g7', 'g6', 'g5', 'g4', 'g3', 'g2', 'g1']]

    def preprocess_image(self, image):
        res = get_hsv_filter(image, self.hsv_min_marker, self.hsv_max_marker)
        res = cv2.dilate(res, np.ones((3, 3), np.uint8))
        contours, hierarchy = cv2.findContours(res, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        marker_centers = []
        for c in contours:
            if cv2.contourArea(c) > 30:
                (x, y, w, h) = cv2.boundingRect(c)
                marker_centers.append([int(x + w / 2), int(y + h / 2)])

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
        self.images_to_show = self.images
        logger.info("Set images to show")

    def visualize_last_move(self):
        pass

    def capture_image(self):
        logger.info("Capturing image")
        image = None  # TODO: Get from camera
        image = self.fake_images.pop(0)
        image = self.preprocess_image(image)
        return image

    def update_images(self):
        logger.info("Updating images")
        if len(self.images) > self.image_buffer_size:
            self.images.pop(0)
        self.images.append(self.capture_image())

    def capture_initial_chessboard_layout(self):
        logger.info("Capturing initial board layout")
        self.update_images()  # Called when pieces are set
        logger.info("Done capturing initial board layout")

    def calibrate(self):
        logger.info("Calibrating board")
        self.empty_board_image = self.capture_image()  # Capture empty board and set it as
        image_write = self.empty_board_image.copy()

        self.squares, self.matrix_2d, calibration_b, calibration_w, calibration_bw, calibration_processed = get_image_information(
            self.empty_board_image, image_write, self.hsv_min_b, self.hsv_max_b,
            self.hsv_min_w, self.hsv_max_w, board_percentage=0.7)
        self.calibrated_image = image_write
        logger.info("Done calibrating board")

    def get_user_move(self) -> str:
        logger.info("Detecting user move")
        if self.images[-1].shape != self.images[-2].shape:
            # TODO: Warp and crop
            logger.debug(f"Reshaping self.current_image from {self.images[-1].shape} to {self.images[-2].shape}")
            up_width = self.images[-1].shape[1]
            up_height = self.images[-1].shape[0]
            up_points = (up_width, up_height)
            self.images[-2] = cv2.resize(self.images[-2], up_points, interpolation=cv2.INTER_LINEAR)

        # Difference between snapshots
        squares_with_differences, squares_differences_images = get_each_square_diff(self.images[-2],
                                                                                    self.images[-1],
                                                                                    self.squares,
                                                                                    threshold=0.4,
                                                                                    show_box=True,
                                                                                    # image=image_write
                                                                                    )


        move_index = get_move_made(squares_with_differences, self.matrix_2d)
        move_string = ''
        for move in move_index:
            move_string += f"{self.chessboard_map[move[0]][move[1]]}"

        self.last_user_move = move_string
        logger.info(f"User moved {move_string}")
        return move_string
