import cv2
from src.utils.cv_utils import concat_images, get_image_information, get_each_square_diff, get_move_made


class VisionCore:
    def __init__(self,
                 hsv_min_b,
                 hsv_max_b,
                 hsv_min_w,
                 hsv_max_w):
        self.current_image = None
        self.previous_image = None
        self.empty_board_image = None
        self.last_user_move = None

        self.squares = None
        self.matrix_2d = None

        self.hsv_min_b = hsv_min_b
        self.hsv_max_b = hsv_max_b
        self.hsv_min_w = hsv_min_w
        self.hsv_max_w = hsv_max_w

        empty_image = cv2.imread('src/assets/moves/empty_lichess2.png')
        imageA = cv2.imread('src/assets/moves/lichess2_1.png')
        imageB = cv2.imread('src/assets/moves/lichess2_2.png')
        self.fake_images = [empty_image, imageA, imageB]

        self.chessboard_map = [['a8', 'a7', 'a6', 'a5', 'a4', 'a3', 'a2', 'a1'],
                               ['b8', 'b7', 'b6', 'b5', 'b4', 'b3', 'b2', 'b1'],
                               ['c8', 'c7', 'c6', 'c5', 'c4', 'c3', 'c2', 'c1'],
                               ['d8', 'd7', 'd6', 'd5', 'd4', 'd3', 'd2', 'd1'],
                               ['e8', 'e7', 'e6', 'e5', 'e4', 'e3', 'e2', 'e1'],
                               ['f8', 'f7', 'f6', 'f5', 'f4', 'f3', 'f2', 'f1'],
                               ['g8', 'g7', 'g6', 'g5', 'g4', 'g3', 'g2', 'g1']]

    def capture_image(self):
        image = None  # TODO: Get from camera
        return self.fake_images.pop(0)

    def update_images(self):
        self.previous_image = self.current_image
        self.current_image = self.capture_image()

    def capture_initial_chessboard_layout(self):
        self.current_image = self.capture_image()  # Called when pieces are set

    def calibrate(self):
        self.empty_board_image = self.capture_image()  # Capture empty board and set it as
        image_write = self.empty_board_image.copy()

        self.squares, self.matrix_2d, calibration_b, calibration_w, calibration_bw, calibration_processed = get_image_information(
            self.empty_board_image, image_write, self.hsv_min_b, self.hsv_max_b,
            self.hsv_min_w, self.hsv_max_w)

    def get_user_move(self) -> str:
        if self.current_image.shape != self.previous_image.shape:
            # TODO: Warp and crop
            print(f"Reshaping self.current_image from {self.current_image.shape} to {self.previous_image.shape}")
            up_width = self.current_image.shape[1]
            up_height = self.current_image.shape[0]
            up_points = (up_width, up_height)
            self.previous_image = cv2.resize(self.previous_image, up_points, interpolation=cv2.INTER_LINEAR)

        # Difference between snapshots
        squares_with_differences, squares_differences_images = get_each_square_diff(self.previous_image,
                                                                                    self.current_image,
                                                                                    self.squares,
                                                                                    threshold=0.8,
                                                                                    show_box=True,
                                                                                    # image=image_write
                                                                                    )

        move_index = get_move_made(squares_with_differences, self.matrix_2d)
        move_string = ''
        for move in move_index:
            move_string += f"{self.chessboard_map[move[0]][move[1]]}"
            print(f"Move {self.chessboard_map[move[0]][move[1]]}")

        self.last_user_move = move_string
        return move_string
