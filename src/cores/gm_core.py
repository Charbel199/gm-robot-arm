from cores.chess_core import ChessCore
from cores.vision_core import VisionCore
from cores.control_core import ControlCore
from dotenv import load_dotenv, find_dotenv
from pynput.keyboard import Key
from pynput import keyboard
import numpy as np
import cv2
from utils.cv_utils import concat_images
from logger.log import LoggerService
import time

logger = LoggerService.get_instance()


class GMCore:
    def __init__(self, engine_side="BLACK", use_camera=False, is_simulation=False, with_sound=False):
        logger.info(f'Launching GM Core')
        # Print instructions
        logger.info(self.get_instructions())
        self.use_camera = use_camera
        self.engine_side = engine_side
        self.is_simulation = is_simulation
        self.with_sound = with_sound

        # Lichess White and Blue board
        # self.hsv_min_b = np.array([0, 69, 0])
        # self.hsv_max_b = np.array([179, 245, 255])
        # self.hsv_min_w = np.array([0, 0, 146])
        # self.hsv_max_w = np.array([179, 66, 234])

        # Lounge brown and white board
        # self.hsv_min_b = np.array([0, 0, 122])
        # self.hsv_max_b = np.array([74, 87, 255])
        # self.hsv_min_w = np.array([7, 0, 0])
        # self.hsv_max_w = np.array([93, 255, 124])

        # Lounge black and white board
        self.hsv_min_b = np.array([0, 0, 0])
        self.hsv_max_b = np.array([27, 241, 90])
        self.hsv_min_w = np.array([0, 0, 84])
        self.hsv_max_w = np.array([176, 152, 255])

        # Red markers
        self.hsv_min_marker = np.array([0, 177, 240])
        self.hsv_max_marker = np.array([98, 255, 255])

        # White/Green chess pieces
        self.hsv_min_greenwhite = np.array([56, 121, 184])
        self.hsv_max_greenwhite = np.array([65, 255, 255])
        # Black/Purple chess pieces
        self.hsv_min_blackpurple = np.array([116, 164, 183])
        self.hsv_max_blackpurple = np.array([154, 255, 255])

        load_dotenv(find_dotenv())

        self.chess_core: ChessCore = None
        self.vision_core: VisionCore = None
        self.control_core: ControlCore = None
        self.initialize_cores()

        listener = keyboard.Listener(
            on_press=self.on_key_press)
        listener.start()
        logger.info(f'Keyboard listeners started ...')

    def initialize_cores(self):
        logger.info(f'Initializing cores')
        self.vision_core = VisionCore(self.hsv_min_b, self.hsv_max_b,
                                      self.hsv_min_w, self.hsv_max_w,
                                      self.hsv_min_marker, self.hsv_max_marker,
                                      self.hsv_min_greenwhite, self.hsv_max_greenwhite,
                                      self.hsv_min_blackpurple, self.hsv_max_blackpurple,
                                      use_camera=self.use_camera)
        self.control_core = ControlCore()
        self.chess_core = ChessCore(engine_side=self.engine_side, is_simulation=self.is_simulation,
                                    with_sound=self.with_sound,
                                    time_increment=5)
        logger.info(f'All cores have been initialized')

    def get_instructions(self):
        instructions = "INSTRUCTIONS \n" \
                       "================\n" \
                       "e: Calibrate on empty board\n" \
                       "i: Capture on initial board layout\n" \
                       "v: Visualize all positions\n" \
                       "b: Visualize last move analysis\n" \
                       "space: User move\n" \
                       "r: Robot move\n" \
                       "m: Perform random move\n" \
                       "q: Exit\n" \
                       "================\n"
        return instructions

    def on_key_press(self, key):

        if 'char' in dir(key):
            logger.debug(f'Key {key.char} was pressed')
            if key.char == 'e':
                self.on_empty_board()
            elif key.char == 'i':
                self.on_initial_board()
            # elif key.char == 'm':
            #     self.random_move()
            elif key.char == 'v':
                self.vision_core.visualize_all_images()
            elif key.char == 'b':
                self.vision_core.visualize_last_move()
            elif key.char == 'r':
                self.on_robot_move()
            else:
                return

        if key == Key.space:
            logger.debug(f"Key 'space' was pressed")
            self.on_user_move()
        if key == Key.esc:
            logger.debug(f"Key 'esc' was pressed")
            logger.info(f"Resetting GM Core")
            self.initialize_cores()

    def on_empty_board(self):
        self.vision_core.calibrate()

    def on_initial_board(self):
        if not self.vision_core._is_calibrated:
            logger.info(f"Calibration is required first")
            return
        self.vision_core.capture_initial_chessboard_layout()

    def random_move(self):
        logger.info("Performing automatic move")
        self.chess_core.update_board(self.chess_core.get_next_best_move())
        logger.info("Done performing automatic move")

    def on_user_move(self):
        if not self.vision_core._is_calibrated and not self.vision_core._captured_initial_board_layout:
            logger.info(f"Calibration and initial board layout capture are required first")
            return
        if not self.chess_core.user_turn:
            logger.warn(f"WARNING: Engine turn")
            return
        self.vision_core.update_images()
        user_squares_changed = self.vision_core.get_user_squares_changed()
        user_move = self.chess_core.deduce_move_from_squares(user_squares_changed)
        # TODO: Update move based on positions
        # ...
        self.chess_core.update_board(user_move)
        self.chess_core.switch_turn()

        # Wait x ms
        # self.on_robot_move()
        # self.chess_core.user_side = True

    def on_robot_move(self):
        if not self.vision_core._is_calibrated and not self.vision_core._captured_initial_board_layout:
            logger.info(f"Calibration and initial board layout capture are required first")
            return
        if self.chess_core.user_turn:
            logger.warn(f"WARNING: User turn")
            return
        arm_move = self.chess_core.get_next_best_move()
        move_commands = self.chess_core.get_move_commands(arm_move)
        # # TODO: Call control core to make the next move
        # # ...
        #
        # WAIT UNTIL MOVE IS COMPLETELY DONE
        self.vision_core.update_images()
        self.chess_core.update_board(arm_move)
        self.chess_core.switch_turn()

    def spin(self):
        logger.info(f'Spinning ...')
        start = time.time()

        while True:
            end = time.time()
            time_elapsed = end - start
            start = time.time()
            # Clock update
            if self.chess_core.game_started:
                if self.chess_core.user_turn:
                    self.chess_core.user_timer -= time_elapsed
                else:
                    self.chess_core.engine_timer -= time_elapsed
            clock = self.chess_core.get_clock()

            # Images to show
            images_to_show = [self.chess_core.get_board_image(), clock]
            if self.vision_core.images_to_show is not None:
                images_to_show.extend(self.vision_core.images_to_show)

            labels = [str(i) for i in range(len(images_to_show))]
            images = concat_images(images_to_show, labels)
            cv2.imshow('Images ', images)
            if cv2.waitKey(33) == ord('q'):
                logger.info("Terminating ... \n\n\n")
                break
            time.sleep(0.1)


if __name__ == "__main__":
    core = GMCore(use_camera=False)
    core.spin()
