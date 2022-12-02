from cores.chess_core import ChessCore
from cores.vision_core import VisionCore
from cores.control_core import ControlCore
from dotenv import load_dotenv, find_dotenv
from pynput.keyboard import Key
from pynput import keyboard
import numpy as np
import cv2
from utils.cv_utils import concat_images
from utils.hsv_utils import parse_hsv_json
from logger.log import LoggerService
from rosserial_msgs.msg import Moves
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import time

logger = LoggerService.get_instance()


class GMCore:
    def __init__(self, engine_side="BLACK", use_camera=False, is_simulation=False, with_sound=False, use_robot=False):
        logger.info(f'Launching GM Core')
        # Print instructions
        logger.info(self.get_instructions())
        self.use_camera = use_camera
        self.engine_side = engine_side
        self.is_simulation = is_simulation
        self.with_sound = with_sound
        self.use_robot = use_robot

        hsv_values = parse_hsv_json("assets/hsv/gm-colors2.json")
        # Board squares
        self.hsv_white_squares_min = hsv_values['hsv_white_squares_min']
        self.hsv_white_squares_max = hsv_values['hsv_white_squares_max']
        self.hsv_black_squares_min = hsv_values['hsv_black_squares_min']
        self.hsv_black_squares_max = hsv_values['hsv_black_squares_max']
        # Red markers
        self.hsv_markers_min = hsv_values['hsv_markers_min']
        self.hsv_markers_max = hsv_values['hsv_markers_max']
        # Chess pieces
        self.hsv_white_pieces_min = hsv_values['hsv_white_pieces_min']
        self.hsv_white_pieces_max = hsv_values['hsv_white_pieces_max']
        self.hsv_black_pieces_min = hsv_values['hsv_black_pieces_min']
        self.hsv_black_pieces_max = hsv_values['hsv_black_pieces_max']

        load_dotenv(find_dotenv())

        self.chess_core: ChessCore = None
        self.vision_core: VisionCore = None
        self.control_core = rospy.Publisher('/control/move', Moves, queue_size=10)
        self.initialize_cores()

        self.bridge = CvBridge()
        self.camera = rospy.Subscriber("/rgb/image_raw", Image, self._camera_callback)

        listener = keyboard.Listener(
            on_press=self.on_key_press)
        listener.start()
        logger.info(f'Keyboard listeners started ...')
        rospy.init_node('GM_core')

    def _camera_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.vision_core.temp_camera_image = cv_image
        except CvBridgeError as e:
            logger.debug(f"Error in CV bridge {e}")

    def initialize_cores(self):
        logger.info(f'Initializing cores')
        self.vision_core = VisionCore(self.hsv_white_squares_min, self.hsv_white_squares_max,
                                      self.hsv_black_squares_min, self.hsv_black_squares_max,
                                      self.hsv_markers_min, self.hsv_markers_max,
                                      self.hsv_white_pieces_min, self.hsv_white_pieces_max,
                                      self.hsv_black_pieces_min, self.hsv_black_pieces_max,
                                      use_camera=self.use_camera)
        self.chess_core = ChessCore(engine_side=self.engine_side, is_simulation=self.is_simulation,
                                    with_sound=self.with_sound,
                                    time_increment=5)
        self._toggle_visualize_last_move = False
        self._toggle_visualize_all_images = False
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
            elif key.char == 'v':
                if self._toggle_visualize_last_move:
                    self._toggle_visualize_last_move = False
                self._toggle_visualize_all_images = not self._toggle_visualize_all_images
                self.vision_core.visualize_all_images()
            elif key.char == 'b':
                if self._toggle_visualize_all_images:
                    self._toggle_visualize_all_images = False
                self._toggle_visualize_last_move = not self._toggle_visualize_last_move
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
        if not self.vision_core.is_calibrated:
            logger.info(f"Calibration is required first")
            return
        self.vision_core.capture_initial_chessboard_layout()


    def send_move(self, move):
        self.control_core.publish(move[0], move[1])

    def on_user_move(self):
        if not self.vision_core.is_calibrated and not self.vision_core.captured_initial_board_layout:
            logger.info(f"Calibration and initial board layout capture are required first")
            return
        if not self.chess_core.user_turn:
            logger.warn(f"WARNING: Engine turn")
            return
        self.vision_core.update_images()
        user_squares_changed = self.vision_core.get_user_squares_changed()
        user_move = self.chess_core.deduce_move_from_squares(user_squares_changed)
        # Update move based on positions
        self.chess_core.update_board(user_move)
        self.chess_core.switch_turn()

        # Wait x ms
        # self.on_robot_move()
        # self.chess_core.user_side = True

    def on_robot_move(self):
        if not self.vision_core.is_calibrated and not self.vision_core.captured_initial_board_layout:
            logger.info(f"Calibration and initial board layout capture are required first")
            return
        if self.chess_core.user_turn:
            logger.warn(f"WARNING: User turn")
            return
        arm_move = self.chess_core.get_next_best_move()
        move_commands = self.chess_core.get_move_commands(arm_move)

        if self.use_robot:
            # Send move to arm
            rospy.set_param('/control/move_complete_counter', len(move_commands))
            for move_command in move_commands:
                self.send_move(move_command)
            # self.chess_core.update_board(arm_move)
            # WAIT UNTIL MOVE IS COMPLETELY DONE
            while (rospy.get_param('/control/move_complete_counter') != 0):
                pass
        time.sleep(1)
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

            # Check images to add
            self.vision_core.images_to_show = []
            if self._toggle_visualize_last_move:
                self.vision_core.visualize_last_move()

            if self._toggle_visualize_all_images:
                self.vision_core.visualize_all_images()

            # Images to show
            images_to_show = [self.chess_core.get_board_image(), clock]
            if self.vision_core.temp_camera_image is not None:
                images_to_show.append(self.vision_core.temp_camera_image)
            if self.vision_core.debug_images is not None:
                # images_to_show.extend(self.vision_core.debug_images)
                pass
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
