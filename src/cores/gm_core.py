from src.cores.chess_core import ChessCore
from src.cores.vision_core import VisionCore
from src.cores.control_core import ControlCore
from dotenv import load_dotenv, find_dotenv
from pynput.keyboard import Key
from pynput import keyboard
import numpy as np
import cv2
from src.logger.log import LoggerService

logger = LoggerService.get_instance()


class GMCore:
    def __init__(self):
        logger.info(f'Launching GM Core')
        # Print instructions
        logger.info(self.get_instructions())

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

        load_dotenv(find_dotenv())
        self.vision_core = VisionCore(hsv_min_b, hsv_max_b, hsv_min_w, hsv_max_w)
        self.control_core = ControlCore()
        self.chess_core = ChessCore(engine_side="WHITE")

        listener = keyboard.Listener(
            on_press=self.on_key_press)
        listener.start()
        logger.info(f'Keyboard listeners started ...')

    def get_instructions(self):
        instructions = "INSTRUCTIONS \n" \
                       "================\n" \
                       "e: Calibrate on empty board\n" \
                       "i: Capture on initial board layout\n" \
                       "m: Perform random move\n" \
                       "space: User move\n" \
                       "q: Exit\n" \
                       "================\n"
        return instructions

    def on_key_press(self, key):

        if 'char' in dir(key):
            logger.info(f'Key {key.char} was pressed')
            if key.char == 'e':
                self.on_empty_board()
            if key.char == 'i':
                self.on_initial_board()
            if key.char == 'm':
                self.random_move()
        if key == Key.space:
            logger.info(f"Key 'space' was pressed")
            self.on_user_move()

    def on_empty_board(self):
        self.vision_core.calibrate()

    def on_initial_board(self):
        self.vision_core.capture_initial_chessboard_layout()

    def random_move(self):
        logger.info("Performing automatic move")
        self.chess_core.update_board(self.chess_core.get_next_best_move())
        logger.info("Done performing automatic move")

    def on_user_move(self):
        self.vision_core.update_images()
        user_move = self.vision_core.get_user_move()
        # TODO: Update move based on positions
        # ...
        self.chess_core.update_board(user_move)

        arm_move = self.chess_core.get_next_best_move()

        # TODO: Call control core to make the next move
        # ...

        self.chess_core.update_board(arm_move)

    def spin(self):
        logger.info(f'Spinning ...')
        while True:
            cv2.imshow('Board ', self.chess_core.get_board_image())
            if cv2.waitKey(33) == ord('q'):
                logger.info("Terminating ... \n\n\n")
                break


if __name__ == "__main__":
    core = GMCore()
    core.spin()
