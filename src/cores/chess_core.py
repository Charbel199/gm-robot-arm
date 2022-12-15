from chess_handler.chess_engine import ChessEngine
import os
from logger.log import LoggerService
import cv2
import numpy as np
from utils.buzz import play_sound
import math

logger = LoggerService.get_instance()


class ChessCore:
    def __init__(self, engine_side, initial_time=360, time_increment=0, is_simulation=True, with_sound=False):
        logger.info(f'Launching Chess Core')

        self.user_timer = initial_time
        self.engine_timer = initial_time
        self.time_increment = time_increment
        self.game_started = False
        self.initial_switch = False

        self.user_turn = engine_side == "BLACK"
        self.with_sound = with_sound
        self.engine_side = engine_side
        self.user_side = "WHITE" if engine_side == "BLACK" else "BLACK"
        self.engine_is_white = self.engine_side == "WHITE"
        logger.info(f'User side: {self.user_side} - Engine side: {self.engine_side}')

        self.engine = ChessEngine(os.environ.get("STOCKFISH_PATH"), elo_rating=os.environ.get("ELO_RATING"),
                                  engine_side=self.engine_side)
        self.is_simulation = is_simulation
        self.engine.check_stockfish_health()
        self.current_board = self.get_board()
        logger.info(f'Stockfish engine with {os.environ.get("ELO_RATING")} ELO rating launched ...')

        # self.fake_moves_black = ["e7e5", "e5d4"]
        # self.fake_moves_white = ["e2e4", "d2d4", "d1d4"]

        self.fake_moves_black = ["e7e5", "f8c5", "d8f6", "g7g5", "f6f2"]
        self.fake_moves_white = ["e2e4", "g1f3", "f1c4", "d2d3", "f3g5"]

    def switch_turn(self):
        self.game_started = True
        # Increment timer
        # Do not increment if it's the first switch
        if self.initial_switch:
            if self.user_turn:
                self.user_timer += self.time_increment
            else:
                self.engine_timer += self.time_increment
        else:
            self.initial_switch = True

        # Switch sides
        self.user_turn = not self.user_turn

        # Play sound
        if self.with_sound:
            play_sound()

    def check_if_checkmate(self):
        return self.engine.check_if_checkmate()

    def check_if_check(self):
        return self.engine.check_if_check()

    def get_clock(self):
        mins1, secs1 = divmod(self.user_timer, 60)
        timer1 = f"{int(mins1)}:{'' if secs1 > 10 else '0'}{math.floor(secs1)}"
        mins2, secs2 = divmod(self.engine_timer, 60)
        timer2 = f"{int(mins2)}:{'' if secs2 > 10 else '0'}{math.floor(secs2)}"
        # logger.debug(f"User time : {timer1} - Engine time: {timer2}")

        clock = 255 * np.ones(shape=[512, 512, 3], dtype=np.uint8)
        if self.game_started:
            if self.user_turn:
                clock[:, :256] = (255, 255, 0)
            else:
                clock[:, 256:] = (255, 255, 0)

        if self.check_if_checkmate():
            clock[:, :] = (0, 0, 255)
            cv2.putText(clock, text='YOU CHECKMATED GM ARM' if self.user_turn else 'GM ARM DESTROYED YOU',
                        org=(120, 300),
                        fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.7, color=(0, 0, 0),
                        thickness=1, lineType=cv2.LINE_AA)
            return clock

        cv2.putText(clock, text=timer1, org=(128, 256),
                    fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=1, color=(0, 0, 0),
                    thickness=2, lineType=cv2.LINE_AA)
        cv2.putText(clock, text='User timer', org=(80, 370),
                    fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.7, color=(0, 0, 0),
                    thickness=1, lineType=cv2.LINE_AA)

        cv2.putText(clock, text=timer2, org=(300, 256),
                    fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=1, color=(0, 0, 0),
                    thickness=2, lineType=cv2.LINE_AA)
        cv2.putText(clock, text='Engine timer', org=(300, 370),
                    fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.7, color=(0, 0, 0),
                    thickness=1, lineType=cv2.LINE_AA)

        return clock

    def get_board(self):
        return self.engine.get_board(white_side=self.engine_is_white)

    def get_board_image(self):
        return self.engine.visualizer.get_board_image()

    def deduce_move_from_squares(self, squares):
        if not self.is_simulation:
            next_best_move = self.engine.deduce_move(squares)
        else:
            next_best_move = self.fake_moves_white.pop(0) if self.engine_side == "BLACK" else self.fake_moves_black.pop(
                0)

        return next_best_move

    def get_move_commands(self, move):
        return self.engine.get_move_commands(move)

    def update_board(self, move) -> None:
        logger.info(f"Updating board with move: {move}")
        self.engine.make_move(move)
        self.current_board = self.get_board()

    def get_next_best_move(self) -> str:
        if not self.is_simulation:
            next_best_move = self.engine.get_next_best_move()
        else:
            next_best_move = self.fake_moves_black.pop(0) if self.engine_side == "BLACK" else self.fake_moves_white.pop(
                0)

        logger.info(f"Next best move is {next_best_move}")
        return next_best_move
