from chess_handler.chess_engine import ChessEngine
import os
from logger.log import LoggerService

logger = LoggerService.get_instance()


class ChessCore:
    def __init__(self, engine_side, is_simulation=True):
        logger.info(f'Launching Chess Core')
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

        self.fake_moves = ["e7e5", "e5d4"]

    def get_board(self):
        return self.engine.get_board(white_side=self.engine_is_white)

    def get_board_image(self):
        return self.engine.visualizer.get_board_image()

    def deduce_move_from_squares(self, squares):
        return self.engine.deduce_move(squares)

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
            next_best_move = self.fake_moves.pop(0)

        logger.info(f"Next best move is {next_best_move}")
        return next_best_move
