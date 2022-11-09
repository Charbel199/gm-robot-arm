from src.chess.chess_engine import ChessEngine
import os


class ChessCore:
    def __init__(self, engine_side):
        self.engine_side = engine_side
        self.user_side = "WHITE" if engine_side == "BLACK" else "BLACK"
        self.engine_is_white = self.engine_side == "WHITE"

        self.engine = ChessEngine(os.environ.get("STOCKFISH_PATH"), elo_rating=os.environ.get("ELO_RATING"))
        self.engine.check_stockfish_health()
        self.current_board = self.get_board()

    def get_board(self):
        return self.engine.get_board(white_side=self.engine_is_white)

    def get_board_image(self):
        return self.engine.visualizer.get_board_image()

    def update_board(self, move) -> None:
        self.engine.make_move(move)
        self.current_board = self.get_board()

    def get_next_best_move(self) -> str:
        return self.engine.get_next_best_move()
