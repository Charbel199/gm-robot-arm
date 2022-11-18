from stockfish import Stockfish
from chess_handler.chess_visualizer import ChessVisualizer
from logger.log import LoggerService

logger = LoggerService.get_instance()


class ChessEngine:
    def __init__(self, stockfish_path, elo_rating=1300, engine_side="BLACK") -> None:
        self.engine_side = engine_side
        self.user_side = "WHITE" if engine_side == "BLACK" else "BLACK"
        self.visualizer = ChessVisualizer(engine_side=engine_side)
        self.stockfish = Stockfish(path=stockfish_path)
        self.stockfish.set_elo_rating(elo_rating)

    def check_stockfish_health(self):
        valid = self.stockfish.is_fen_valid("rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1")
        return valid

    def make_move(self, move):
        if (self.stockfish.is_move_correct(move)):
            self.stockfish.make_moves_from_current_position([move])
            self.visualizer.make_move(move)
        else:
            logger.info(f"Move {move} is not valid.")

    def get_board(self, white_side=True):
        return self.stockfish.get_board_visual(white_side)

    def get_board_image(self):
        return self.visualizer.get_board_image()

    def get_next_best_move(self):
        return self.stockfish.get_best_move()

    def get_piece(self, position):
        return self.stockfish.get_what_is_on_square(position)

    def get_type_of_move(self, move):
        return self.stockfish.will_move_be_a_capture(move)

    def deduce_move(self, squares):
        logger.info(f"Deducing moves from squares: {squares}")
        pieces = []
        for s in squares:
            piece_color = self.stockfish.get_what_is_on_square(s).name.split('_')[
                0] if self.stockfish.get_what_is_on_square(s) is not None else None
            piece_type = self.stockfish.get_what_is_on_square(s).name.split('_')[
                1] if self.stockfish.get_what_is_on_square(s) is not None else None
            pieces.append(
                (s, piece_color, piece_type)
            )

        empty_squares = [piece[0] for piece in pieces if piece[1] is None]
        user_squares = [piece[0] for piece in pieces if piece[1] == self.user_side]
        engine_squares = [piece[0] for piece in pieces if piece[1] == self.engine_side]
        # If 2
        if len(pieces) == 2:
            # Take
            if len(empty_squares) == 0:
                logger.info(f"Move - Taking: {user_squares[0]}{engine_squares[0]}")
                return f"{user_squares[0]}{engine_squares[0]}"

            # Move
            if len(empty_squares) == 1:
                logger.info(f"Move - Moving: {user_squares[0]}{empty_squares[0]}")
                return f"{user_squares[0]}{empty_squares[0]}"

        # If 3
        if len(pieces) == 3:
            # En passant
            logger.info(f"Move - En Passant: {user_squares[0]}{empty_squares[0]}")
            return f"{user_squares[0]}{empty_squares[0]}"

        # If 4
        if len(pieces) == 4:
            king_square = [piece[0] for piece in user_squares if piece[2] == 'KING'][0]
            empty_castle_square = [piece[0] for piece in empty_squares if "c" or "g" in piece[0]][0]

            logger.info(f"Move - Castling: {king_square}{empty_castle_square}")
            # Castle
            return f"{king_square}{empty_castle_square}"

        return


if __name__ == "__main__":
    stockfish_path = "/home/charbel199/projs/gm-robot-arm/src/assets/engine/stockfish"
    engine = ChessEngine(stockfish_path)
    print(engine.get_board())
    print(engine.get_next_best_move())
    engine.make_move("d2d4")
    print(engine.get_board())
