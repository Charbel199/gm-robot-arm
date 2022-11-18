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

    def get_move_commands(self, move):
        logger.info(f"Generating move commands from move: {move}")
        first_square = move[0:2]
        second_square = move[2:]
        first_square_piece = self.stockfish.get_what_is_on_square(first_square)
        second_square_piece = self.stockfish.get_what_is_on_square(second_square)
        commands = []
        first_square_row = first_square[1]
        first_square_column = first_square[0]
        second_square_row = second_square[1]
        second_square_column = second_square[0]
        # Normal capture
        if self.stockfish.will_move_be_a_capture(move) == self.stockfish.Capture.DIRECT_CAPTURE:
            commands.append(["PICK", second_square])
            commands.append(["YEET"])
            commands.append(["PICK", first_square])
            commands.append(["PLACE", second_square])
        # En passant
        if self.stockfish.will_move_be_a_capture(move) == self.stockfish.Capture.EN_PASSANT:

            piece_to_remove_square = f"{first_square_row}{second_square_column}"
            commands.append(["PICK", piece_to_remove_square])
            commands.append(["YEET"])
            commands.append(["PICK", first_square])
            commands.append(["PLACE", second_square])

        # If move
        if self.stockfish.will_move_be_a_capture(move) == self.stockfish.Capture.NO_CAPTURE:
            # If castle
            if 'KING' in first_square_piece.name and first_square_column == 'e' and (second_square_column == 'c' or second_square_column == 'g'):
                if second_square == "c1":
                    rook_square = "a1"
                    rook_empty_square = "d1"
                elif second_square == "g1":
                    rook_square = "h1"
                    rook_empty_square = "f1"
                elif second_square == "c8":
                    rook_square = "a8"
                    rook_empty_square = "d8"
                elif second_square == "g8":
                    rook_square = "h8"
                    rook_empty_square = "h8"
                commands.append(["PICK", first_square])
                commands.append(["PLACE", second_square])
                commands.append(["PICK", rook_square])
                commands.append(["PLACE", rook_empty_square])
            # If regular move
            else:
                commands.append(["PICK", first_square])
                commands.append(["PLACE", second_square])
        logger.info(f"Move commands are: {commands}")
        return commands

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
    engine.make_move("d7d6")

    engine.make_move("e2e3")
    engine.make_move("d6d5")
    print(engine.get_move_commands('f1d3'))
    engine.make_move("f1d3")
    engine.make_move("a7a6")

    engine.make_move("g1f3")
    engine.make_move("a6a5")
    # print(engine.get_board())
    # print("Getting move command")
    # engine.get_move_commands('e1g1')
    # engine.make_move("e1g1")
    # print(dir(engine.stockfish))
    # print(engine.get_board())

    engine.make_move("c1d2")
    engine.make_move("h7h6")

    engine.make_move("b1a3")
    engine.make_move("h6h5")

    engine.make_move("d1e2")
    engine.make_move("h5h4")
    print(engine.get_move_commands('e1c1'))

    print(engine.get_board())
    print(engine.get_move_commands('f3h4'))