from stockfish import Stockfish


class ChessEngine():
    def __init__(self, stockfish_path, elo_rating=1300) -> None:
        self.stockfish = Stockfish(path=stockfish_path)
        self.stockfish.set_elo_rating(elo_rating)

    def check_stockfish_health(self):
        valid = self.stockfish.is_fen_valid("rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1")
        return valid

    def make_move(self, move):
        if (self.stockfish.is_move_correct(move)):
            self.stockfish.make_moves_from_current_position([move])
        else:
            print(f"Move {move} is not valid.")
    
    def get_board(self,white_side = True):
        return self.stockfish.get_board_visual(white_side)

    def get_next_best_move(self):
        return self.stockfish.get_best_move()


if __name__ == "__main__":
    stockfish_path = "/home/charbel199/projs/gm-robot-arm/src/assets/engine/stockfish"
    engine = ChessEngine(stockfish_path)
    print(engine.get_board())
    print(engine.get_next_best_move())