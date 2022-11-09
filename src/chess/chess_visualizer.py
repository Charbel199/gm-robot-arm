import chess.svg
import numpy as np
from io import BytesIO
from PIL import Image
from cairosvg import svg2png
import cv2


class ChessVisualizer:
    def __init__(self, engine_side):
        self.engine_side = engine_side
        self.current_svg = None
        self.current_img = None
        self.create_board()

    def create_board(self):
        self.board = chess.Board()

    def make_move(self, move):
        self.board.push_uci(move)

    def get_board_image(self):
        self.current_svg = chess.svg.board(self.board, flipped=self.engine_side == "BLACK")
        png = svg2png(bytestring=self.current_svg)
        pil_img = Image.open(BytesIO(png)).convert('RGBA')
        self.current_img = cv2.cvtColor(np.array(pil_img), cv2.COLOR_RGBA2BGRA)
        return self.current_img

    def save_board(self, path):
        with open(path, 'w') as fh:
            fh.write(self.current_svg)
