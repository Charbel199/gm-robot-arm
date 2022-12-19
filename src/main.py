from cores.gm_core import GMCore
import os
import sys

sys.path.append('../')

USE_CAMERA = int(os.getenv("USE_CAMERA", "0")) # 1: Use camera / 0: Use Fake images in Vision Core
IS_SIMULATION = int(os.getenv("IS_SIMULATION", "1")) # 1: Use Fake moves in Chess core / 0: Use engine next best move
USE_ROBOT = int(os.getenv("USE_ROBOT", "0")) # 1: Call robot arm to perform move / 0: Do not call robot arm
WITH_SOUND = int(os.getenv("WITH_SOUND", "0")) # 1: Enable sound on turn switch / 0: Disable sound on turn switch
USE_PREVIOUS_CALIBRATED_BOARD = int(os.getenv("USE_PREVIOUS_CALIBRATED_BOARD", "0")) # 1: Use previously recorded empty board for calibration
ENGINE_SIDE = os.getenv("ENGINE_SIDE", "BLACK") # Engine Side: Black or White


core = GMCore(engine_side=ENGINE_SIDE, use_camera=USE_CAMERA, is_simulation=IS_SIMULATION, with_sound=WITH_SOUND,
              use_robot=USE_ROBOT, use_previous_calibrated_board=USE_PREVIOUS_CALIBRATED_BOARD)
core.spin()
