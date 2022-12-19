from cores.gm_core import GMCore
import os
import sys

sys.path.append('../')

USE_CAMERA = int(os.getenv("USE_CAMERA", "0"))
IS_SIMULATION = int(os.getenv("IS_SIMULATION", "1"))
USE_ROBOT = int(os.getenv("USE_ROBOT", "0"))
WITH_SOUND = int(os.getenv("WITH_SOUND", "0"))
USE_PREVIOUS_CALIBRATED_BOARD = int(os.getenv("USE_PREVIOUS_CALIBRATED_BOARD", "0"))
ENGINE_SIDE = os.getenv("ENGINE_SIDE", "BLACK")


core = GMCore(engine_side=ENGINE_SIDE, use_camera=USE_CAMERA, is_simulation=IS_SIMULATION, with_sound=WITH_SOUND,
              use_robot=USE_ROBOT, use_previous_calibrated_board=USE_PREVIOUS_CALIBRATED_BOARD)
core.spin()
