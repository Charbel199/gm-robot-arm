from cores.gm_core import GMCore
import os
import sys
sys.path.append('../')

USE_CAMERA = int(os.getenv("USE_CAMERA", "0"))
IS_SIMULATION = int(os.getenv("IS_SIMULATION", "1"))

core = GMCore(use_camera=USE_CAMERA, is_simulation=IS_SIMULATION)
core.spin()
