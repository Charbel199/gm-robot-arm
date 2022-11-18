from cores.gm_core import GMCore
import os
import sys
sys.path.append('../')

USE_CAMERA = int(os.getenv("USE_CAMERA", "0"))
IS_SIMULATION = int(os.getenv("IS_SIMULATION", "1"))
WITH_SOUND = int(os.getenv("WITH_SOUND", "0"))

core = GMCore(use_camera=USE_CAMERA, is_simulation=IS_SIMULATION, with_sound=WITH_SOUND)
core.spin()
