from cores.gm_core import GMCore
import os
import sys
sys.path.append('../')

USE_CAMERA = int(os.getenv("USE_CAMERA", "0"))

core = GMCore(use_camera=USE_CAMERA)
core.spin()
