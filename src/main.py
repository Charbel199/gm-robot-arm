from cores.gm_core import GMCore
import os
import sys

sys.path.append('../')

core = GMCore(use_camera=os.getenv('USE_CAMERA'))
core.spin()
