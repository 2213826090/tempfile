from os import sys, path
import os
for p in sys.path:
    if "PyUiApiTests" in p:
        sys.path.append(path.dirname(path.dirname(p)))
# deactivating screen recording, it may cause problems from unstable releases
# os.environ["RECORD_PYUIAPITESTS"] = "SET"
# os.environ["RECORDING_DELAY_PYUIAPITESTS"] = "15"