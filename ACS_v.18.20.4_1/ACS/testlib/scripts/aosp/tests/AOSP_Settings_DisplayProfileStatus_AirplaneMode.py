#Build in libraries
import sys
import time

# Used defined libraries
from testlib.base.base_utils import get_args
from testlib.base import base_step
from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.ui import ui_steps

from testlib.scripts.wireless.wifi_generic import wifi_generic_steps


# ############# Get parameters ############

globals().update(vars(get_args(sys.argv)))
args = {}
for entry in script_args:
    key, val = entry.split("=")
    args[key] = val
#####################################

#Run
wifi_generic_steps.set_airplane_mode(state="ON", serial=serial)()

#Verify Airplane mode in settings
ui_steps.press_home(serial=serial)()
ui_steps.open_settings(view_to_find={"text":"Airplane mode is on"},serial=serial)()

#TearDown
wifi_generic_steps.set_airplane_mode(state="OFF",serial = serial)()
