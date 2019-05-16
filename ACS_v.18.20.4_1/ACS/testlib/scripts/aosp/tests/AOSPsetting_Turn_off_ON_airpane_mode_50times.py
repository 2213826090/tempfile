
#######################################################################
#
# @filename:    AOSPsetting_Turn_off_ON_airpane_mode_50times.py
# @description: Turn ON OFF  Airplanemode 50 times
# @author:      dpanday@intel.com
#
######################################################################

#Build in libraries
import sys

# Used defined libraries
from testlib.base.base_utils import get_args

from testlib.scripts.wireless.wifi_generic import wifi_generic_steps
f
from testlib.scripts.android.ui import ui_steps


# ############# Get parameters ############

globals().update(vars(get_args(sys.argv)))
args = {}
for entry in script_args:
    key, val = entry.split("=")
    args[key] = val


#main step
ui_steps.press_home(serial=serial)()

ui_steps.open_settings(serial=serial)()
ui_steps.click_button_common(view_to_find={"text":"Network & Internet"},view_to_check={"text":"Airplane mode"},serial=serial)()

for i in range(1, 50):
    wifi_generic_steps.set_airplane_mode(state="ON",serial = serial)()


#teardown

wifi_generic_steps.set_airplane_mode(state="OFF",serial = serial)()
