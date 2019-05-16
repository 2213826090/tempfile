#######################################################################
#
# @filename:    AOSP_Setting_System_UpTime.py
# @description: check uptime of the DUT
# @author:      dpanday@intel.com
#
######################################################################


#Build in libraries
import sys
import time

# Used defined libraries
from testlib.base.base_utils import get_args
from testlib.base import base_step
from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.ui import ui_steps

# ############# Get parameters ############

globals().update(vars(get_args(sys.argv)))
args = {}
for entry in script_args:
    key, val = entry.split("=")
    args[key] = val
###############################


adb_steps.reboot(reboot_timeout=400,serial=serial)()


ui_steps.press_home(serial=serial)()
ui_steps.press_car(serial=serial)()
ui_steps.open_settings(serial=serial)()
ui_steps.click_button_common(view_to_find={"text":"System"},view_to_check={"textContains":"Languages"},serial=serial)()
ui_steps.click_button_common(view_to_find={"text":"About phone"},view_to_check={"text":"Status"},serial=serial)()
ui_steps.click_button_common(view_to_find={"text":"Status"},view_to_check={"text":"Up time"},serial=serial)()

uptime= ui_steps.wait_for_view_common(view_to_find={"text":"Up time"},second_view_to_find={"className":"android.widget.TextView"},position="down",retrieve_info=True,serial=serial)()["text"]

# checking for uptime not greater than 10 minutes
if sum(x * int(t) for x, t in zip([3600, 60, 1], uptime.split(":"))) < 600:
    print "pass"
else:
    print "fail"

#Teardown
ui_steps.press_home(serial=serial)()