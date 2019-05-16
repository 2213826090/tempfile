#######################################################################
#
# @filename:    AOSP_setting_system_serial_number.py
# @description: check serial number of the DUT
# @author:      dpanday@intel.com
#
######################################################################

#Build in libraries
import sys
import time

# Used defined libraries
from testlib.base.base_utils import get_args

from testlib.scripts.android.ui import ui_steps

import serial.tools.list_ports
from uiautomator import device as d

# ############# Get parameters ############

globals().update(vars(get_args(sys.argv)))
args = {}
for entry in script_args:
    key, val = entry.split("=")
    args[key] = val

ui_steps.press_home(serial=serial)()
ui_steps.press_car(serial=serial)()
ui_steps.open_settings(serial=serial)()
ui_steps.click_button_common(view_to_find={"text":"System"},view_to_check={"textContains":"Languages"},serial=serial)()
ui_steps.click_button_common(view_to_find={"text":"About phone"},view_to_check={"text":"Status"},serial=serial)()
ui_steps.click_button_common(view_to_find={"text":"Status"},view_to_check={"text":"Serial number"},serial=serial)()

#print the seria; number of the dveice
print ui_steps.wait_for_view_common(view_to_find={"text":"Serial number"},second_view_to_find={"className":"android.widget.TextView"},position="down",retrieve_info=True,serial=serial)()["text"]