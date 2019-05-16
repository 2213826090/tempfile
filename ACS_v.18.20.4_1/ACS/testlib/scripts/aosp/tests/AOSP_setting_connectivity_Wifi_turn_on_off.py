#######################################################################
#
# @filename:    AOSP_setting_connectivity_Wifi_turn_on_off.py
# @description: Turn On/OFF Wi-Fi
# @author:      dpanday@intel.com
#
#######################################################################



#Build in libraries
import sys


# Used defined libraries
from testlib.base.base_utils import get_args
from testlib.scripts.wireless.wifi import wifi_steps
from testlib.scripts.android.ui import ui_steps
import time

# ############# Get parameters ############


globals().update(vars(get_args(sys.argv)))
args = {}
for entry in script_args:
    key, val = entry.split("=")
    args[key] = val

ui_steps.press_home(serial=serial)()
ui_steps.open_settings(serial=serial)()
ui_steps.click_button_common(view_to_find={"text":"Network & Internet"},view_to_check={"text":"VPN"},serial=serial)()

#check wifi state if its OFF if not , Tur it on
wifi_steps.set_from_wifi_settings(serial=serial)()

time.sleep(5)


wifi_steps.set_from_wifi_settings(state = "OFF",serial=serial)()

#Teardown: #check wifi state if its OFF if not , Tur it ON
ui_steps.click_button_common(view_to_find={"text":"OFF"},view_to_check={"text":"On"},serial=serial)()

ui_steps.press_home(serial=serial)()
