#######################################################################
#
# @filename:    AOSP_Setting_System_UpTime.py
# @description: enable wifi preference
# @author:      dpanday@intel.com
#
######################################################################



#Build in libraries
import sys


# Used defined libraries
from testlib.base.base_utils import get_args

from testlib.scripts.android.ui import ui_steps

# ############# Get parameters ############

globals().update(vars(get_args(sys.argv)))
args = {}
for entry in script_args:
    key, val = entry.split("=")
    args[key] = val

#Setup

ui_steps.press_home(serial=serial)()
ui_steps.press_car(serial=serial)()

ui_steps.open_settings(serial=serial)()
ui_steps.click_button_common(view_to_find={"text":"Network & Internet"},view_to_check={"text":"VPN"},serial=serial)()

ui_steps.click_button_common(view_to_find={"className":"android.widget.TextView"},
                             second_view_to_find={"textContains":"Fi"},view_to_check={"text":"On"},serial=serial)()

ui_steps.click_button_common(view_to_find={"textContains":"preferences"},second_view_to_find={"resourceId":"android:id/title"},view_to_check={"text":"Open network notification"},
                             scroll=True, view_to_scroll={"scrollable": "true"},serial=serial)()



if not ui_steps.click_button_common(view_to_find = {"text":"Open network notification"}, second_view_to_find={"className": "android.widget.Switch", "text": "ON"},view_to_check = {"text": "OFF"},
                                    optional= True, serial=serial)():


    ui_steps.click_button_common(view_to_find={"text": "Open network notification"},
                                        second_view_to_find={"className":
                                                                 "android.widget.Switch", "text": "ON"},
                                        view_to_check={"text": "OFF"}, serial=serial)()
#main step

ui_steps.press_home(serial=serial)()
ui_steps.press_car(serial=serial)()

ui_steps.open_settings(serial=serial)()
ui_steps.click_button_common(view_to_find={"text":"Network & Internet"},view_to_check={"text":"VPN"},serial=serial)()

ui_steps.click_button_common(view_to_find={"className":"android.widget.TextView"},
                             second_view_to_find={"textContains":"Fi"},view_to_check={"text":"On"},serial=serial)()

ui_steps.click_button_common(view_to_find={"textContains":"preferences"},second_view_to_find={"resourceId":"android:id/title"},view_to_check={"text":"Open network notification"},
                             scroll=True, view_to_scroll={"scrollable": "true"},serial=serial)()

#if Open network notification switch is not enable , check the condition and enable the switch to
#  get the notification


if not ui_steps.click_button_common(view_to_find = {"text":"Open network notification"}, second_view_to_find={"className": "android.widget.Switch", "text": "OFF"},view_to_check = {"text": "ON"},optional= True, serial=serial)():

    ui_steps.click_button_common(view_to_find={"text": "Open network notification"},
                                 second_view_to_find={"className": "android.widget.Switch",
                                                      "text": "OFF"},
                                 view_to_check={"text": "ON"}, serial=serial)()



    # Teardown

    # we have to

    ui_steps.click_button_common(view_to_find={"text": "Open network notification"},
                                 second_view_to_find={"className": "android.widget.Switch",
                                                      "text": "OFF"},