#######################################################################
#
# @filename:    AOSP_Android_N_Car_Application-Menu.Dialer_A_number_hide.py
# @description: entering the number and hiding the dialer pad coming to home screen
# @author:      dpanday@intel.com
#
#######################################################################



#Build in libraries
import sys


# Used defined libraries
from testlib.base.base_utils import get_args
import time
from testlib.scripts.wireless.bluetooth.bt_step import Step as BtStep
from testlib.scripts.android.ui import ui_steps

globals().update(vars(get_args(sys.argv)))
args = {}
for entry in script_args:
    key, val = entry.split("=")
    args[key] = val

ui_steps.press_home(serial=serial)()
ui_steps.press_car(serial=serial)()

ui_steps.press_dialer(serial=serial)()

ui_steps.click_button_common( view_to_find={'text':"Phone"}, second_view_to_find={"className":"android.widget.ImageButton"},serial=serial)()

time.sleep(5)
#Dialing the numbers
ui_steps.click_button_common(view_to_find={"text":"Dial a number"},view_to_check={"text":"Dial a number"},serial=serial)()
ui_steps.click_button_common(view_to_find={"text":"1"},view_to_check={"text":"1"},serial=serial)()
ui_steps.click_button_common(view_to_find={"text":"2"},view_to_check={"text":"2"},serial=serial)()
ui_steps.click_button_common(view_to_find={"text":"3"},view_to_check= {"text":"123"},serial=serial)()

#press hide button
ui_steps.click_button(view_to_find={"resourceId":"com.android.car.dialer:id/exit_dialer_button"},view_to_check={"text":"Phone"},serial=serial)()


