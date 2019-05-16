#######################################################################
#
# @filename:    AAOSP_Android_N_Car_Application-Menu.Dialer_A_number_input.py
# @description: entering the number using dialer pad
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
####################################
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
#input the numbers through dialpad

ui_steps.click_button_common(view_to_find={"text":"Dial a number"},view_to_check={"text":"Dial a number"},serial=serial)()
ui_steps.click_button_common(view_to_find={"text":"1"},view_to_check={"text":"1"},serial=serial)()
ui_steps.click_button_common(view_to_find={"text":"2"},view_to_check={"text":"2"},serial=serial)()
ui_steps.click_button_common(view_to_find={"text":"3"},view_to_check={"text":"3"},serial=serial)()
ui_steps.click_button_common(view_to_find={"text":"4"},view_to_check={"text":"4"},serial=serial)()
ui_steps.click_button_common(view_to_find={"text":"5"},view_to_check={"text":"5"},serial=serial)()
ui_steps.click_button_common(view_to_find={"text":"6"},view_to_check={"text":"6"},serial=serial)()

ui_steps.click_button_common(view_to_find={"text":"7"},view_to_check= {"text":"123-4567"},serial=serial)()
#d(resourceId="com.android.car.dialer:id/number").clear_text()
ui_steps.click_button_common(view_to_find={"resourceId":"com.android.car.dialer:id/exit_dialer_button"},view_to_check={"text":"Phone"},serial=serial)()



