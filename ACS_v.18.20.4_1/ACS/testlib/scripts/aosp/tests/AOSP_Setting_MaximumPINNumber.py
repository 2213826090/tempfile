#######################################################################
#
# @filename:    AOSP_setting_maximumPINNumber.py
# @description: checking how many maximum characters number ,security password can take at a time
# @author:      dpanday@intel.com
#
######################################################################

#Build in libraries
import sys

# Used defined libraries
from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.ui import ui_steps
from testlib.base.base_utils import get_args
from uiautomator import device as d

# ############# Get parameters ############
globals().update(vars(get_args(sys.argv)))
args = {}
for entry in script_args:
    key, val = entry.split("=")
    args[key] = val


while not ui_steps.remove_user(user_name="New user",optional=True, serial=serial)():
    break

#setup

ui_steps.press_home(serial=serial)()
ui_steps.press_car(serial=serial)()
ui_steps.open_settings(serial=serial)()

ui_steps.click_button_common(view_to_find={"text":"Security & location"},view_to_check={"text":"Screen lock"},serial=serial)()
ui_steps.click_button_common(view_to_find={"text":"Screen lock"},serial=serial)()
ui_steps.click_button_common(view_to_find={"text":"PIN"},serial=serial)()

#try entering invalid password
ui_steps.edit_text(view_to_find={"resourceId":"com.android.settings:id/password_entry"},
                   value="12345678912345678", is_password=True,serial=serial)()

ui_steps.click_button_common(view_to_find={"text":"NEXT"},view_to_check={"text":"Must be fewer "
                                                                                "than "
                                                                             "17 "
                                                                          "digits"},serial=serial)()

#entering valid pin to set lock

ui_steps.edit_text(view_to_find={"resourceId":"com.android.settings:id/password_entry"},value="123456789123456",is_password=True ,serial=serial)()
ui_steps.click_button_if_exists(view_to_find={"text":"NEXT"},view_to_check={"text":"Re-enter your PIN"},serial=serial)()
ui_steps.edit_text(view_to_find={"resourceId":"com.android.settings:id/password_entry"},value="123456789123456",is_password=True ,serial=serial)()
ui_steps.click_button_if_exists(view_to_find={"text":"OK"},view_to_check={"text":"Notifications"},serial=serial)()
ui_steps.click_button_if_exists(view_to_find={"text":"DONE"},serial=serial)()


#remove the password
ui_steps.press_home(serial=serial)()
ui_steps.press_car(serial=serial)()
ui_steps.open_settings(serial=serial)()
ui_steps.click_button_common(view_to_find={"text":"Security & location"},view_to_check={"text":"Screen lock"},serial=serial)()
ui_steps.click_button_common(view_to_find={"text":"Screen lock"},view_to_check={"text":"Re-enter your PIN"},serial=serial)()
ui_steps.click_button_common(view_to_find={"text":"Re-enter your PIN"},serial=serial)()
ui_steps.edit_text(view_to_find={"resourceId":"com.android.settings:id/password_entry"},value="123456789123456",is_password=True ,serial=serial)()

adb_steps.command(command="input keyevent KEYCODE_ENTER",serial=serial)()
ui_steps.click_button_common(view_to_find={"text":"None"},view_to_check={"text":"Remove device protection?"},serial=serial)()
ui_steps.click_button_common(view_to_find={"text":"YES, REMOVE"},view_to_check={"text":"Screen lock"},serial=serial)()


