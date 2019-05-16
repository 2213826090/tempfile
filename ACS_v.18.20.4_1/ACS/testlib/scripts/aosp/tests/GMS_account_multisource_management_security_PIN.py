#######################################################################
#
# @filename:    GMS_account_multisource_management_security_PIN.py
# @description: setting PIN for owner and new user
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


# Setup to remove existing new users

while not ui_steps.remove_user(user_name="New user",optional=True, serial=serial)():
    break

#setup
ui_steps.press_home(serial=serial)()
ui_steps.press_car(serial=serial)()
ui_steps.open_settings(serial=serial)()
#ui_steps.switch_user(user_name = "Owner",serial=serial)()
ui_steps.click_button_common(view_to_find={"text":"Security & location"},view_to_check={"text":"Screen lock"},serial=serial)()
ui_steps.click_button_common(view_to_find={"text":"Screen lock"},serial=serial)()
ui_steps.click_button_common(view_to_find={"text":"PIN"},serial=serial)()
ui_steps.edit_text(view_to_find={"resourceId":"com.android.settings:id/password_entry"}, value="1234", is_password=True,serial=serial)()
ui_steps.click_button_if_exists(view_to_find={"text":"NEXT"},view_to_check={"text":"Re-enter your PIN"},serial=serial)()
ui_steps.edit_text(view_to_find={"resourceId":"com.android.settings:id/password_entry"},value="1234",is_password=True ,serial=serial)()
ui_steps.click_button_if_exists(view_to_find={"text":"OK"},view_to_check={"text":"Notifications"},serial=serial)()
ui_steps.click_button_if_exists(view_to_find={"text":"DONE"},serial=serial)()

# create user and switching to new user ... setting the password
ui_steps.press_home(serial=serial)()
ui_steps.press_car(serial=serial)()
ui_steps.open_settings(serial=serial)()
ui_steps.click_button_common(view_to_find={"text":"Users & accounts"},view_to_check={"text":"Users"},serial=serial)()
ui_steps.click_button_common(view_to_find={"text":"Users"},view_to_check={"text":"Add user or profile"},serial=serial)()
ui_steps.click_button_common(view_to_find={"text":"Add user or profile"},view_to_check={"text":"Users have their own apps and content"},serial=serial)()
ui_steps.click_button_common(view_to_find={"text":"Users have their own apps and content"},view_to_check={"text":"Add new user?"},serial=serial)()
ui_steps.click_button_if_exists(view_to_find={"text":"OK"},view_to_check={"text":"NOT NOW"},serial=serial)()
ui_steps.click_button_common(view_to_find={"text":"SET UP NOW"},serial=serial)()


ui_steps.press_home(serial=serial)()
ui_steps.press_car(serial=serial)()
ui_steps.open_settings(serial=serial)()
ui_steps.click_button_common(view_to_find={"text":"Security & location"},view_to_check={"text":"Screen lock"},serial=serial)()
ui_steps.click_button_common(view_to_find={"text":"Screen lock"},serial=serial)()
ui_steps.click_button_common(view_to_find={"text":"PIN"},serial=serial)()
ui_steps.edit_text(view_to_find={"resourceId":"com.android.settings:id/password_entry"}, value="4321", is_password=True,serial=serial)()
ui_steps.click_button_if_exists(view_to_find={"text":"NEXT"},view_to_check={"text":"Re-enter your PIN"},serial=serial)()
ui_steps.edit_text(view_to_find={"resourceId":"com.android.settings:id/password_entry"},value="4321",is_password=True ,serial=serial)()
ui_steps.click_button_if_exists(view_to_find={"text":"OK"},view_to_check={"text":"Notifications"},serial=serial)()
ui_steps.click_button_if_exists(view_to_find={"text":"DONE"},serial=serial)()

ui_steps.put_device_into_sleep_mode(serial=serial)()
#time.sleep(5)
d.wakeup()

ui_steps.edit_text(view_to_find={"resourceId":"com.android.systemui:id/pinEntry"}, value="4321", is_password=True,serial=serial)()

ui_steps.click_button(view_to_find={"resourceId":"com.android.systemui:id/key_enter"},view_to_check={"text":"Screen lock"},serial=serial)()

#Teardown

ui_steps.press_home(serial=serial)()
ui_steps.press_car(serial=serial)()
d(index="5",className="android.widget.FrameLayout").child(text="Settings").click()
ui_steps.click_button_common(view_to_find={"text":"Users"},view_to_check={"text":"Owner"},serial=serial)()
ui_steps.click_button_common(view_to_find={"text":"Owner"},serial=serial)()
ui_steps.click_button_common(view_to_find={"resourceId":"com.android.car.settings:id/action_button2"},serial=serial)()
ui_steps.edit_text(view_to_find={"resourceId":"com.android.systemui:id/pinEntry"}, value="1234", is_password=True,serial=serial)()
ui_steps.click_button(view_to_find={"resourceId":"com.android.systemui:id/key_enter"},serial=serial)()


#Teardown
ui_steps.press_home(serial=serial)()
ui_steps.press_car(serial=serial)()
ui_steps.open_settings(serial=serial)()
ui_steps.click_button_common(view_to_find={"text":"Security & location"},view_to_check={"text":"Screen lock"},serial=serial)()
ui_steps.click_button_common(view_to_find={"text":"Screen lock"},view_to_check={"text":"Re-enter your PIN"},serial=serial)()
ui_steps.click_button_common(view_to_find={"text":"Re-enter your PIN"},serial=serial)()
ui_steps.edit_text(view_to_find={"resourceId":"com.android.settings:id/password_entry"},value="1234",is_password=True ,serial=serial)()
#adb_steps.adb shell input keyevent KEYCODE_ENTER
adb_steps.command(command="input keyevent KEYCODE_ENTER",serial=serial)()
ui_steps.click_button_common(view_to_find={"text":"None"},view_to_check={"text":"Remove device protection?"},serial=serial)()
ui_steps.click_button_common(view_to_find={"text":"YES, REMOVE"},view_to_check={"text":"Screen lock"},serial=serial)()

#Teardown

while not ui_steps.remove_user(user_name="New user",optional=True, serial=serial)():
    break
ui_steps.press_home(serial=serial)()
