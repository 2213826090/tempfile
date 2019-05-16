#######################################################################
#
# @filename:    AOSP_settings_Input_Incorrect_Password_to_Unlock_Screen_for_Many_Times
# @description: checking for the display message after entering the wrong password more than 5
#                times
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
import time

# ############# Get parameters ############
globals().update(vars(get_args(sys.argv)))
args = {}
for entry in script_args:
    key, val = entry.split("=")
    args[key] = val


# Setup to remove existing new users
while not ui_steps.remove_user(user_name="New user",optional=True, serial=serial)():
    break

ui_steps.press_home(serial=serial)()
ui_steps.press_car(serial=serial)()
ui_steps.open_settings(serial=serial)()

ui_steps.click_button_common(view_to_find={"text":"Security & location"},view_to_check={"text":"Screen lock"},serial=serial)()
ui_steps.click_button_common(view_to_find={"text":"Screen lock"},serial=serial)()
ui_steps.click_button_common(view_to_find={"text":"Password"},serial=serial)()
ui_steps.edit_text(view_to_find={"resourceId":"com.android.settings:id/password_entry"}, value="qwer", is_password=True,serial=serial)()
ui_steps.click_button_if_exists(view_to_find={"text":"NEXT"},view_to_check={"text":"Re-enter your password"},serial=serial)()
ui_steps.edit_text(view_to_find={"resourceId":"com.android.settings:id/password_entry"},value="qwer",is_password=True ,serial=serial)()
ui_steps.click_button_if_exists(view_to_find={"text":"OK"},view_to_check={"text":"Notifications"},serial=serial)()
ui_steps.click_button_if_exists(view_to_find={"text":"DONE"},serial=serial)()





ui_steps.put_device_into_sleep_mode(serial=serial)()
ui_steps.wake_up_device(serial=serial)()



ui_steps.put_device_into_sleep_mode(serial=serial)()
ui_steps.wake_up_device(serial=serial)()

#trying to enter wrong password for 5 times to get the message

#1
ui_steps.edit_text(view_to_find={"resourceId":"com.android.systemui:id/passwordEntry"}, value="tyui", is_password=True,serial=serial)()
adb_steps.command(command="input keyevent KEYCODE_ENTER",serial=serial)()
ui_steps.wait_for_view(view_to_find={"text": "Wrong Password"},serial=serial)()

print "1 done "

#2
ui_steps.edit_text(view_to_find={"resourceId":"com.android.systemui:id/passwordEntry"}, value="tyui", is_password=True,serial=serial)()
adb_steps.command(command="input keyevent KEYCODE_ENTER",serial=serial)()
ui_steps.wait_for_view(view_to_find={"text": "Wrong Password"},serial=serial)()

print "2 done "


#3

ui_steps.edit_text(view_to_find={"resourceId":"com.android.systemui:id/passwordEntry"}, value="tyui", is_password=True,serial=serial)()
adb_steps.command(command="input keyevent KEYCODE_ENTER",serial=serial)()
ui_steps.wait_for_view(view_to_find={"text": "Wrong Password"},serial=serial)()

print "3 done "
#4

ui_steps.edit_text(view_to_find={"resourceId":"com.android.systemui:id/passwordEntry"}, value="tyui", is_password=True,serial=serial)()
adb_steps.command(command="input keyevent KEYCODE_ENTER",serial=serial)()
ui_steps.wait_for_view(view_to_find={"text": "Wrong Password"},serial=serial)()

print "4 done "

#5

ui_steps.edit_text(view_to_find={"resourceId":"com.android.systemui:id/passwordEntry"}, value="tyui", is_password=True,serial=serial)()
adb_steps.command(command="input keyevent KEYCODE_ENTER",serial=serial)()

ui_steps.wait_for_view(view_to_find={"textContains":"You have incorrectly typed your password 5 "
                                                    "times. \n\nTry again in 30 seconds"
},
                       serial=serial)()
print "5 done "



time.sleep(30)
ui_steps.click_button_if_exists(view_to_find={"text":"OK"},view_to_check={"resourceId":"com.android.systemui:id/passwordEntry"},serial=serial)()


ui_steps.edit_text(view_to_find={"resourceId":"com.android.systemui:id/passwordEntry"}, value="qwer", is_password=True,serial=serial)()

#  since there is no way to press enter button we have used Keydode enter
adb_steps.command(command="input keyevent KEYCODE_ENTER",serial=serial)()


#Teardown


#removing password
ui_steps.press_home(serial=serial)()
ui_steps.press_car(serial=serial)()
ui_steps.open_settings(serial=serial)()
ui_steps.click_button_common(view_to_find={"text":"Security & location"},view_to_check={"text":"Screen lock"},serial=serial)()
ui_steps.click_button_common(view_to_find={"text":"Screen lock"},view_to_check={"text":"Re-enter your password"},serial=serial)()
ui_steps.click_button_common(view_to_find={"text":"Re-enter your password"},serial=serial)()
ui_steps.edit_text(view_to_find={"resourceId":"com.android.settings:id/password_entry"},value="qwer",is_password=True ,serial=serial)()
adb_steps.command(command="input keyevent KEYCODE_ENTER",serial=serial)()
ui_steps.click_button_common(view_to_find={"text":"None"},view_to_check={"text":"Remove device protection?"},serial=serial)()
ui_steps.click_button_common(view_to_find={"text":"YES, REMOVE"},view_to_check={"text":"Screen lock"},serial=serial)()



