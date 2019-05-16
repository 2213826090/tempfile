#######################################################################
#
# @filename:    GMS_multisource_mangaement_time_format.py
# @description: enable 24 hours format for user and disable for new user
# @author:      dpanday@intel.com
#
######################################################################

#Build in libraries
import sys

# Used defined libraries
from testlib.scripts.android.ui import ui_steps
from testlib.base.base_utils import get_args


# ############# Get parameters ############

globals().update(vars(get_args(sys.argv)))
args = {}
for entry in script_args:
    key, val = entry.split("=")
    args[key] = val


while not ui_steps.remove_user(user_name="New user",optional=True, serial=serial)():
    break
#Setup
ui_steps.open_settings(serial=serial)()
ui_steps.click_button_with_scroll(view_to_find={"text":"Users & accounts"},view_to_check={"text":"Users"},serial=serial)()
ui_steps.click_button_common(view_to_find={"text":"Users"},view_to_check={"text":"Add user or profile"},serial=serial)()
ui_steps.click_button_common(view_to_find={"text":"Admin"},serial=serial)()
ui_steps.press_home(serial=serial)()
ui_steps.open_settings(serial=serial)()
ui_steps.click_button_common(view_to_find={"text":"System"},view_to_check={"text":"Date & time"},serial=serial)()
ui_steps.click_button_common(view_to_find={"text":"Date & time"},view_to_check={"text":"Automatic date & time"},serial=serial)()

if not ui_steps.click_button_common(view_to_find = {"text":"Use 24-hour format"}, second_view_to_find={"className": "android.widget.Switch", "text": "OFF"},view_to_check = {"text": "ON"},optional= True, serial=serial)():
    ui_steps.click_button_common(view_to_find={"text": "Use 24-hour format"},
                                        second_view_to_find={"className": "android.widget.Switch", "text": "ON"},
                                        view_to_check={"text": "OFF"}, serial=serial)()


#change to owner will enable 24hors format

ui_steps.open_settings(serial=serial)()
ui_steps.click_button_with_scroll(view_to_find={"text":"Users & accounts"},view_to_check={"text":"Users"},serial=serial)()
ui_steps.click_button_common(view_to_find={"text":"Users"},view_to_check={"text":"Add user or profile"},serial=serial)()
ui_steps.click_button_common(view_to_find={"text":"Admin"},serial=serial)()
ui_steps.press_home(serial=serial)()
ui_steps.open_settings(serial=serial)()
ui_steps.click_button_common(view_to_find={"text":"System"},view_to_check={"text":"Date & time"},serial=serial)()
ui_steps.click_button_common(view_to_find={"text":"Date & time"},view_to_check={"text":"Automatic date & time"},serial=serial)()
ui_steps.click_button_common(view_to_find = {"text":"Use 24-hour format"},second_view_to_find={"className": "android.widget.Switch", "text": "OFF"},view_to_check = {"text": "ON"}, serial=serial)()


#Change to new user
ui_steps.press_home(serial=serial)()
ui_steps.open_settings(serial=serial)()
ui_steps.click_button_with_scroll(view_to_find={"text":"Users & accounts"},view_to_check={"text":"Users"},serial=serial)()
ui_steps.click_button_common(view_to_find={"text":"Users"},view_to_check={"text":"Add user or profile"},serial=serial)()
ui_steps.click_button(view_to_find={"text":"Add user or profile"},view_to_check={"text":"Users have their own apps and content"},serial=serial)()
ui_steps.click_button(view_to_find={"text":"Users have their own apps and content"},view_to_check={"text":"Add new user?"},serial=serial)()
ui_steps.click_button_if_exists(view_to_find={"text":"OK"},serial=serial)()
ui_steps.click_button(view_to_find={"text":"SET UP NOW"},serial=serial)()

#Setup for new user
ui_steps.press_home(serial=serial)()
ui_steps.open_settings(serial=serial)()
ui_steps.click_button_common(view_to_find={"text":"System"},view_to_check={"text":"Date & time"},serial=serial)()
ui_steps.click_button_common(view_to_find={"text":"Date & time"},view_to_check={"text":"Automatic date & time"},serial=serial)()

if not ui_steps.click_button_common(view_to_find = {"text":"Use 24-hour format"}, second_view_to_find={"className": "android.widget.Switch", "text": "OFF"},view_to_check = {"text": "ON"},optional= True, serial=serial)():
    ui_steps.click_button_common(view_to_find={"text": "Use 24-hour format"},
                                        second_view_to_find={"className": "android.widget.Switch", "text": "OFF"},
                                        view_to_check={"text": "ON"}, serial=serial)()

# disbaling  24 hors format for new user
ui_steps.open_settings(serial=serial)()
ui_steps.press_home(serial=serial)()
ui_steps.open_settings(serial=serial)()
ui_steps.click_button_common(view_to_find={"text":"System"},view_to_check={"text":"Date & time"},serial=serial)()
ui_steps.click_button_common(view_to_find={"text":"Date & time"},view_to_check={"text":"Use 24-hour format"},serial=serial)()
ui_steps.click_button_common(view_to_find = {"text":"Use 24-hour format"},second_view_to_find={"className": "android.widget.Switch", "text": "ON"},view_to_check = {"text": "OFF"}, serial=serial)()


#Teardown going back to owener and removing new user


while not ui_steps.remove_user(user_name="New user",optional=True, serial=serial)():
    break