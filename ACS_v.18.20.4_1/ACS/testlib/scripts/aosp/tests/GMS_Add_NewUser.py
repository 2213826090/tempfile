#######################################################################
#
# @filename:    GMS_Add_NewUser.py
# @description: creating new users
# @author:      dpanday@intel.com
#
######################################################################

# Build in librariesimport sys
import sys

# Used defined libraries
from testlib.base.base_utils import get_args

from testlib.scripts.android.ui import ui_steps

# ############# Get parameters ############
globals().update(vars(get_args(sys.argv)))
args = {}
#b = {}
if script_args[0].upper() != 'NONE':
    for entry in script_args:
        key, val = entry.split("=")
        args[key] = val


# Setup to remove existing new users

while not ui_steps.remove_user(user_name="New user",optional=True, serial=serial)():
    break


ui_steps.press_home(serial=serial)()
ui_steps.press_car(serial=serial)()
ui_steps.open_settings(serial=serial)()
ui_steps.click_button_with_scroll(view_to_find={"text":"Users & accounts"},view_to_check={"text":"Users"},serial=serial)()
ui_steps.click_button_common(view_to_find={"text":"Users"},view_to_check={"text":"Add user or profile"},serial=serial)()
ui_steps.click_button(view_to_find={"textContains": "Add user"}, view_to_check={
    "text": "OK"}, serial=serial)()
ui_steps.click_button(view_to_find={"text":"User"},view_to_check={"text":"Add new user?"},serial=serial)()
ui_steps.click_button_if_exists(view_to_find={"text":"OK"},view_to_check={"text":"NOT NOW"},serial=serial)()
ui_steps.click_button(view_to_find={"text":"SET UP NOW"},serial=serial)()


#teardown
while not ui_steps.remove_user(user_name="New user",optional=True, serial=serial)():
    break


  ui_steps.click_button_with_scroll(view_to_find={"text":"Users & accounts"},view_to_check={"text":"Users"},serial=serial)()
  ui_steps.click_button_common(view_to_find={"text":"Users"},view_to_check={"text":"Add user"},
                              serial=serial)()
  ui_steps.click_button(view_to_find={"textContains":"Add user"},view_to_check={
        "text":"OK"},serial=serial)()