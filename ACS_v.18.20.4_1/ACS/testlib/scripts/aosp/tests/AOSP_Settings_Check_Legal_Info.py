#Build in libraries
import sys
import time

# Used defined libraries
from testlib.base.base_utils import get_args
from testlib.base import base_step
from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.ui import ui_steps

# ############# Get parameters ############

globals().update(vars(get_args(sys.argv)))
args = {}
for entry in script_args:
    key, val = entry.split("=")
    args[key] = val

#Check for Third-party licenses
ui_steps.press_home(serial=serial)()
ui_steps.open_settings(serial=serial)()
ui_steps.click_button_common(view_to_find={"text":"System"},view_to_check={"text":"About phone"},serial=serial)()
ui_steps.click_button_common(view_to_find={"text":"About phone"},view_to_check={"text":"Legal information"},serial=serial)()
ui_steps.click_button_common(view_to_find={"text":"Legal information"},view_to_check={"text":"Third-party licenses"},serial=serial)()
ui_steps.click_button_common(view_to_find={"text":"Third-party licenses"},serial=serial)()

#Check for system WebView licenses
ui_steps.press_home(serial=serial)()
ui_steps.open_settings(serial=serial)()
ui_steps.click_button_common(view_to_find={"text":"System"},view_to_check={"text":"About phone"},serial=serial)()
ui_steps.click_button_common(view_to_find={"text":"About phone"},view_to_check={"text":"Legal information"},serial=serial)()
ui_steps.click_button_common(view_to_find={"text":"Legal information"},view_to_check={"text":"Third-party licenses"},serial=serial)()
ui_steps.click_button_common(view_to_find={"text":"System WebView licenses"},serial=serial)()