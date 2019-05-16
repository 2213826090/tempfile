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

# Car UI_Mode_Verify_Car_Icon
ui_steps.press_car(serial=serial)()
ui_steps.wait_for_view_with_scroll(view_to_find={"text":"Camera"},serial=serial)()
ui_steps.wait_for_view_with_scroll(view_to_find={"text":"Contacts"},serial=serial)()
ui_steps.wait_for_view_with_scroll(view_to_find={"text":"Gallery"},serial=serial)()
ui_steps.wait_for_view_with_scroll(view_to_find={"text":"Kitchen Sink"},serial=serial)()
ui_steps.wait_for_view_with_scroll(view_to_find={"text":"Google Play Store"},serial=serial)()
ui_steps.wait_for_view_with_scroll(view_to_find={"text":"Settings"},serial=serial)()
ui_steps.wait_for_view_with_scroll(view_to_find={"text":"SystemUpdater"},serial=serial)()
ui_steps.wait_for_view_with_scroll(view_to_find={"text":"WebView Shell"},serial=serial)()

#Car UI_Mode_Verify_Music_Icon
ui_steps.press_media(serial=serial)()
ui_steps.wait_for_view_with_scroll(view_to_find={"text":"Bluetooth Audio"},serial=serial)()
ui_steps.wait_for_view_with_scroll(view_to_find={"text":"Google Play Music"},serial=serial)()
ui_steps.wait_for_view_with_scroll(view_to_find={"text":"Local Media Player"},serial=serial)()
ui_steps.wait_for_view_with_scroll(view_to_find={"text":"Radio"},serial=serial)()

#Car UI_Mode_Verify_Navigator
ui_steps.press_map(serial=serial)()
ui_steps.wait_for_view_with_scroll(view_to_find={"text":"Maps"},serial=serial)()

#Car UI_Mode_Verify_Overview_Icon
ui_steps.press_home(serial=serial)()
ui_steps.wait_for_view_with_scroll(view_to_find={"text":"Owner"},serial=serial)()

#Car UI_Mode_Verify_Phone_Icon
ui_steps.press_dialer(serial=serial)()
ui_steps.wait_for_view_with_scroll(view_to_find={"text":"Phone"},serial=serial)()
