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
###############################

ui_steps.press_home(serial=serial)()

#RUN
ui_steps.open_settings(serial=serial)()
ui_steps.wait_for_view_with_scroll(view_to_find={"text":"Network & Internet"},serial=serial)()
ui_steps.wait_for_view_with_scroll(view_to_find={"text":"Connected devices"},serial=serial)()
ui_steps.wait_for_view_with_scroll(view_to_find={"text":"Apps & notifications"},serial=serial)()
ui_steps.wait_for_view_with_scroll(view_to_find={"text":"Display"},serial=serial)()
ui_steps.wait_for_view_with_scroll(view_to_find={"text":"USB Role"},serial=serial)()
ui_steps.wait_for_view_with_scroll(view_to_find={"text":"Sound"},serial=serial)()
ui_steps.wait_for_view_with_scroll(view_to_find={"text":"Storage"},serial=serial)()
ui_steps.wait_for_view_with_scroll(view_to_find={"text":"Security & location"},serial=serial)()
ui_steps.wait_for_view_with_scroll(view_to_find={"text":"Users & accounts"},serial=serial)()
ui_steps.wait_for_view_with_scroll(view_to_find={"text":"Accessibility"},serial=serial)()
ui_steps.wait_for_view_with_scroll(view_to_find={"text":"Google"},serial=serial)()
ui_steps.wait_for_view_with_scroll(view_to_find={"text":"System"},serial=serial)()