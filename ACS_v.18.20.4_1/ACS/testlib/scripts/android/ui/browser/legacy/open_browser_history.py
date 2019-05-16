#!/usr/bin/env python
from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.adb import adb_utils
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.android.ui.browser import browser_steps
from testlib.base import base_utils

import sys

globals().update(vars(base_utils.get_args(sys.argv)))
adb_steps.connect_device(
    serial = serial,
    port = adb_server_port
)()
globals().update({"version": adb_utils.get_android_version()})

ui_steps.press_home(serial = serial)()

browser_steps.open_browser(serial = serial)()
browser_steps.open_new_tab(serial = serial)()
browser_steps.open_browser_history(serial = serial)()

ui_steps.press_home(serial = serial)()

