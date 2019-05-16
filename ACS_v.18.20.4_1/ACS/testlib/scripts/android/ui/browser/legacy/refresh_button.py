#!/usr/bin/env python


import sys

from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.adb import adb_utils
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.android.ui.browser import browser_steps
from testlib.base.base_utils import get_args

globals().update(vars(get_args(sys.argv)))
adb_steps.connect_device(
    serial = serial,
    port = adb_server_port
)()

globals().update({"version": adb_utils.get_android_version()})

args = {}
for entry in script_args:
    splits = entry.split("=")
    args[splits[0]] = "=".join(splits[1:])

ui_steps.press_home()()

browser_steps.open_browser()()
browser_steps.close_all_tabs()()
browser_steps.open_new_tab()()
browser_steps.clear_browser_history()()
browser_steps.close_tab()()
browser_steps.open_new_tab()()
browser_steps.open_specific_page(url = "google.com",
                                 url_title = "Gogole")()
browser_steps.refresh_button()()

ui_steps.press_home()()

