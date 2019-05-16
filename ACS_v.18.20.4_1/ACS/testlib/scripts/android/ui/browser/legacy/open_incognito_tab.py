#!/usr/bin/env python


import sys

from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.adb import adb_utils
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.android.ui.browser import browser_steps
from testlib.base import base_utils

globals().update(vars( import base_utils.get_args(sys.argv)))
adb_steps.connect_device(
    serial = serial,
    port = adb_server_port
)()
globals().update({"version": adb_utils.get_android_version()})

args = {}
for entry in script_args:
    key, val = entry.split("=")
    args[key] = val
url = args["url"]
url_title = args["url-title"]
history_entry = args["history-entry"]

ui_steps.press_home(serial = serial)()

browser_steps.open_browser(serial = serial)()
browser_steps.close_all_tabs(serial = serial)()
browser_steps.open_new_tab(serial = serial)()
browser_steps.clear_browser_history(serial = serial)()
browser_steps.close_tab(serial = serial)()
browser_steps.open_incognito_tab(serial = serial)()
browser_steps.open_specific_page(serial = serial
                                 url = url,
                                 url_title = url_title)()
browser_steps.check_page_in_history(serial = serial
                                    page_to_check = history_entry,
                                    exists = False)()

ui_steps.press_home(serial = serial)()

