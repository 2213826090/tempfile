#!/usr/bin/env python
# usage:
#   python open_image_in_a_new_tab.py --serial 4E4FAF62
#     --script-args url = "http://commons.wikimedia.org/wiki/\
#                          File:Starfish,_Mauritius.jpg"
#                   url-title="Mauritius.jpg(JPEG Image"
#                   history-entry="1200px-Starfish,_Mauritius.jpg \
#                                  (1200x900)"

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
    key, val = entry.split("=")
    args[key] = val

url = args["url"]
url_title = args["url-title"]
history_entry = args["history-entry"]

ui_steps.press_home()()

browser_steps.open_browser()()
browser_steps.close_all_tabs()()
browser_steps.open_new_tab()()
browser_steps.clear_browser_history()()
browser_steps.close_tab()()
browser_steps.open_new_tab()()
browser_steps.open_specific_page(url = url,
                                 url_title = url_title)()
browser_steps.open_image_in_new_tab(text_to_check = history_entry)()

ui_steps.press_home()()

