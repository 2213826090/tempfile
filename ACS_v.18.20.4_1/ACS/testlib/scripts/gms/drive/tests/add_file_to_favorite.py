#!/usr/bin/env python

##############################################################################
#
# @filename:    add_file_to_favorite.py
#
# @description: Star a file
#
# @author:      alexandru.n.branciog@intel.com
#
##############################################################################

import sys
import time
import datetime
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.gms.drive import drive_steps
from testlib.scripts.wireless.wifi import wifi_steps
from testlib.scripts.gms.drive import drive_utils
from testlib.base.base_utils import get_args

args = get_args(sys.argv)
globals().update(vars(args))
globals().update(eval(script_args[0]))

file_name = file_name.replace("_", " ")

ui_steps.press_home(serial = serial)()

drive_steps.open_drive(serial = serial, account = account,
                        password = password, force = True)()

drive_utils.search_by(name = file_name, serial = serial)
ui_steps.click_button(serial = serial,
    view_to_find = {"descriptionContains":file_name},
    child_view_to_find = {"description":"Show item properties"},
    view_to_check = {"text":"Star"})()

ui_steps.click_button(serial = serial,
    view_to_find = {"text":"Star"},
    view_to_check = {"text":"Unstar"})()

ui_steps.click_button(serial = serial,
    view_to_find = {"description":"Close item details"},
    view_to_check = {"description":"Close search"})()

ui_steps.click_button(serial = serial,
    view_to_find = {"description":"Close search"},
    view_to_check = {"text":"My Drive"})()

ui_steps.click_button(serial = serial,
    view_to_find = {"description":"Open navigation drawer"},
    view_to_check = {"text":"Starred"})()

ui_steps.click_button(serial = serial,
    view_to_find = {"text":"Starred"},
    view_to_check = {"descriptionContains":file_name})()

ui_steps.check_object_count(serial = serial,
    view_to_find = {"className":"android.widget.FrameLayout",
        "resourceId":"com.google.android.apps.docs:id/doc_entry_root"},
    count = 1, comparator = "=")()

# unstar the file
ui_steps.click_button(serial = serial,
    view_to_find = {"descriptionContains":file_name},
    child_view_to_find = {"description":"Show item properties"},
    view_to_check = {"text":"Unstar"})()

ui_steps.click_button(serial = serial,
    view_to_find = {"text":"Unstar"},
    view_to_check = {"text":"Star"})()

ui_steps.click_button(serial = serial,
    view_to_find = {"description":"Close item details"},
    view_to_check = {"descriptionContains":file_name}, view_presence = False)()

ui_steps.click_button(serial = serial,
    view_to_find = {"description":"Open navigation drawer"},
    view_to_check = {"text":"My Drive"})()

ui_steps.click_button(serial = serial,
    view_to_find = {"text":"My Drive"},
    view_to_check = {"description":"Open navigation drawer"})()

# Close app
ui_steps.close_app_from_recent(serial = serial,
                               view_to_find={"text": "Drive"})()
