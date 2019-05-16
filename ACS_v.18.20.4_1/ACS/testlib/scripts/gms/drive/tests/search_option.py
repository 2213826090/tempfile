#!/usr/bin/env python

##############################################################################
#
# @filename:    search_option.py
#
# @description: Check search option for gdrive
#
# @author:      alexandru.n.branciog@intel.com
#
##############################################################################

import sys
import time
import datetime
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.gms.drive import drive_steps

from testlib.base.base_utils import get_args

args = get_args(sys.argv)
globals().update(vars(args))
globals().update(eval(script_args[0]))

ui_steps.press_home(serial = serial)()

drive_steps.open_drive(serial = serial, account = account,
                        password = password, force = True)()

ui_steps.click_button(serial = serial,
    view_to_find = {"description":"Search"},
    view_to_check = {"textContains":"Search"})()

# Input search string
ui_steps.edit_text(serial = serial, view_to_find = {"textContains":"Search"},
                        value = search_for)();

ui_steps.wait_for_view(serial = serial,
    view_to_find = {"textContains":"Earlier"})();

# Check
ui_steps.check_object_count(serial = serial,
    view_to_find = {"descriptionContains":search_for},
    count = 2, comparator = "=")()

ui_steps.click_button(serial = serial,
    view_to_find = {"description":"Close search"},
    view_to_check = {"text":"My Drive"})()

# Close app
ui_steps.close_app_from_recent(serial = serial,
                               view_to_find={"text": "Drive"})()
