#!/usr/bin/env python

##############################################################################
#
# @filename:    filter_by.py
#
# @description: Check filter by option for gdrive
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
    view_to_find = {"description":"More options"},
    view_to_check = {"text":"Filter by"})()

ui_steps.click_button(serial = serial,
    view_to_find = {"text":"Filter by"},
    view_to_check = {"text":"PDF"})()

ui_steps.click_button(serial = serial,
    view_to_find = {"text":"PDF"},
    view_to_check = {"text":"Filter: PDF"})()

drive_steps.check_files_type(serial = serial, file_type = "PDF")()

# Close app
ui_steps.close_app_from_recent(serial = serial,
                               view_to_find={"text": "Drive"})()
