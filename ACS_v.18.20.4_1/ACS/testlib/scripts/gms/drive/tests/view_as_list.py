#!/usr/bin/env python

##############################################################################
#
# @filename:    view_as_list.py
#
# @description: Check view as list option
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


if ui_steps.click_button_if_exists(serial = serial,
    view_to_find = {"description":"View as List"})():
    # is list
    ui_steps.check_object_count(serial = serial,
        view_to_find = {"resourceId":"com.google.android.apps.docs:id/doc_entry_root",
        "className":"android.widget.RelativeLayout"},
        count = 3, comparator = ">")()
    # switch back to Grid view
    ui_steps.click_button(serial = serial,
        view_to_find = {"description":"View as Grid"},
        view_to_check = {"description":"View as List"})()
    # check
    ui_steps.check_object_count(serial = serial,
        view_to_find = {"resourceId":"com.google.android.apps.docs:id/doc_entry_root",
        "className":"android.widget.FrameLayout"},
        count = 3, comparator = ">")()
else:
    # switch to Grid view
    ui_steps.click_button(serial = serial,
        view_to_find = {"description":"View as Grid"},
        view_to_check = {"description":"View as List"})()
    # check
    ui_steps.check_object_count(serial = serial,
        view_to_find = {"resourceId":"com.google.android.apps.docs:id/doc_entry_root",
        "className":"android.widget.FrameLayout"},
        count = 3, comparator = ">")()

    # switch to List view
    ui_steps.click_button(serial = serial,
        view_to_find = {"description":"View as List"},
        view_to_check = {"description":"View as Grid"})()
    # check
    ui_steps.check_object_count(serial = serial,
        view_to_find = {"resourceId":"com.google.android.apps.docs:id/doc_entry_root",
        "className":"android.widget.RelativeLayout"},
        count = 3, comparator = ">")()

    # switch back to Grid view
    ui_steps.click_button(serial = serial,
        view_to_find = {"description":"View as Grid"},
        view_to_check = {"description":"View as List"})()
    # check
    ui_steps.check_object_count(serial = serial,
        view_to_find = {"resourceId":"com.google.android.apps.docs:id/doc_entry_root",
        "className":"android.widget.FrameLayout"},
        count = 3, comparator = ">")()

# Close app
ui_steps.close_app_from_recent(serial = serial,
                               view_to_find={"text": "Drive"})()
