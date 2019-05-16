#!/usr/bin/env python

#######################################################################
#
# @filename:    Media_Open_Picture.py
# @description: Open picture taken
# @author:      costin.carabas@intel.com
#
#######################################################################

import sys
import time

from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.media import media_steps
from testlib.utils.ui.uiandroid import UIDevice as ui_device

# Connect to device
from testlib.base.base_utils import get_args

globals().update(vars(get_args(sys.argv)))


########### Preconditions ###############
#########################################

adb_steps.connect_device(
    serial = serial,
    port = adb_server_port
)()

media_steps.clear_old_media()()

############### Test ####################
#########################################

if script_args and "gmin" in script_args:
    media_steps.take_picture(platform = "gmin")()
else:
    media_steps.take_picture(platform = "starpeak")()

time.sleep(3)

#swipe_left (to enter the gallery)
uidevice().swipe(400,400, 100,400, steps = 10)

time.sleep(1)
#open details and check for title
ui_steps.click_button(view_to_find  = {"descriptionContains": "More Options"},
                      view_to_check = {"text": "Details"})()
ui_steps.click_button(view_to_find = {"text": "Details"},
                      view_to_check = {"textContains": "Title:"})()
ui_steps.click_button(view_to_find = {"text": "Close"})()

########### Postconditions ##############
#########################################
if not (script_args and "gmin" in script_args):
    adb_steps.disconnect_device(
        serial = serial,
        local_port = adb_server_port
    )()
