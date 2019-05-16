#!/usr/bin/env python

#######################################################################
#
# @filename:    Camera_Preview_displayed_onscreen.py
# @description: Open camera app
# @author:      costin.carabas@intel.com
#
#######################################################################

import sys

from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.media import media_steps

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

ui_steps.press_home()()
ui_steps.open_app_from_allapps(view_to_find = {'text': 'Camera'},
                               view_to_check = {'descriptionContains': 'Shutter'})()
########### Postconditions ##############
#########################################
if not (script_args and "gmin" in script_args):
    adb_steps.disconnect_device(
        serial = serial,
        local_port = adb_server_port
    )()
