#!/usr/bin/env python

#######################################################################
#
# @filename:    Graphics_Correct_screen_resolution.py
# @description: Take screenshot and verify that the picture dimension
#		correspond to the screen resolution
# @author:      costin.carabas@intel.com
#
#######################################################################

import sys

from testlib.scripts.android.adb import adb_steps
from testlib.scripts.media import media_steps
from testlib.scripts.connections.local import local_steps

# Connect to device
from testlib.base.base_utils import get_args

globals().update(vars(get_args(sys.argv)))

SCREENSHOT_NAME = 'check_resolution.png'

########### Preconditions ###############
#########################################

adb_steps.connect_device(
    serial = serial,
    port = adb_server_port
)()

############### Test ####################
#########################################

media_steps.check_resolution(screenshot_name = SCREENSHOT_NAME)()

########### Postconditions ##############
#########################################

local_steps.command('rm ' + SCREENSHOT_NAME)

adb_steps.disconnect_device(
    serial = serial,
    local_port = adb_server_port
)()
