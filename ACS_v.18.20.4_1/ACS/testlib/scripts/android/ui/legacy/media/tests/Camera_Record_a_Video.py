#!/usr/bin/env python

#######################################################################
#
# @filename:    Camera_Record_a_Video.py
# @description: Record video using default settings
# @author:      costin.carabas@intel.com
#
#######################################################################

import sys

from testlib.scripts.android.adb import adb_steps
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

if script_args and "gmin" in script_args:
    media_steps.record_video(platform = "gmin")()
else:
    media_steps.record_video(platform = "starpeak")()


########### Postconditions ##############
#########################################
if not (script_args and "gmin" in script_args):
    adb_steps.disconnect_device(
        serial = serial,
        local_port = adb_server_port
    )()
