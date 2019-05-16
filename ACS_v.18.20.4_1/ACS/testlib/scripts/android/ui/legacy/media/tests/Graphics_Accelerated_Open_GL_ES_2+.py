#!/usr/bin/env python

#######################################################################
#
# @filename:    Graphics_Accelerated_Open_GL_ES_2+.py
# @description: Open About Tablet and verify GL Version: OpenGL ES 3.0
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

OPENGL_VERSION = 'OpenGL ES 3.0'

########### Preconditions ###############
#########################################

adb_steps.connect_device(
    serial = serial,
    port = adb_server_port
)()

############### Test ####################
#########################################

ui_steps.press_home()()
ui_steps.open_settings()()
ui_steps.open_app_from_settings(
                        view_to_find = {'text': 'About tablet'},
                        view_to_check = {'textContains': OPENGL_VERSION}
                               )()

########### Postconditions ##############
#########################################

adb_steps.disconnect_device(
    serial = serial,
    local_port = adb_server_port
)()
