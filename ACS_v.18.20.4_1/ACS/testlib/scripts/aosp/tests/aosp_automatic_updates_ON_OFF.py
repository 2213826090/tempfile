#!/usr/bin/env python

#######################################################################
#
# @filename:    aosp_automatic_updates_ON_OFF.py
# @description: Turn ON and OFF automatic updates
# @author:      saddam.hussain.abbas@intel.com
# @usage example:
#      python aosp_automatic_updates_ON_OFF.py -s DutSerialNumber
#               --script-args None
#######################################################################

# Build in libraries
import sys

# Used defined libraries
from testlib.base.base_utils import get_args
from testlib.scripts.android.ui import ui_steps

# ############# Get parameters ############
globals().update(vars(get_args(sys.argv)))
args = {}
if script_args[0].upper() != 'NONE':
    for entry in script_args:
        key, val = entry.split("=")
        args[key] = val

# Setup
ui_steps.press_home(serial=serial)()

# Run
ui_steps.enable_options_from_developer_options(serial=serial,
                 developer_options=["Automatic system updates"],
                                               )()
ui_steps.disable_options_from_developer_options(serial=serial,
                 developer_options=["Automatic system updates"],
                                                enabled=True)()


