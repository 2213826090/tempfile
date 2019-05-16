#!/usr/bin/env python

#######################################################################
#
# @filename:    eavb_device_setup.py
# @description: prepare device for eavb master or slave
# @author:      saddam.hussain.abbas@intel.com
# @usage example:
#      python eavb_device_setup.py -s DutSerialNumber
#               --script-args mode=<m|s>
#
# Note: Always 'serial' will be master and 'serial2' will be slave
#######################################################################

# Build in libraries
import sys
import os

# Used defined libraries
from testlib.base.base_utils import get_args
from testlib.scripts.audio import audio_steps

############## Get parameters ############
globals().update(vars(get_args(sys.argv)))
args = {}
for entry in script_args:
    key, val = entry.split("=")
    args[key] = val

# Mandatory arguments
serial2 = args["serial2"]
#if "mode" in args.keys():
#    mode = args['mode']
#else:
#    raise Exception("Missing Argument: Mode argument is mandatory")

audio_steps.EavbSetMode(serial=serial2, mode="s")()
audio_steps.EavbSetMode(serial=serial, mode="m")()
audio_steps.EavbSetup(serial=serial2, mode="s")()
audio_steps.EavbSetup(serial=serial, mode="m")()
