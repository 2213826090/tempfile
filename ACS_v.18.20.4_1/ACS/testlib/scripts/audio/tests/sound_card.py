#!/usr/bin/env python

#######################################################################
#
# @filename:    eavb_stream_play_and_record_audio.py
# @description: stream, play and record audio from master to slave device
# @author:      uma.rajendran@intel.com
# @usage example:
#      python eavb_device_setup.py -s DutSerialNumber
#
# Note: Always 'serial' will be master and 'serial2' will be slave
#######################################################################

# Build in libraries
import sys
import os

# Used defined libraries
from testlib.base.base_utils import get_args
from testlib.scripts.audio import audio_utils

globals().update(vars(get_args(sys.argv)))
args = {}
#for entry in script_args:
#    key, val = entry.split("=")
#    args[key] = val

# mandatory arguments
#serial2 = args["serial2"]
print serial

audio_utils.sound_card(serial=serial)
#audio_utils.stream(serial=serial2)


