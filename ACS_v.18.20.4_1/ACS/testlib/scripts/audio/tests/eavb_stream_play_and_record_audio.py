#!/usr/bin/env python

#######################################################################
#
# @filename:    eavb_stream_play_and_record_audio.py
# @description: stream, play and record audio from master to slave device
# @author:      saddam.hussain.abbas@intel.com
# @usage example:
#      python eavb_stream_play_and_record_audio.py -s DutSerialNumber
#               --script-args serial2=ReferenceSerialNumber
#               input_audio_file=InputAudioFile record_audio=<true|false>
#               eavb_stream=Stream
#
# Note: Always 'serial' will be master and 'serial2' will be slave
#######################################################################

# Build in libraries
import sys
import os
import time

# Used defined libraries
from testlib.base.base_utils import get_args
from testlib.scripts.audio import audio_steps
from testlib.scripts.audio import audio_utils

############## Get parameters ############
globals().update(vars(get_args(sys.argv)))
args = {}
for entry in script_args:
    key, val = entry.split("=")
    args[key] = val

# mandatory arguments
serial2 = args["serial2"]
if "input_audio_file" in args.keys():
    input_audio_file = args["input_audio_file"]
else:
    raise Exception("Missing argument: input_audio_file is mandatory")

if "record_audio" in args.keys():
    record_audio = args["record_audio"].lower()
else:
    raise Exception("Missing argument: record_audio is mandatory")

if "eavb_stream" in args.keys():
    eavb_stream = args["eavb_stream"]
else:
    raise Exception("Missing argument: eavb_stream is mandatory")

#optional parameters
if "slave_to_master" in args.keys():
    slave_to_master = args["slave_to_master"]
    if slave_to_master == "true":
        serial, serial2 = serial2, serial;

if "avb" in args.keys():
    avb = args["avb"]
    if avb == "true":
        audio_utils.avb_stream_handler(serial=serial)
        time.sleep(30)

if "gptp" in args.keys():
    gptp = args["gptp"]
    if gptp == "true":
        audio_utils.gptp(serial=serial)


# Setup

# Run
if record_audio == "false":
    audio_utils.set_eavb_audio_on_speaker(serial=serial2, state="on")
else:
    audio_utils.set_eavb_audio_on_speaker(serial=serial2, state="off")
    file_name = os.path.basename(input_audio_file)
    audio_steps.EavbStartRecordAudio(serial=serial2, eavb_stream=eavb_stream,
                                     output_audio_file="/sdcard/Download/recorded_{0}".format(file_name))()
audio_steps.EavbStreamAudio(serial=serial, input_audio_file=input_audio_file, eavb_stream=eavb_stream)()

if record_audio != "false":
    audio_steps.EavbStopRecordAudio(serial=serial2)()

# Teardown
# reverting back the changes
if record_audio == "true":
    audio_utils.set_eavb_audio_on_speaker(serial=serial2, state="on")
