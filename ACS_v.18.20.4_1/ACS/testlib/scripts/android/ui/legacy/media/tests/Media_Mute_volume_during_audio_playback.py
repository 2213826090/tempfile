#!/usr/bin/env python

#######################################################################
#
# @filename:    Media_Mute_volume_during_audio_playback.py
# @description: Modify volume and verify the database that the
#               modification has been made
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
adb_steps.root_connect_device()()

############### Test ####################
#########################################

ui_steps.press_home()()
ui_steps.open_settings()()
ui_steps.open_app_from_settings(view_to_find = {'textContains': 'Sound'},
                                view_to_check = {'textContains': 'Volume'})()

if not (script_args and "gmin" in script_args):
    ui_steps.click_button(view_to_find = {'textContains': 'Volumes'})()

if script_args and "gmin" in script_args:
    media_steps.mute_sound('volume_alarm_speaker', 1)()
    media_steps.mute_sound('volume_ring_speaker', 2)()
    media_steps.mute_sound('volume_music_speaker', 0)()
else:
    media_steps.mute_sound('volume_ring_speaker', 1)()
    media_steps.mute_sound('volume_alarm_speaker', 2)()
    media_steps.mute_sound('volume_music_speaker', 0)()


if not (script_args and "gmin" in script_args):
    ui_steps.click_button(view_to_find = {'textContains': 'Ok'})()

########### Postconditions ##############
#########################################
if not (script_args and "gmin" in script_args):
    adb_steps.disconnect_device(
        serial = serial,
        local_port = adb_server_port
    )()
