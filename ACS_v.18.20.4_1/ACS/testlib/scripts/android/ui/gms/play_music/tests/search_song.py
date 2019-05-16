#!/usr/bin/env python

##############################################################################
#
# @filename:    search_song.py
#
# @description: Checking if user can search a song by its exact name
#
# @author:      alexandru.n.branciog@intel.com
#
##############################################################################

import sys
import time
from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.gms.play_music import play_music_steps


from testlib.base.base_utils import get_args

args = get_args(sys.argv)
globals().update(vars(args))
globals().update(eval(script_args[0]))

artist = artist.replace("_", " ")

play_music_steps.open_play_music(serial = serial)()

# click Search
ui_steps.click_button(serial = serial, view_to_find = {"description":"Search"},
                        view_to_check = {"textContains":"Search music"})()

# Input song title
ui_steps.edit_text(serial = serial,
    view_to_find = {"textContains":"Search music"}, value = song_title)()

# Check
ui_steps.wait_for_view(serial = serial, view_to_find = {"text":"Songs"})()
ui_steps.wait_for_view(serial = serial, view_to_find = {"text":artist})()
ui_steps.wait_for_view(serial = serial, view_to_find = {"text":song_title})()
ui_steps.check_object_count(serial = serial,
    view_to_find = {"resourceId":"com.google.android.music:id/play_card"},
    count = 1)()

# Go back to initial app screen
ui_steps.click_button(serial = serial,
    view_to_find = {"description":"Navigate up"}, 
    view_to_check = {"packageName":"com.google.android.music"})()

# press Home
ui_steps.press_home(serial = serial)()
