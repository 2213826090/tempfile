#!/usr/bin/env python

##############################################################################
#
# @filename:    go_to_artist.py
#
# @description: Go to artist option check
#
# @author:      alexandru.n.branciog@intel.com
#
##############################################################################

import sys
import time
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.gms.play_music import play_music_steps


from testlib.base.base_utils import get_args

args = get_args(sys.argv)
globals().update(vars(args))
globals().update(eval(script_args[0]))

artist = artist.replace("_", " ")

play_music_steps.open_play_music(serial = serial)()

play_music_steps.go_to_songs(serial = serial)()

ui_steps.click_button(serial = serial, view_to_find = {"text":song_title},
                        right_view_to_find = {"description":"Options"},
                        view_to_check = {"text":"Go to artist"})()

ui_steps.click_button(serial = serial, view_to_find = {"text":"Go to artist"},
                        view_to_check = {"text":artist})()

# Go back to initial app screen
play_music_steps.go_to_listen_now(serial = serial)()
# press Home
ui_steps.press_home(serial = serial)()
