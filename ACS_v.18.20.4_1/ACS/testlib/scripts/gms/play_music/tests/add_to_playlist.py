#!/usr/bin/env python

##############################################################################
#
# @filename:    add_to_playlist.py
#
# @description: Add song to playlist
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

play_music_steps.open_play_music(serial = serial)()

play_music_steps.go_to_songs(serial = serial)()

ui_steps.click_button(serial = serial, view_to_find = {"text":song_title},
    right_view_to_find = {"description":"Options"},
    view_to_check = {"text":"Add to playlist"})()

ui_steps.click_button(serial = serial,
    view_to_find = {"text":"Add to playlist"},
    view_to_check = {"text":"New playlist"})()

ui_steps.click_button(serial = serial, 
    view_to_find = {"text":"New playlist"},
    view_to_check = {"text":"Create playlist"})()

ui_steps.edit_text(serial = serial, 
    view_to_find = {"textContains":"Name"},
    value = playlist_name)()

ui_steps.click_button(serial = serial, 
    view_to_find = {"text":"Create playlist"},
    view_to_check = {"text": "SONGS"})()

ui_steps.click_button(serial = serial, 
    view_to_find = {"description":"Show navigation drawer"},
    view_to_check = {"text": "Playlists"})()

ui_steps.click_button(serial = serial, 
    view_to_find = {"text": "Playlists"},
    view_to_check = {"text": "Recent Playlists"})()

ui_steps.click_button(serial = serial, 
    view_to_find = {"text": playlist_name},
    view_to_check = {"text": song_title})()

# Delete the playlist
ui_steps.click_button(serial = serial, view_to_find = {"text":"My playlist"},
    right_view_to_find = {"description":"Options"},
    view_to_check = {"text":"Delete"})()

ui_steps.click_button(serial = serial, view_to_find = {"text":"Delete"},
    view_to_check = {"text":"OK"})()

ui_steps.click_button(serial = serial, view_to_find = {"text":"OK"},
    view_to_check = {"text": song_title}, view_presence = False)()

# Go back to initial app screen
play_music_steps.go_to_listen_now(serial = serial)()
# press Home
ui_steps.press_home()()
