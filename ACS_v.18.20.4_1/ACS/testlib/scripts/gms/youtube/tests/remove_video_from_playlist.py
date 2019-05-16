#!/usr/bin/env python

##############################################################################
#
# @filename:    remove_video_from_playlist.py
#
# @description: Remove video from a playlist
#
# @author:      alexandru.n.branciog@intel.com
#
##############################################################################

import sys
import time
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.gms.youtube import youtube_steps

from testlib.base.base_utils import get_args

args = get_args(sys.argv)
globals().update(vars(args))
globals().update(eval(script_args[0]))
search_for = search_for.replace("_", " ")

ui_steps.press_home(serial = serial)()
youtube_steps.open_youtube(serial = serial)()

# Make sure user is signed in
youtube_steps.sign_in(serial = serial, account = account,
                        password = password, force = True)()

youtube_steps.search_video(serial = serial, search_for = search_for,
                            items_count = 5, comparator = ">")()

# Add video to playlist
ui_steps.click_button(serial = serial,
    view_to_find = {"description":"Menu"},
    view_to_check = {"textMatches":"Add to[^\s]"})()

ui_steps.click_button(serial = serial,
    view_to_find = {"textMatches":"Add to[^\s]"},
    view_to_check = {"text":playlist_name})()

ui_steps.click_button(serial = serial,
    view_to_find = {"text":"Autotest"},
    view_to_check = {"description":"Navigate up"})()

# Go back to initial app screen
ui_steps.click_button(serial = serial,
    view_to_find = {"description":"Navigate up"},
    view_to_check = {"packageName":"com.google.android.youtube"})()

ui_steps.click_button(serial = serial,
    view_to_find = {"description":"Show navigation drawer"},
    view_to_check = {"text":playlist_name})()

ui_steps.click_button(serial = serial,
    view_to_find = {"text":playlist_name},
    view_to_check = {"textContains":search_for})()

ui_steps.click_button(serial = serial,
    view_to_find = {"description":"Menu", "instance":3},
    view_to_check = {"text":"Remove"})()

ui_steps.click_button(serial = serial,
    view_to_find = {"text":"Remove"},
    view_to_check = {"textContains":search_for},
    view_presence = False)()

# Close app
ui_steps.close_app_from_recent(serial = serial,
        view_to_find={"text": "YouTube"})()
