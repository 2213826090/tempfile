#!/usr/bin/env python

##############################################################################
#
# @filename:    search_video.py
#
# @description: Checking search results
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

youtube_steps.search_video(serial = serial, search_for = search_for,
                            items_count = 5, comparator = ">")()

# Go back to initial app screen
ui_steps.click_button(serial = serial,
    view_to_find = {"description":"Navigate up"},
    view_to_check = {"packageName":"com.google.android.youtube"})()

# Close app
ui_steps.close_app_from_recent(serial = serial,
        view_to_find={"text": "YouTube"})()
