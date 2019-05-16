#!/usr/bin/env python

##############################################################################
#
# @filename:    upload_video.py
#
# @description: Upload a video to youtube
#
# @author:      alexandru.n.branciog@intel.com
#
##############################################################################

import sys
import time
import datetime
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.gms.youtube import youtube_steps

from testlib.base.base_utils import get_args

args = get_args(sys.argv)
globals().update(vars(args))
globals().update(eval(script_args[0]))

# Ensure video title is unique
video_title = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")

ui_steps.press_home(serial = serial)()

youtube_steps.open_youtube(serial = serial)()

# Make sure user is signed in before trying to upload video
youtube_steps.sign_in(serial = serial, account = account,
                        password = password, force = False)()

ui_steps.click_button(serial = serial,
    view_to_find = {"description":"Show navigation drawer"},
    view_to_check = {"text":"Uploads"})()

ui_steps.click_button(serial = serial, view_to_find = {"text":"Uploads"},
    view_to_check = {"resourceId":\
    "com.google.android.youtube:id/menu_upload"})()

ui_steps.click_button(serial = serial, view_to_find = {"resourceId":\
    "com.google.android.youtube:id/menu_upload"},
    view_to_check = {"packageName":"com.android.documentsui"})()

ui_steps.click_button_if_exists(serial = serial,
    view_to_find = {"description":"Show roots"})()

ui_steps.click_button(serial = serial, view_to_find = {"text":"Videos"},
    view_to_check = {"text":"Movies"})()

ui_steps.show_as_list(serial = serial)()

ui_steps.click_button(serial = serial, view_to_find = {"text":"Movies"},
    view_to_check = {"text":"Movies",
                    "resourceId":"android:id/title"})()

ui_steps.show_as_list(serial = serial)()

ui_steps.click_button(serial = serial, view_to_find = {"text":video_file},
    view_to_check = {"packageName":"com.google.android.youtube"})()

ui_steps.click_button_if_exists(serial = serial, view_to_find = {"text":"OK"})()

ui_steps.edit_text(serial = serial,
            view_to_find = {"textContains":"Title"},
            value = video_title)()

if not ui_steps.click_button_if_exists(serial = serial,
                        view_to_find = {"text": "Private"})():
    if not ui_steps.click_button_if_exists(serial = serial,
                        view_to_find = {"text": "Public"})():
        ui_steps.click_button(serial = serial,
                                view_to_find = {"text": "Unlisted"})()
ui_steps.click_button(serial = serial, view_to_find = {"text": "Public"})()

ui_steps.click_button(serial = serial, view_to_find = {"description":"Upload"},
    view_to_check = {"packageName":"com.google.android.youtube"})()

# It takes a bit for the video to be uploaded and processed
ui_steps.wait_for_view(view_to_find = {"textContains":"ago"}, timeout = 600)()
# Waiting 2 minutes for the video to become visible on youtube
time.sleep(120)

ui_steps.click_button(serial = serial,
    view_to_find = {"description":"Show navigation drawer"},
    view_to_check = {"text":"What to Watch"})()

ui_steps.click_button(serial = serial,
    view_to_find = {"text":"What to Watch"},
    view_to_check = {"text":"What to Watch"}, wait_time = 5000)()

ui_steps.click_button_if_exists(serial = serial, view_to_find = {"text":"OK"})()

youtube_steps.search_video(serial = serial, search_for = video_title,
                            items_count = 1, comparator = ">")()

ui_steps.click_button(serial = serial,
    view_to_find = {"description":"Navigate up"},
    view_to_check = {"packageName":"com.google.android.youtube"})()

ui_steps.click_button(serial = serial,
    view_to_find = {"description":"Show navigation drawer"},
    view_to_check = {"text":"Uploads"})()

ui_steps.click_button(serial = serial,
    view_to_find = {"text":"Uploads"},
    view_to_check = {"resourceId":\
    "com.google.android.youtube:id/menu_upload"})()

ui_steps.click_button(serial = serial,
    view_to_find = {"description":"Menu"},
    view_to_check = {"text":"Delete"})()

ui_steps.click_button(serial = serial, view_to_find = {"text":"Delete"},
    view_to_check = {"textContains":"Delete this video"})()

ui_steps.click_button(serial = serial, view_to_find = {"text":"OK"},
    view_to_check = {"resourceId":\
    "com.google.android.youtube:id/menu_upload"})()

# Go back to initial app screen
ui_steps.click_button(serial = serial,
    view_to_find = {"description":"Show navigation drawer"},
    view_to_check = {"packageName":"com.google.android.youtube"})()

ui_steps.click_button(serial = serial,
    view_to_find = {"text":"What to Watch"},
    view_to_check = {"text":"What to Watch"}, wait_time = 5000)()

# Close app
ui_steps.close_app_from_recent(serial = serial,
        view_to_find={"text": "YouTube"})()
