#!/usr/bin/env python

##############################################################################
#
# @description: ET/../Settings/Settings Volume
#               Change the 3 volumes in settings / sound and verify with values
#               in DB.
# @author:      silviux.l.andrei@intel.com
#
##############################################################################

import sys
from testlib.scripts.android.ui.settings import steps
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.android.ui import ui_utils
from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.adb import adb_utils
from testlib.scripts.android.ui.ui_step import step as ui_step
from testlib.base.base_utils import get_args


globals().update(vars(get_args(sys.argv)))
adb_steps.connect_device(serial = serial, port = adb_server_port)()
globals().update({"version": adb_utils.get_android_version()})

#adb_steps.root_connect_device()()

if version == "L":
    music_instance_no = 0
    music_sqlite_field = "volume_music_speaker"
    ring_instance_no = 2
    ring_sqlite_field = "volume_ring_speaker"
    alarm_instance_no = 1
    alarm_sqlite_field = "volume_alarm_speaker"

elif version == "K":
    music_instance_no = 0
    music_sqlite_field = "volume_music_speaker"
    ring_instance_no = 1
    ring_sqlite_field = "volume_ring_speaker"
    alarm_instance_no = 2
    alarm_sqlite_field = "volume_alarm_speaker"
    

ui_steps.open_settings_app(view_to_find = {"textContains": "Sound"}, 
                           view_to_check = {"text": "Sound"})()

if version == "K":
    ui_steps.click_button(view_to_find = {"text":"Volumes"},
                          view_to_check= {"textContains":
                                         "Music, video, games"})()
# notifications slider
steps.move_volume_slider(view_to_find = {"className":'android.widget.SeekBar',
                                         "instance": ring_instance_no},
                         position = 0,
                         db = "/data/data/com.android.providers.settings"
                              "/databases/settings.db",
                         table = "system",
                         columns = ["value"],
                         values = ["0"],
                         where_columns = ["name"],
                         where_values = [ring_sqlite_field],
                         blocking = False,
                         critical = False)()
# music slider
steps.move_volume_slider(view_to_find = {"className":'android.widget.SeekBar',
                                         "instance": music_instance_no},
                         position = 0,
                         db = "/data/data/com.android.providers.settings"
                              "/databases/settings.db",
                         table = "system",
                         columns = ["value"],
                         values = ["0"],
                         where_columns = ["name"],
                         where_values = [music_sqlite_field],
                         blocking = False,
                         critical = False)()
# alarms slider
steps.move_volume_slider(view_to_find = {"className":'android.widget.SeekBar',
                                         "instance": alarm_instance_no},
                         position = 0,
                         db = "/data/data/com.android.providers.settings"
                              "/databases/settings.db",
                         table = "system",
                         columns = ["value"],
                         values = ["0"],
                         where_columns = ["name"],
                         where_values = [alarm_sqlite_field],
                         blocking = False,
                         critical = False)()

ui_steps.press_home()()

