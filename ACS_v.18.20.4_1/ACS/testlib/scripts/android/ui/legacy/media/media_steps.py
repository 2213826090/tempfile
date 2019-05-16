#!/usr/bin/env python

#######################################################################
#
# @filename:    media_steps.py
# @description: Media test steps
# @author:      costin.carabas@intel.com
#
#######################################################################

import sys
import time

from testlib.scripts.android.ui.ui_step import step as ui_step
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.android.ui import ui_utils
from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.adb.adb_step import step as adb_step
from testlib.scripts.connections.local import local_steps
from testlib.scripts.media import media_utils
from testlib.scripts.android.adb.adb_utils import Sqlite


REFRESH_MEDIA_COMMAND = 'am broadcast -a android.intent.action.MEDIA_MOUNTED' \
                        ' -d file:///'
CAMERA_PATH = '/storage/sdcard0/DCIM/Camera/'
VIDEO_LENGTH = 5
SETTINGS_DB = '/data/data/com.android.providers.settings/databases/settings.db'


class clear_old_media(adb_step):
    """ description:
            Removes all the pictures in media
        usage:
            media_steps.clear_old_media()()
        tags:
            media, android
    """

    def do(self):
        self.adb_connection.run_cmd('rm ' + CAMERA_PATH + '*')
        self.adb_connection.run_cmd(REFRESH_MEDIA_COMMAND + CAMERA_PATH + '*')

    def check_condition(self):
        return not (media_utils.is_media(serial = self.serial,
                                         path = CAMERA_PATH,
                                         grep_for = 'IMG_')
                 or media_utils.is_media(serial = self.serial,
                                         path = CAMERA_PATH,
                                         grep_for = 'VID_'))


class take_picture(ui_step, adb_step):
    """ description:
            Takes a picture with the camera's device
        usage:
            media_steps.take_picture()()
        tags:
            android, media, picture, shutter
    """

    def __init__(self, platform, **kwargs):
        adb_step.__init__(self, **kwargs)
        ui_step.__init__(self, **kwargs)
        self.platform = platform


    def do(self):
        ui_steps.press_home(serial = self.serial,)()
        ui_steps.open_app_from_allapps(serial = self.serial,
                          view_to_find = {'text': 'Camera'},
                          view_to_check = {'descriptionContains': 'Shutter'})()

        if self.platform == "gmin":
            self.uidevice.swipe(serial = self.serial,
                                       sx = 100,
                                       sy = 400,
                                       ex = 400,
                                       ey = 400,
                                       steps = 10)()
            ui_steps.click_button(serial = self.serial,
                                  view_to_find = {
                                    'text': 'Camera'})()
        else:
            ui_steps.click_button(serial = self.serial,
                        view_to_find = {'className': 'android.widget.ImageView',
                                        'descriptionContains': 'Camera'
                                       },
                        view_to_check = {'descriptionContains' : 'Photo'})()
            ui_steps.click_button(serial = self.serial,
                    view_to_find = {'descriptionContains': 'Photo'},
                    view_to_check = {'descriptionContains' :'Shutter'})()

        ui_steps.click_button(serial = self.serial,
                        view_to_find = {'descriptionContains': 'Shutter'},
                        view_to_check= {'descriptionContains': 'Shutter'})()

    def check_condition(self):
        time.sleep(3)
        return media_utils.is_media(serial = self.serial,
                                    path = CAMERA_PATH,
                                    grep_for = 'IMG_')


class record_video(ui_step, adb_step):
    """ description:
            Records a short video
        usage:
            media_steps.record_video()()
        tags:
            android, media, video
    """

    def __init__(self, platform, **kwargs):
        adb_step.__init__(self, **kwargs)
        self.platform = platform


    def do(self):
        ui_steps.press_home(serial = self.serial)()
        ui_steps.open_app_from_allapps(serial = self.serial,
                         view_to_find = {'text': 'Camera'},
                         view_to_check = {'descriptionContains': 'Shutter'})()

        if self.platform == "gmin":
            ui_setps.swipe(serial = self.serial,
                           sx = 100,
                           sy = 400,
                           ex = 400,
                           ey = 400,
                           steps = 10)()
            ui_steps.click_button(serial = self.serial,
                                  view_to_find = {'text': 'Video'})()
        else:
            ui_steps.click_button(serial = self.serial,
                                  view_to_find = {
                                    'className': 'android.widget.ImageView',
                                    'descriptionContains': 'video'},
                                  view_to_check = {
                                    'descriptionContains': 'Video'})()
            ui_steps.click_button(serial = self.serial,
                                  view_to_find = {
                                    'descriptionContains': 'Video'},
                                  view_to_check = {
                                    'descriptionContains': 'Shutter'})()

        ui_steps.click_button(serial = self.serial,
                              view_to_find = {
                                    'descriptionContains': 'Shutter'},
                              view_to_check = {
                                    'descriptionContains': 'Shutter'})()

        time.sleep(VIDEO_LENGTH)
        ui_steps.click_button(serial = self.serial,
                              view_to_find = {
                                    'descriptionContains': 'Shutter'},
                              view_to_check = {
                                    'descriptionContains': 'Shutter'})()


    def check_condition(self):
        return media_utils.is_media(serial = self.serial,
                                    path = CAMERA_PATH,
                                    grep_for = 'VID_')


class modify_sound_status(ui_step, adb_step):
    """ description:
            Modify sound volume
        usage:
            media_steps.modify_sound_status('volume_alarm_speaker', 2)()
        tags:
            android, media, sound, alarm, notifications
    """

    def __init__(self, name, position, **kwargs):
        ui_step.__init__(self, **kwargs)
        adb_step.__init__(self, **kwargs)
        self.name = name
        self.position = position
        self.initial_value = -1
        self.final_value = -1


    def do(self):
        ui_utils.move_slider(serial = self.serial,
                             view_to_find = {
                                        "className":'android.widget.SeekBar',
                                        "instance": self.position
                                            },
                             position = 0)
        self.initial_value = Sqlite.get_value_from_db(
                                                 self,
                                                 db = SETTINGS_DB,
                                                 table = 'system',
                                                 columns = ["value"],
                                                 where_columns = ["name"],
                                                 where_values = [self.name]
                                                    )

        ui_utils.move_slider(serial = self.serial,
                             view_to_find = {
                                        "className":'android.widget.SeekBar',
                                        "instance": self.position
                                            },
                             position = 100)
        self.final_value = Sqlite.get_value_from_db(
                                                 self,
                                                 db = SETTINGS_DB,
                                                 table = 'system',
                                                 columns = ["value"],
                                                 where_columns = ["name"],
                                                 where_values = [self.name]
                                                    )

    def check_condition(self):
        return int(self.initial_value[0]) != int(self.final_value[0])


class mute_sound(ui_step, adb_step):
    """ description:
            Mute volume
        usage:
            media_steps.mute_sound('volume_alarm_speaker', 2)()
        tags:
            android, media, sound, alarm, notifications
    """

    def __init__(self, name, position, **kwargs):
        ui_step.__init__(self, **kwargs)
        adb_step.__init__(self, **kwargs)
        self.name = name
        self.position = position
        self.initial_value = -1


    def do(self):
        ui_utils.move_slider(serial = self.serial,
                             view_to_find = {
                                        "className":'android.widget.SeekBar',
                                        "instance": self.position
                                            },
                             position = 0)

        self.intial_value = Sqlite.get_value_from_db(
                                                 self,
                                                 db = SETTINGS_DB,
                                                 table = 'system',
                                                 columns = ["value"],
                                                 where_columns = ["name"],
                                                 where_values = [self.name]
                                                    )


    def check_condition(self):
        return int(self.intial_value[0]) == 0


class check_resolution(ui_step, adb_step):
    """ description:
            Check resolution of the device's screen
        usage:
            media_steps.check_resolution(screenshot_name = 'name')()
        tags:
            android, media, resolution
    """

    def __init__(self, screenshot_name, **kwargs):
        ui_step.__init__(self, **kwargs)
        adb_step.__init__(self, **kwargs)
        self.screenshot_name = screenshot_name

    def do(self):
        self.uidevice.screenshot(self.screenshot_name)

    def check_condition(self):
        a = local_steps.command('file ' + self.screenshot_name)()
        screenshot_resolution = a[0].split(',')[1]
        prop_resolution = self.adb_connection.parse_cmd_output(
                        cmd = 'getprop ro.sf.lcd_density_info').split('px')[0]
        return screenshot_resolution.strip() == prop_resolution.strip()
