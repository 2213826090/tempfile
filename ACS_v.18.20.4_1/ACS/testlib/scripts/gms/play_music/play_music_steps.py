#!/usr/bin/env python

##############################################################################
#
# @filename:    play_music_steps.py
#
# @description: Play music app test steps
#
# @author:      alexandru.n.branciog@intel.com
#
##############################################################################

import time
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.android.ui.ui_step import step as ui_step
from testlib.scripts.android.adb import adb_steps

class open_play_music(ui_step):
    """ description:
            opens Play Music app. If opened for first time
            will click Skip button  

        usage:
            play_music_steps.open_play_music(serial = serial)()

        tags:
            ui, android, music 
    """

    def do(self):
        # sending MEDIA_MOUNTED intent to have
        # audio files available for Play Music app
        adb_steps.command(serial = self.serial, command = "am broadcast -a\
 android.intent.action.MEDIA_MOUNTED -d file:///storage/sdcard0/Music",
                            timeout = 10)()
        # Close app before openning it to ensure first screen consistency
        ui_steps.close_app_from_recent(serial = self.serial,
                                       view_to_find={"text": "Play Music"})()
        ui_steps.press_home(serial = self.serial)()
        ui_steps.open_app_from_allapps(serial = self.serial, 
            view_to_find = {"text": "Play Music"},
            view_to_check = {"packageName":"com.google.android.music"})()
        time.sleep(5)
        ui_steps.click_button_if_exists(serial = self.serial,
                    view_to_find = {"text":"Use Standard"})()
        ui_steps.click_button_if_exists(serial = self.serial,
                    view_to_find = {"text":"Done"})()
        ui_steps.click_button_if_exists(serial = self.serial,
                    view_to_find = {"text":"Skip"})()
        ui_steps.click_button_if_exists(serial = self.serial, 
                    view_to_find = {"description":"Navigate up"})()
        try:
            ui_steps.wait_for_view(serial = self.serial, 
                    view_to_find = {"text":"Listen Now"})()
        except:
            ui_steps.click_button(serial = self.serial, 
                    view_to_find = {"description":"Show navigation drawer"},
                    view_to_check = {"text": "Listen Now"})()
            ui_steps.click_button(serial = self.serial,
                    view_to_find = {"text": "Listen Now"},
                    view_to_check = {"text": "Listen Now"})()


class go_to_songs(ui_step):
    """ description:
            Navigates to SONGS

        usage:
            play_music_steps.go_to_songs(serial = serial)()

        tags:
            ui, android, music 
    """

    def do(self):
        ui_steps.click_button(serial = self.serial, 
            view_to_find = {"description":"Show navigation drawer"},
            view_to_check = {"text": "My Library"})()
        ui_steps.click_button(serial = self.serial,
            view_to_find = {"text": "My Library"}, 
            view_to_check = {"text": "SONGS"})()
        ui_steps.click_button(serial = self.serial,
            view_to_find = {"text": "SONGS"}, 
            view_to_check = {"text": "SHUFFLE ALL"})()

class go_to_listen_now(ui_step):
    """ description:
            Navigates to Listen Now

        usage:
            play_music_steps.go_to_listen_now(serial = serial)()

        tags:
            ui, android, music 
    """

    def do(self):
        ui_steps.click_button(serial = self.serial, 
            view_to_find = {"description":"Navigate up"}, 
            view_to_check = {"packageName":"com.google.android.music"})()
        ui_steps.click_button(serial = self.serial, 
            view_to_find = {"description":"Show navigation drawer"},
            view_to_check = {"text": "Listen Now"})()
        ui_steps.click_button(serial = self.serial, 
            view_to_find = {"text": "Listen Now"},
            view_to_check = {"text": "Listen Now"})()
