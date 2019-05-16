#Copyright (C) 2014  Yi, GraceX <gracex.yi@intel.com>
#Intel Corporation All Rights Reserved.

#The source code contained or described herein and
#all documents related to the source code ("Material") are owned by
#Intel Corporation or its suppliers or licensors.

#Title to the Material remains with Intel Corporation or
#its suppliers and licensors.
#The Material contains trade secrets and proprietary and
#confidential information of Intel or its suppliers and licensors.
#The Material is protected by worldwide copyright and
#trade secret laws and treaty provisions.
#No part of the Material may be used, copied, reproduced, modified,
#published, uploaded, posted, transmitted, distributed
#or disclosed in any way without Intel's prior express written permission.
#No license under any patent, copyright, trade secret or
#other intellectual property right is granted to
#or conferred upon you by disclosure or delivery of the Materials,
#either expressly, by implication, inducement, estoppel or otherwise.

#Any license under such intellectual property rights must be express
#and approved by Intel in writing.

"""
@summary: This file implements for music function
@since: 06/12/2014
@author: Grace Yi (gracex.yi@intel.com)
"""

import os
import time
from testlib.util.common import g_common_obj

class MusicImpl:
    """
    @summary: The basic function of music
    """

#--------- begin locator -------------
    class Locator(object):
        """
            Helper for locator UI Object
        """

        def __init__(self, device):
            self.d = device

        @property
        def btn_apps(self):
            """ UI button apps """
            return self.d(description="Apps")

        @property
        def btn_inactive(self):
            """ UI button inactive """
            return self.d(resourceId=\
                "com.google.android.googlequicksearchbox:id/inactive")


        @property
        def btn_select_account(self):
            """ UI button select account"""
            return self.d(text="Select an account")

        @property
        def btn_account_skip(self):
            """ UI button account skip """
            return self.d(text="Skip", className="android.widget.Button")

        @property
        def btn_not_now(self):
            """ UI button account not now """
            return self.d(text="Not now", \
                resourceId="com.google.android.music:id/skip_button")

        @property
        def btn_listen_now(self):
            """ UI button listen now """
            return self.d(text="Listen Now", \
                resourceId="com.google.android.music:id/text")

        def btn_music_title(self, title):
            """ UI button music title """
            #return self.d(text=title,resourceId="com.google.android.music:id/li_title")
            #return self.d(resourceId="android:id/list").\
            return self.d(resourceId="com.google.android.music:id/recycler_view"). \
            child_by_text(title, allow_scroll_search=True, \
                resourceId="com.google.android.music:id/li_title")

        def btn_music(self, music):
            """ UI button music to play """
            return self.d(resourceId="android:id/list").\
            child_by_text(music, \
                allow_scroll_search=True, \
                resourceId="com.google.android.music:id/line1")

        @property
        def btn_music_repeat(self):
            """ UI button music repeat """
            return self.d(resourceId="com.google.android.music:id/repeat")

        @property
        def btn_music_pause(self):
            """ UI button music pause """
            return self.d(resourceId="com.google.android.music:id/pause")

        @property
        def btn_music_album(self):
            """ UI button music album art """
            return self.d(index="0", \
                resourceId="com.google.android.music:id/album_art")

        @property
        def btn_music_clear(self):
            """ UI button music clear """
            return self.d(text="Clear cache")

        @property
        def btn_music_stop(self):
            """ UI button music clear """
            return self.d(text="Force stop")

        @property
        def btn_ok(self):
            """ UI button ok """
            return self.d(text="OK", \
                resourceId="android:id/button1")

    packagename = "com.google.android.music"
    activityname = "com.google.android.music.ui.TrackContainerActivity"

    def __init__ (self, cfg):
        self.d = g_common_obj.get_device()
        self.cfg = cfg
        self._locator = MusicImpl.Locator(self.d)

    @staticmethod
    def __search_from_logcat(message):
        """
        @summary: search the message from log
        @parameter:
            message : the message searched
        @return: True or False
        """
        cmd = "adb logcat -d|grep %s" % message
        pipe = os.popen(cmd).read().strip()
        #print "[@@Grace_Debug]:", pipe, len(pipe)
        if len(pipe) == 0:
            print("Search %s is None" % message)
            return False
        print("INFO:Get the key log in logcat!")
        return True

    def music_pause(self):
        self._locator.btn_music_pause.click()

    #add by samx.lan@intel.com
    #aim to read audio1 and audio 2 in music.conf file 
    def get_cfg_music_btn(self):
        print "[INFO]: The music name to play is: %s" % self.cfg.get("music_name")
        return self.d(resourceId="android:id/list").\
        child_by_text(self.cfg.get("music_name"),\
            allow_scroll_search=True, \
            resourceId="com.google.android.music:id/line1")
    
    #music section is ie: [audio3] set up in the conf file
    def music_play(self, music_section=None):
        """
        @summary: launch music play activity
        @return: None
        """
        play_message = "android.media.action.OPEN_AUDIO_EFFECT_CONTROL_SESSION"
        #g_common_obj.launch_app_am(self.packagename, self.activityname)
        self.launch_app_from_home_sc("Play Music")

        #Skip set up screen while launch the music app in the first time
        time.sleep(5)
        if self._locator.btn_not_now.exists:
            self._locator.btn_not_now.click()
            time.sleep(1)
        if self._locator.btn_select_account.exists:
            self._locator.btn_account_skip.click()
        if self._locator.btn_listen_now.exists:
            self._locator.btn_listen_now.click()
            time.sleep(1)
        time.sleep(1)

        #read music tile from config file
        music_title=self.cfg.get("music_title")
        music_name=self.cfg.get("music_name")
        assert self._locator.btn_music_title(music_title).exists, \
        "ERROR: Not find music folder in list, Please run deploy.sh first"
        self._locator.btn_music_title(music_title).click()
        time.sleep(1)
        if music_section==None:
            assert self._locator.btn_music(music_name).exists, \
            "ERROR: Not find music folder in list, Please run deploy.sh first"
            self._locator.btn_music(music_name).long_click()
        else:
            music_to_play=self.get_cfg_music_btn()
            assert music_to_play.exists, \
            "ERROR: Not find music folder in list, Please run deploy.sh first"
            music_to_play.long_click()

        time.sleep(2)
        assert self.__search_from_logcat(play_message), "Play music error"
        print("INFO:The music is playing")
        return True

    def music_play_for_times(self, count):
        """
        @summary: play music for times
        @parameter:
            count: play the music via gestures control for count times
        @return None
        """
        print "[START]:Test music play for [%d] times!" % count
        play_message = "android.media.action.OPEN_AUDIO_EFFECT_CONTROL_SESSION"
        try:
            if 0 == count:
                print "ERROR:The times should not be 0!"
                raise TypeError
        except TypeError:
            print "The times of music playing should be more than 1"
        self.music_play()
        time.sleep(10)
        print ("INFO:Play for [1] time:PASS")

        for i in range(1, int(count)):
            #print i, type(i)
            os.system("adb logcat -c")
            self._locator.btn_music_album.click()
            time.sleep(10)
            assert self.__search_from_logcat(play_message), "Play music error"
            print("INFO:Play for [%d] times:PASS" % int(i+1))
        print "[END]:Test music play for [%d] times!\n" % count
        return True

    def music_play_for_longtime(self, seconds):
        """
        @summary: play music for a long time
        @parameter:
            seconds: the seconds music lasting
        @return None
        """
        seconds = 10
        print "Test play %d sec" % seconds
        hours = seconds / 3600
        mins = (seconds % 3600) / 60
        second = seconds % 60
        play_message = "android.media.action.OPEN_AUDIO_EFFECT_CONTROL_SESSION"
        try:
            if 0 == seconds:
                print "ERROR:The times should not be 0!"
                raise TypeError
        except TypeError:
            print "The times of music playing should be more than 1"
        self.music_play()
        time.sleep(1)
        self.d(index="0", \
            resourceId="com.google.android.music:id/header_metadata").click()
        time.sleep(1)
        self._locator.btn_music_repeat.click()
        print "INFO:Now is " + time.strftime("%Y-%m-%d %H:%M:%S")
        print "[START]:Test music play for [%d]hours [%d]mins [%d]seconds!" \
        % (int(hours), int(mins), int(second))

        hcount = 1
        while int(hours) != int(0):
            print("[INFO:Sleep for %d hour]"% hcount)
            time.sleep(3600)
            hcount += 1
            hours -= 1
        mcount = 1
        while int(mins) != int(0):
            print("[INFO:Sleep for %d min]" % mcount)
            time.sleep(60)
            mcount += 1
            mins -= 1
        scount = 1
        while int(second) != int(0):
            print("[INFO:Sleep for %d second]" % scount)
            time.sleep(1)
            scount += 1
            second -= 1

        print "INFO:Sleep END " + time.strftime("%Y-%m-%d %H:%M:%S")
        os.system("adb logcat -c")
        self._locator.btn_music_pause.click()
        time.sleep(2)
        self._locator.btn_music_pause.click()
        time.sleep(2)
        assert self.__search_from_logcat(play_message), "Play music error"
        print "[END]:\
        Test music play for [%d]hours [%d]mins [%d]seconds!\n" % \
        (int(seconds/3600), int((seconds%3600)/60), int(seconds%60))
        return True

    def music_clear(self):
        """
        @summary: clear the music data
        @return: None
        """
        print("INFO:Clear Play Music app data")
        cmd = self.cfg.get("music_clear")
        g_common_obj.adb_cmd(cmd)
        time.sleep(3)
        self._locator.btn_music_clear.click()
        time.sleep(1)
        self._locator.btn_music_stop.click()
        time.sleep(1)
        if self._locator.btn_ok.exists:
            self._locator.btn_ok.click()

    def launch_app_from_home_sc(self, appname):
        """
            restrute for there is no switch widget/apps in app screen
        """
        iffind = False
        self.d.press.home()
        self._locator.btn_apps.click()
        time.sleep(2)
        count = int(self._locator.btn_inactive.count)
        for i in range(0, count*2):
            time.sleep(2)
            if self.d(text=appname).exists:
                self.d(text=appname).click()
                iffind = True
                break
            if i < count:
                self.d(scrollable=True).scroll.horiz()
            else:
                self.d(scrollable=True).scroll.horiz.backward()
        assert iffind == True
