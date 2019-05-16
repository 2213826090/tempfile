# Copyright (C) 2015  Jin, YingjunX <yingjunx.jin@intel.com>
# Intel Corporation All Rights Reserved.

# The source code contained or described herein and
# all documents related to the source code ("Material") are owned by
# Intel Corporation or its suppliers or licensors.

# Title to the Material remains with Intel Corporation or
# its suppliers and licensors.
# The Material contains trade secrets and proprietary and
# confidential information of Intel or its suppliers and licensors.
# The Material is protected by worldwide copyright and
# trade secret laws and treaty provisions.
# No part of the Material may be used, copied, reproduced, modified,
# published, uploaded, posted, transmitted, distributed
# or disclosed in any way without Intel's prior express written permission.
# No license under any patent, copyright, trade secret or
# other intellectual property right is granted to
# or conferred upon you by disclosure or delivery of the Materials,
# either expressly, by implication, inducement, estoppel or otherwise.

# Any license under such intellectual property rights must be express
# and approved by Intel in writing.

'''
@summary: Class for ChromeCast operation
@since: 03/25/2015
@author: Yingjun Jin
'''


import time
import os
from testlib.util.common import g_common_obj
from testlib.util.config import TestConfig
from testlib.util.repo import Artifactory
from testlib.graphics.tools import ConfigHandle


class Locator(object):

    """
    locator
    """

    def __init__(self, device):
        self.d = device

    @property
    def wait_exist(self):
        """ wait until the ui object exist """
        def _wait(uiobj, timeout=5):
            return uiobj.wait("exists", timeout * 500)
        return _wait

    @property
    def performance_tests(self):
        return self.d(text="Performance Tests")


class RenderImpl:

    '''
    classdocs
    '''

    def __init__(self):
        self.d = g_common_obj.get_device()
        self._locator = Locator(self.d)

    def install_apk(self, apk_name):
        """ Install the apk from artifactory
        """
        config = TestConfig()
        cfg_file = 'tests.tablet.basic_render_script.conf'
        cfg_arti = config.read(cfg_file, 'artifactory')
        config_handle = ConfigHandle()
        cfg_arti["location"] = config_handle.read_configuration('artifactory', 'location', '/etc/oat/', 'sys.conf')
        arti = Artifactory(cfg_arti.get('location'))
        cfg_apk = config.read(cfg_file, apk_name)
        apk_name = cfg_apk.get("name")
        file_path = arti.get(apk_name)
        g_common_obj.adb_cmd_common('install ' + file_path)

    def play_video_in_photos(self):
        """ play video in photos
        """
        print "play video in photos"
        self.d(resourceId="com.google.android.apps.plus:id/tile_row")\
            .child(className="android.view.View").click()
        time.sleep(2)
        self.d(description="Play video").click()
        time.sleep(10)

    def launch_photos_am(self):
        """ Launch Photos via adb am command
        """
        print "Launch Photos by adb am"
        g_common_obj.launch_app_am("com.google.android.apps.plus" \
                                   , "com.google.android.apps.photos.phone.PhotosHomeActivity")
        time.sleep(2)
        if self.d(text="Later").exists:
            self.d(text="Later").click()
            time.sleep(2)
        self._locator.wait_exist(self._locator.performance_tests)
        if self.d(text="No thanks").exists:
            self.d(text="No thanks").click()
            time.sleep(5)

    @staticmethod
    def stop_photos_am():
        """ Stop Photos via adb am command
        """
        print "Stop Photos by adb am"
        g_common_obj.stop_app_am("com.google.android.apps.plus")

    def launch_fountain_am(self):
        """ Launch Fountain via adb am command
        """
        print "Launch Fountain by adb am"
        g_common_obj.launch_app_am("com.example.android.rs.fountain",
                                   "com.example.android.rs.fountain.Fountain")
        self._locator.wait_exist(self._locator.performance_tests)

    @staticmethod
    def stop_fountain_am():
        """ Stop Fountain via adb am command
        """
        print "Stop Fountain by adb am"
        g_common_obj.stop_app_am("com.example.android.rs.fountain")

    def suspend_and_resume(self):
        """ suspend and resume the DUT
        """
        self.d.press.power()
        time.sleep(5)
        self.d.press.power()
        time.sleep(2)

    def launch_photovisualizer_am(self):
        """ Launch PhotoVisualizer via adb am command
        """
        print "Launch PhotoVisualizer by adb am"
        g_common_obj.launch_app_am("jp.kiroru_inc.photo_visualizer",
                                   "jp.kiroru_inc.photo_visualizer.MainActivity")
        self._locator.wait_exist(self._locator.performance_tests)

    @staticmethod
    def stop_photovisualizer_am():
        """ Stop PhotoVisualizer via adb am command
        """
        print "Stop PhotoVisualizer by adb am"
        g_common_obj.stop_app_am("jp.kiroru_inc.photo_visualizer")

    def run_photovisualizer(self):
        """ start and stop the PhotoVisualizer
        """
        print "Start the PhotoVisualizer"
        self.d(text="Start").click()
        time.sleep(5)
        if not self.d(text="Settings").exists:
            self.d.press.back()
        time.sleep(2)

    def init_photos(self):
        """ push some pictures in DUT
        """
        config = TestConfig()
        cfg_file = 'tests.tablet.basic_render_script.conf'
        cfg_arti = config.read(cfg_file, 'artifactory')
        config_handle = ConfigHandle()
        cfg_arti["location"] = config_handle.read_configuration('artifactory', 'location', '/etc/oat/', 'sys.conf')
        arti = Artifactory(cfg_arti.get('location'))
        cfg_name = config.read(cfg_file, 'girls')
        pictures_name = cfg_name.get("name")
        self.floder_name = cfg_name.get('floder')
        pictures_path = arti.get(pictures_name)
        print os.system("cd " + os.path.dirname(pictures_path) +
                        ";tar xzf " + pictures_path)
        unzip_photos = pictures_path.replace(".tar.gz", "/")
        cmd = "mkdir -p %s" % self.floder_name
        g_common_obj.adb_cmd_common(cmd)
        cmd = "push %s /sdcard/%s" % (unzip_photos, self.floder_name)
        g_common_obj.adb_cmd_common(cmd)
        cmd = "am broadcast -a android.intent.action.MEDIA_MOUNTED \
        -d file:///sdcard/"
        g_common_obj.adb_cmd(cmd)

    def delete_photos(self):
        """ Delete the photos tested
        """
        print "Delete the photos"
        cmd = "rm -rf /sdcard/%s " % self.floder_name
        g_common_obj.adb_cmd(cmd)
        cmd = "am broadcast -a android.intent.action.MEDIA_MOUNTED \
        -d file:///sdcard/"
        g_common_obj.adb_cmd(cmd)

    def set_photovisualizer_settings_1st(self):
        """ Set the photovisualizer settings 1st
        """
        self.d(text="Settings").click()
        cmd = "jp.kiroru_inc.photo_visualizer:id/settings_directory_set"
        self.d(resourceId=cmd).click()
        if self.d().scroll.vert.to(text="girls"):
            self.d(text="girls").click()
        cmd = "jp.kiroru_inc.photo_visualizer:id/settings_dir_select_set"
        self.d(resourceId=cmd).child(className="android.widget.RadioButton"
                                     ).click()
        self.d.press.back()
        cmd = "jp.kiroru_inc.photo_visualizer:id/settings_clock_color_set"
        self.d(resourceId=cmd).click()
        self.d(text="Lemon Yellow").click()
        time.sleep(2)
        cmd = "jp.kiroru_inc.photo_visualizer:id/settings_clock_font_set"
        # self.d.press.recent()
        time.sleep(2)
        # self.d.press.back()
        self.d(resourceId=cmd).click()
        self.d(text="Serif").click()
        cmd = "jp.kiroru_inc.photo_visualizer:id/settings_clock_location_set"
        # self.d.press.recent()
        time.sleep(2)
        # self.d.press.back()
        self.d(resourceId=cmd).click()
        self.d(text="Top-Left").click()
        self.d.press.back()

    def set_photovisualizer_settings_2st(self):
        """ Set the photovisualizer settings 2st
        """
        if not self.d(text="Settings").exists:
            self.launch_photovisualizer_am()
            time.sleep(1)
        self.d(text="Settings").click()
        cmd = "jp.kiroru_inc.photo_visualizer:id/settings_clock_location_set"
        self.d(resourceId=cmd).click()
        self.d(text="Bottom-Right").click()
        self.d.press.back()

    def launch_rsballs_am(self):
        """ Launch RsBalls via adb am command
        """
        print "Launch RsBalls by adb am"
        g_common_obj.launch_app_am("com.example.android.rs.balls",
                                   "com.example.android.rs.balls.Balls")
        self._locator.wait_exist(self._locator.performance_tests)

    @staticmethod
    def stop_rsballs_am():
        """ Stop RsBalls via adb am command
        """
        print "Stop RsBalls by adb am"
        g_common_obj.stop_app_am("com.example.android.rs.balls")

    def back_to_rsballs(self):
        """ Via recent back to rsballs
        """
        self.d.press.recent()
        self.d(text="RsBalls").click()

    def check_logcat(self):
        """ check the key word in logcat
        """
        cmd = "logcat -d > /tmp/RsBenchmark.log"
        g_common_obj.adb_cmd_common(cmd)
        assert not os.system("cat /tmp/RsBenchmark.log|grep 'Compute Device: GPU'"),\
            "The RsBenchmark not triggered well"

    @staticmethod
    def clera_logcat():
        """ Clera log
        """
        cmd = "logcat -d -c"
        g_common_obj.adb_cmd_common(cmd)
        print "[clear log]"

    def launch_perftest_am(self):
        """ Launch perftest via adb am command
        """
        print "Launch perftest by adb am"
        g_common_obj.launch_app_am("com.android.perftest",
                                   "com.android.perftest.RsBench")
        self._locator.wait_exist(self._locator.performance_tests)

    @staticmethod
    def stop_perftest_am():
        """ Stop perftest via adb am command
        """
        print "Stop perftest by adb am"
        g_common_obj.stop_app_am("com.android.perftest")

    def launch_hellocompute_am(self):
        """ Launch hellocompute via adb am command
        """
        print "Launch hellocompute by adb am"
        g_common_obj.launch_app_am("com.example.android.rs.hellocompute",
                                   "com.example.android.rs.hellocompute.HelloCompute")
        self._locator.wait_exist(self._locator.performance_tests)

    @staticmethod
    def stop_hellocompute_am():
        """ Stop hellocompute via adb am command
        """
        print "Stop hellocompute by adb am"
        g_common_obj.stop_app_am("com.example.android.rs.hellocompute")

    def swith_to_photos_and_back(self):
        """ Swith back to hellocompute apk
        """
        self.launch_photos_am()
        self.d.press.recent()
        self.d(text="RsHelloCompute").click()

    def launch_miscsamples_states_am(self):
        """ Launch miscsamples via adb am command
        """
        print "Launch miscsamples by adb am"
        g_common_obj.launch_app_am("com.example.android.rs.miscsamples",
                                   "com.example.android.rs.miscsamples.RsRenderStates")
        self._locator.wait_exist(self._locator.performance_tests)

    @staticmethod
    def stop_miscsamples_states_am():
        """ Stop miscsamples via adb am command
        """
        print "Stop miscsamples by adb am"
        g_common_obj.stop_app_am("com.example.android.rs.miscsamples")

    def press_view_10times(self):
        """ Press view sometimes
        """
        for _ in range(0, 10):
            self.d(className="android.view.View").click()

    def launch_miscsamples_list_am(self):
        """ Launch miscsamples via adb am command
        """
        print "Launch miscsamples by adb am"
        g_common_obj.launch_app_am("com.example.android.rs.miscsamples",
                                   "com.example.android.rs.miscsamples.RsList")
        self._locator.wait_exist(self._locator.performance_tests)

    @staticmethod
    def stop_miscsamples_list_am():
        """ Stop miscsamples via adb am command
        """
        print "Stop miscsamples by adb am"
        g_common_obj.stop_app_am("com.example.android.rs.miscsamples")

    def scroll_to_bottom(self):
        """ Scroll to the bottom of the list
        """
        x = self.d.info["displayWidth"]
        y = self.d.info["displayHeight"]
        for _ in range(0, 5):
            self.d.swipe(x / 2, y / 2, x / 2, 0)

    def launch_glsurfaceview_am(self):
        """ Launch glsurfaceview via adb am command
        """
        print "Launch glsurfaceview by adb am"
        g_common_obj.launch_app_am("com.example.android.basicglsurfaceview",
                                   "com.example.android.basicglsurfaceview.BasicGLSurfaceViewActivity")
        self._locator.wait_exist(self._locator.performance_tests)

    @staticmethod
    def stop_glsurfaceview_am():
        """ Stop glsurfaceview via adb am command
        """
        print "Stop glsurfaceview by adb am"
        g_common_obj.stop_app_am("com.example.android.basicglsurfaceview")

    def launch_fountainfbo_am(self):
        """ Launch FountainFbo via adb am command
        """
        print "Launch FountainFbo by adb am"
        g_common_obj.launch_app_am("com.example.android.rs.fountainfbo",
                                   "com.example.android.rs.fountainfbo.FountainFbo")
        self._locator.wait_exist(self._locator.performance_tests)

    @staticmethod
    def stop_fountainfbo_am():
        """ Stop FountainFbo via adb am command
        """
        print "Stop FountainFbo by adb am"
        g_common_obj.stop_app_am("com.example.android.rs.fountainfbo")
