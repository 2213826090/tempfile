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
@since: 03/02/2015
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
        self._device = device

    @property
    def wait_exist(self):
        """ wait until the ui object exist """
        def _wait(uiobj, timeout=5):
            return uiobj.wait("exists", timeout * 500)
        return _wait

    @property
    def performance_tests(self):
        return self._device(text="Performance Tests")

class ChromeCastImpl:
    '''
    classdocs
    '''
    pkg_name = "com.google.android.apps.chromecast.app"
    activity_name = "com.google.android.apps.chromecast.app.DiscoveryActivity"


    def __init__(self):
        self._device = g_common_obj.get_device()
        self._locator = Locator(self._device)

    def set_environment(self):
        """ init the test environment
        """
        config = TestConfig()
        cfg_file = 'tests.tablet.chromecast.conf'
        cfg_arti = config.read(cfg_file, 'artifactory')
        config_handle = ConfigHandle()
        cfg_arti["location"] = config_handle.read_configuration('artifactory', 'location', '/etc/oat/', 'sys.conf')
        arti = Artifactory(cfg_arti.get('location'))
        cfg_apk = config.read(cfg_file, 'chromecast_apk')
        apk_name = cfg_apk.get("name")
        file_path = arti.get(apk_name)
        g_common_obj.adb_cmd_common('install ' + file_path)

    def connect_chromecast(self):
        """ connect to the chromecast
        """
        self.launch_settings_am()
        self._device(text="Display").click()
        time.sleep(1)
        self._device(text="Cast").click()
        time.sleep(10)
        if self._device(text="No nearby devices were found.").exists:
            time.sleep(30)
        assert self._device(className="android.widget.RelativeLayout"
            ).exists , "No nearby chromecast adapter"
        self._device(className="android.widget.RelativeLayout"
            ).click()
        time.sleep(5)
        self.stop_settings_am()

    def disconnect_chromecast(self):
        """ disconnect the chromecast
        """
        self.launch_settings_am()
        self._device(text="Display").click()
        time.sleep(1)
        self._device(text="Cast").click()
        time.sleep(5)
        if self._device(text="Connected"):
            self._device(className="android.widget.RelativeLayout"
            ).click()
            time.sleep(2)
            self._device(resourceId="android:id/media_route_disconnect_button").click()
            time.sleep(2)
        self.stop_settings_am()

    def launch_settings_am(self):
        """ Launch Settings via adb am command
        """
        print "Launch Settings by adb am"
        g_common_obj.launch_app_am(
            "com.android.settings", "com.android.settings.Settings")
        time.sleep(3)
        self._locator.wait_exist(self._locator.performance_tests)

    @staticmethod
    def stop_settings_am():
        """ Stop Settings via adb am command
        """
        print "Stop Settings by adb am"
        g_common_obj.stop_app_am("com.android.settings")

    def launch_chromecast_am(self):
        """ Launch ChromeCast via adb am command
        """
        print "Launch ChromeCast by adb am"
        g_common_obj.launch_app_am(\
            ChromeCastImpl.pkg_name, ChromeCastImpl.activity_name)
        self._locator.wait_exist(self._locator.performance_tests)
        time.sleep(5)
        if self._device(text="OK, got it").exists:
            self._device(text="OK, got it").click()
        if self._device(text="No Thanks").exists:
            self._device(text="No Thanks").click()

    @staticmethod
    def stop_chromecast_am():
        """ Stop ChromeCast via adb am command
        """
        print "Stop ChromeCast by adb am"
        g_common_obj.stop_app_am(ChromeCastImpl.pkg_name)

    def install_apidemos(self):
        """ lunch apidemos
        """
        config = TestConfig()
        cfg_file = 'tests.tablet.chromecast.conf'
        cfg_arti = config.read(cfg_file, 'artifactory')
        config_handle = ConfigHandle()
        cfg_arti["location"] = config_handle.read_configuration('artifactory', 'location', '/etc/oat/', 'sys.conf')
        arti = Artifactory(cfg_arti.get('location'))
        cfg_apk = config.read(cfg_file, 'api_demos')
        apk_name = cfg_apk.get("name")
        file_path = arti.get(apk_name)
        g_common_obj.adb_cmd_common('install ' + file_path)

    def launch_apidemos_am(self):
        """ Launch Apidemos via adb am command
        """
        print "Launch Apidemos by adb am"
        g_common_obj.launch_app_am("com.example.android.apis"
            , "com.example.android.apis.ApiDemos")
        self._locator.wait_exist(self._locator.performance_tests)
        time.sleep(5)

    @staticmethod
    def stop_apidemos_am():
        """ Stop Apidemos via adb am command
        """
        print "Stop Apidemos by adb am"
        g_common_obj.stop_app_am("com.example.android.apis")

    def run_medie_router(self):
        """ run the Presentation with Media Router
        """
        self._device(text="App").click()
        time.sleep(2)
        self._device(text="Activity").click()
        time.sleep(2)
        self._device(text="Presentation with Media Router").click()
        time.sleep(5)

    def suspend_and_resume(self):
        """ suspend and resume the DUT
        """
        self._device.press.power()
        time.sleep(5)
        self._device.press.power()
        time.sleep(2)

    def sleep_dut(self):
        """ sleep the DUT
        """
        self._device.press.power()
        time.sleep(5)

    def launch_youtube_am(self):
        """ Launch youtube via adb am command
        """
        print "Launch youtube by adb am"
        g_common_obj.launch_app_am("com.google.android.youtube"
            , "com.google.android.apps.youtube.app.WatchWhileActivity")
        self._locator.wait_exist(self._locator.performance_tests)
        time.sleep(5)
        if self._device(text="RetryRetry").exists:
            self._device(text="RetryRetry").click()
            time.sleep (5)

    @staticmethod
    def stop_youtube_am():
        """ Stop youtube via adb am command
        """
        print "Stop youtube by adb am"
        g_common_obj.stop_app_am("com.google.android.youtube")

    def play_video_in_youtube(self):
        """ play a video via ChromeCast
        """
        print "[Play a video via ChromeCast]"
        if self._device(text="Skip").exists:
            self._device(text="Skip").click()
        time.sleep(2)
        if self._device(text="OK").exists:
            self._device(text="OK").click()
        time.sleep(2)
        if self._device(text="OK").exists:
            self._device(text="OK").click()
        time.sleep(2)
        if self._device(text="OK").exists:
            self._device(text="OK").click()
        time.sleep(2)
        self._device(resourceId="com.google.android.youtube:id/feed_item"
            ).child(resourceId=
            "com.google.android.youtube:id/thumbnail").click()
        time.sleep(15)
        self._device(resourceId=
            "com.google.android.youtube:id/watch_player").click()
        self._device(resourceId=
            "com.google.android.youtube:id/watch_player").click()
        time.sleep(1)
        self._device(resourceId=
            "com.google.android.youtube:id/media_route_button").click()
        time.sleep(5)
        self._device(resourceId="android:id/text1").click()
        time.sleep(15)

    def add_three_video_in_queue(self):
        """ add three video in queue
        """
        print "[add three video in queue]"
        self._device(className="android.widget.FrameLayout", index=0
            ).child(resourceId="com.google.android.youtube:id/thumbnail").click()
        time.sleep(5)
        self._device(text="Add to Queue").click()
        time.sleep(5)
        self._device(className="android.widget.FrameLayout", index=1
            ).child(resourceId="com.google.android.youtube:id/thumbnail").click()
        time.sleep(5)
        self._device(text="Add to Queue").click()
        time.sleep(5)
        self._device(className="android.widget.FrameLayout", index=2
            ).child(resourceId="com.google.android.youtube:id/thumbnail").click()
        time.sleep(5)
        self._device(text="Add to Queue").click()
        time.sleep(5)
        self._device(text="TV Queue").click()
        time.sleep(3)
        assert self._device(textMatches=".* videos"
            ).exists , "Videos not added in the queue"

    def init_wps(self):
        """ lunch wps kingsoft office
        """
        config = TestConfig()
        cfg_file = 'tests.tablet.chromecast.conf'
        cfg_arti = config.read(cfg_file, 'artifactory')
        config_handle = ConfigHandle()
        cfg_arti["location"] = config_handle.read_configuration('artifactory', 'location', '/etc/oat/', 'sys.conf')
        arti = Artifactory(cfg_arti.get('location'))
        cfg_apk = config.read(cfg_file, 'wps')
        apk_name = cfg_apk.get("name")
        apk_path = arti.get(apk_name)
        g_common_obj.adb_cmd_common('install ' + apk_path)
        pdf_name = cfg_apk.get("pdf")
        pdf_path = arti.get(pdf_name)
        g_common_obj.adb_cmd_common('push ' + pdf_path + ' /sdcard/')

    def launch_wps_am(self):
        """ Launch wps via adb am command
        """
        print "Launch wps by adb am"
        g_common_obj.launch_app_am("cn.wps.moffice_eng"
            , "cn.wps.moffice.main.local.home.PadHomeActivity")
        time.sleep(3)
        if self._device(text="ALLOW"):
            self._device(text="ALLOW").click()

    @staticmethod
    def stop_wps_am():
        """ Stop wps via adb am command
        """
        print "Stop wps by adb am"
        g_common_obj.stop_app_am("cn.wps.moffice_eng")

    def view_pdf_with_wps(self):
        """ view pdf with wps
        """
        print "View pdf with WPS"
        self._device(text="Open").click()
        time.sleep(3)
        self._device(text="All Documents").click()
        time.sleep(3)
        self._device(text="test").click()
        time.sleep(4)

    @staticmethod
    def uninstall_wps():
        """ uninstall the WPS
        """
        print "Uninstall the WPS "
        cmd = 'uninstall %s' % "cn.wps.moffice_eng"
        g_common_obj.adb_cmd_common(cmd)

    def start_airplane_mode_and_open_wifi(self):
        """ start airplane mode and open wifi
        """
        self.launch_settings_am()
        self._device(text="More").click()
        time.sleep(2)
        self._device(text="Airplane mode").click()
        time.sleep(2)
        self._device.press.back()
        self._device(textMatches="Wi.*Fi").click()
        time.sleep(2)
        self._device(text="Off").click()
        time.sleep(5)
        self.stop_settings_am()

    def stop_airplane_mode (self):
        """ stop airplane mode
        """
        self.launch_settings_am()
        self._device(text="More").click()
        time.sleep(2)
        self._device(text="Airplane mode").click()
        time.sleep(2)
        self.stop_settings_am()

    def launch_camera_am(self):
        """ Launch Camera via adb am command
        """
        print "Launch Camera by adb am"
        g_common_obj.launch_app_am("com.android.camera2", "com.android.camera.CameraLauncher")
        self._locator.wait_exist(self._locator.performance_tests)
        time.sleep(5)
        if self._device(text="NEXT").exists:
            self._device(text="NEXT").click()
            time.sleep(5)

    @staticmethod
    def stop_camera_am():
        """ Stop Camera via adb am command
        """
        print "Stop Camera by adb am"
        g_common_obj.stop_app_am("com.android.camera2")

    def capture_a_video(self):
        """ capture a video with camera
        """
        print "Capture a video"
        x = self._device.info["displayWidth"]
        self._device.swipe(0, 0, x, 0)
        time.sleep(5)
        self._device(text="Video").click()
        time.sleep(5)
        self._device(resourceId="com.android.camera2:id/shutter_button").click()
        print "Recoding..."
        time.sleep(10)
        self._device(resourceId="com.android.camera2:id/shutter_button").click()
        print "Stop Recoding"
        time.sleep(2)

    def swith_camera_to_capture_mode(self):
        """ Swith back to capture
        """
        print "Swith back to capture"
        self.launch_camera_am()
        x = self._device.info["displayWidth"]
        self._device.swipe(0, 0, x, 0)
        time.sleep(5)
        self._device(text="Camera").click()
        time.sleep(2)
        self.stop_camera_am()

    def launch_photos_am(self):
        """ Launch Photos via adb am command
        """
        print "Launch Photos by adb am"
        g_common_obj.launch_app_am("com.google.android.apps.photos"
            , "com.google.android.apps.photos.home.HomeActivity")
        time.sleep(2)
        if self._device(text="Later").exists:
            self._device(text="Later").click()
            time.sleep(2)
        self._locator.wait_exist(self._locator.performance_tests)
        time.sleep(5)
        if self._device(text="No thanks").exists:
            self._device(text="No thanks").click()
            time.sleep(5)
        if self._device(text="ALLOW"):
            self._device(text="ALLOW").click.wait()
            time.sleep(4)
        if self._device(text="GET STARTED"):
            self._device(text="GET STARTED").click.wait()
            time.sleep(4)
        if self._device(text="DONE"):
            self._device(text="DONE").click.wait()
            time.sleep(4)
        if self._device(resourceId="com.google.android.apps.photos:id/next_button"):
            self._device(resourceId="com.google.android.apps.photos:id/next_button").click.wait()


    @staticmethod
    def stop_photos_am():
        """ Stop Photos via adb am command
        """
        print "Stop Photos by adb am"
        g_common_obj.stop_app_am("com.google.android.apps.plus")

    def play_video_in_photos(self):
        """ play video in photos
        """
        print "play video in photos"
        #self._device(resourceId="com.google.android.apps.plus:id/tile_row")\
        #.child(className="android.view.View").click()
        self._device(className="android.view.View", index=1).click.wait()
        time.sleep(10)
        #self._device(description="Play video").click()
        #time.sleep(10)

    def play_video_in_photos_direct(self, video_path):
        print "Play video directly in photos"
        cmd = "am start -a android.intent.action.VIEW -d file://%s \
            -t video/* \
            -n com.google.android.apps.photos/com.google.android.apps.photos.pager.HostPhotoPagerActivity" % video_path
        g_common_obj.adb_cmd(cmd)
        time.sleep(10)

    def delete_captured_video(self):
        """ delete video captured
        """
        print "delete the video captured"
        cmd = "rm -rf /sdcard/DCIM/Camera"
        g_common_obj.adb_cmd(cmd)
        cmd = "am broadcast -a android.intent.action.MEDIA_MOUNTED -d file:///sdcard/"
        g_common_obj.adb_cmd(cmd)

    def init_local_video(self):
        """ push video to DUT
        """
        config = TestConfig()
        cfg_file = 'tests.tablet.chromecast.conf'
        cfg_arti = config.read(cfg_file, 'artifactory')
        config_handle = ConfigHandle()
        cfg_arti["location"] = config_handle.read_configuration('artifactory', 'location', '/etc/oat/', 'sys.conf')
        arti = Artifactory(cfg_arti.get('location'))
        cfg_video = config.read(cfg_file, 'video')
        video_name = cfg_video.get("name")
        video_path = arti.get(video_name)
        g_common_obj.adb_cmd_common('push ' + video_path + ' /sdcard/Movies/')
        cmd = "am broadcast -a android.intent.action.MEDIA_MOUNTED -d file:///sdcard/"
        g_common_obj.adb_cmd(cmd)
        self.video = cfg_video.get("filename")
        return '/sdcard/Movies/' + video_name.strip().split("/")[-1]

    def delete_local_video(self):
        """ delete the video pushed
        """
        print "delete the video pushed"
        cmd = "rm -rf /sdcard/%s" % self.video
        g_common_obj.adb_cmd(cmd)
        cmd = "am broadcast -a android.intent.action.MEDIA_MOUNTED -d file:///sdcard/"
        g_common_obj.adb_cmd(cmd)

    def set_record_sd_mode(self):
        """ set video Recod with SD size
        """
        x = self._device.info["displayWidth"]
        self._device.swipe(0, 0, x, 0)
        time.sleep(5)
        self._device(description="Settings").click()
        time.sleep(2)
        self._device(text="Resolution & quality").click()
        time.sleep(2)
        self._device(text="Back camera video").click()
        time.sleep(2)
        self._device(text="SD 480p").click()
        time.sleep(2)
        self._device.press.back()
        time.sleep(2)
        self._device.press.back()
        time.sleep(2)

    def init_photos(self):
        """ push some pictures in DUT
        """
        config = TestConfig()
        cfg_file = 'tests.tablet.chromecast.conf'
        cfg_arti = config.read(cfg_file, 'artifactory')
        config_handle = ConfigHandle()
        cfg_arti["location"] = config_handle.read_configuration('artifactory', 'location', '/etc/oat/', 'sys.conf')
        arti = Artifactory(cfg_arti.get('location'))
        cfg_name = config.read(cfg_file, 'pictures')
        pictures_name = cfg_name.get("name")
        self.floder_name = cfg_name.get('floder')
        pictures_path = arti.get(pictures_name)
        #print os.system("cd " + os.path.dirname(pictures_path) + ";tar xzf " + pictures_path)
        os.system("cd " + os.path.dirname(pictures_path) + ";tar xzf " + pictures_path)
        unzip_photos = pictures_path.replace(".tar.gz", "/")
        pic_names_list = os.popen("ls " + unzip_photos).read().strip().split("\n")
        i = 0
        for pic_name in pic_names_list:
            pic_names_list[i] = "file:///sdcard/"+ self.floder_name + '/' + pic_name
            i = i + 1
        print pic_names_list
        cmd = "mkdir -p %s" % self.floder_name
        g_common_obj.adb_cmd_common(cmd)
        cmd = "push %s /sdcard/%s" % (unzip_photos, self.floder_name)
        g_common_obj.adb_cmd_common(cmd)
        cmd = "am broadcast -a android.intent.action.MEDIA_MOUNTED -d file:///sdcard/"
        g_common_obj.adb_cmd(cmd)
        return pic_names_list

    def delete_photos(self):
        """ Delete the photos tested
        """
        print "Delete the photos"
        cmd = "rm -rf /sdcard/%s " % self.floder_name
        g_common_obj.adb_cmd(cmd)
        cmd = "am broadcast -a android.intent.action.MEDIA_MOUNTED -d file:///sdcard/"
        g_common_obj.adb_cmd(cmd)

    def view_photos(self):
        """ view photos with slide
        """
        print "view photos"
        self._device(resourceId="com.google.android.apps.plus:id/tile_row")\
        .child(className="android.view.View").click()
        time.sleep(2)
        for _ in range(1, 6):
            x = self._device.info["displayWidth"]
            y = self._device.info["displayHeight"]
            self._device.swipe(x * 2 / 3, y / 2, 0, y / 2)
            time.sleep(3)

    def view_photos_new(self, pic_name):
        """View photos directly"""
        cmd = "am start -a android.intent.action.VIEW -d %s -t image/*" % pic_name
        g_common_obj.adb_cmd(cmd)
        if self._device(text = "Photos"):
            self._device(text="Photos").click.wait()
        if self._device(text = "ALWAYS"):
            self._device(text="ALWAYS").click.wait()

    def set_live_wallpaper(self):
        """ set live wallpaper with blackhole
        """

        print "Set live wallpaper"
        self.launch_settings_am()
        self._device(text="Display").click()
        time.sleep(2)
        self._device(text="Wallpaper").click()
        time.sleep(2)
        self._device(text="Live Wallpapers").click()
        time.sleep(2)
        if not self._device(text="Black Hole"):
            config = TestConfig().read("tests.castscreen.cases.conf", "test_rotation_livewallpaper")
            config['artifactory_location'] = ConfigHandle().read_configuration('artifactory', 'location', '/etc/oat', 'sys.conf')
            arti = Artifactory(config.get('artifactory_location'))
            local_apk = arti.get(config.get("black_hole_wallpaper_apk"))
            import pdb
            pdb.set_trace()
            g_common_obj.adb_cmd_common('install ' + local_apk, time_out=600)
        self._device(text="Black Hole").click()
        time.sleep(2)
        self._device(text="Set wallpaper").click()
        time.sleep(2)
        self.stop_settings_am()

    def rotate_clockwise_anticlockwise(self):
        """ rotate DUT clockwise and anticlockwise
        """
        print "Rotate DUT clockwise and anticlockwise"
        self._device.orientation = "l"
        time.sleep(2)
        self._device.orientation = "n"
        time.sleep(2)
        self._device.orientation = "r"
        time.sleep(2)
        self._device.orientation = "n"
        time.sleep(2)

    def play_long_video_background_with_youtube(self):
        """ play long video background with youtube
        """
        print "Play a long video with youtube"
        self._device(description="Search").click()
        time.sleep(2)
        cmd = "input text 20mins"
        g_common_obj.adb_cmd(cmd)
        time.sleep(2)
        self._device.press.enter()
        time.sleep(10)
        if self._device(
            resourceId="com.google.android.youtube:id/load_progress").exists:
            time.sleep(30)
        self._device(resourceId="com.google.android.youtube:id/video_info_view").click()
        time.sleep(10)
        self._device(resourceId=
            "com.google.android.youtube:id/watch_player").click()
        self._device(resourceId=
            "com.google.android.youtube:id/watch_player").click()
        time.sleep(1)
        self._device(resourceId=
            "com.google.android.youtube:id/media_route_button").click()
        time.sleep(5)
        self._device(resourceId="android:id/text1").click()
        time.sleep(20)
        self._device.press.home()

    def launch_some_app_wait_5mins(self):
        """ launch some app and wait 5 mins
        """
        self.launch_settings_am()
        self._device.press.home()
        self.launch_camera_am()
        self._device.press.home()
        self.launch_photos_am()
        self._device.press.home()
        # time.sleep(20)
        time.sleep(300)

    def back_youtube_check_long_video(self):
        """ back to youtube and check the long video status
        """
        self._device.press.recent()
        time.sleep(2)
        self._device(text="YouTube").click()
        time.sleep(10)
        assert self._device(description="Pause video").exists, "The video not playing"
