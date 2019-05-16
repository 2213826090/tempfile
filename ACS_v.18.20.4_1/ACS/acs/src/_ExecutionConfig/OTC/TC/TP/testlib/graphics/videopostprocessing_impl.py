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
@since: 03/17/2015
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


class VideoPPImpl:

    '''
    classdocs
    '''
    pkg_name = "com.androvid"
    activity_name = "com.androvid.videokit.HomeActivity"

    def __init__(self):
        self.d = g_common_obj.get_device()
        self._locator = Locator(self.d)

    def init_environment(self):
        """ Init the test environment
        """
        config = TestConfig()
        cfg_file = 'tests.tablet.artifactory.conf'
        cfg_arti = config.read(cfg_file, 'artifactory')
        config_handle = ConfigHandle()
        cfg_arti["location"] = config_handle.read_configuration('artifactory', 'location', '/etc/oat/', 'sys.conf')
        cfg = config.read(cfg_file, 'content_androidvid')
        self.arti = Artifactory(cfg_arti.get('location'))
        apk_name = cfg.get("name")
        file_path = self.arti.get(apk_name)
        print "%s" % file_path
        result = config_handle.check_apps(self.pkg_name)
        if result == 0:
            g_common_obj.adb_cmd_common('install ' + file_path)
        video_name = cfg.get("video_arti")
        video_path = self.arti.get(video_name)
        g_common_obj.adb_cmd_common('push ' + video_path + ' /sdcard/')
        cmd = "am broadcast -a \
        android.intent.action.MEDIA_MOUNTED -d file:///sdcard/"
        g_common_obj.adb_cmd(cmd)
        self.video = cfg.get("filename")
        bench_files = config.read(cfg_file, 'gfxbench31_files')
        busybox_path = self.arti.get(bench_files.get("busybox"))
        busybox_tar_path = bench_files.get("busybox_path")
        print g_common_obj.push_file(busybox_path, busybox_tar_path)
        print g_common_obj.adb_cmd_capture_msg("chmod 777 " + busybox_tar_path)
        self.md5_id = config.read(cfg_file, 'md5_id')
        self.check_path = self.md5_id.get('check_path')

    def launch_androvid_am(self):
        """ Launch androvid via adb am command
        """
        print "Launch androvid by adb am"
        g_common_obj.launch_app_am(
            VideoPPImpl.pkg_name, VideoPPImpl.activity_name)
        self._locator.wait_exist(self._locator.performance_tests)
        if self.d(text="OK").exists:
            self.d(text="OK").click()
            time.sleep(2)

    @staticmethod
    def stop_androvid_am():
        """ Stop androvid via adb am command
        """
        print "Stop androvid by adb am"
        g_common_obj.stop_app_am(VideoPPImpl.pkg_name)

    @staticmethod
    def uninstall_app():
        """ uninstall the androvid
        """
        print "Uninstall the androvid"
        cmd = 'uninstall %s' % VideoPPImpl.pkg_name
        g_common_obj.adb_cmd_common(cmd)

    def launch_photos_am(self):
        """ Launch Photos via adb am command
        """
        print "Launch Photos by adb am"
        g_common_obj.launch_app_am(
            "com.google.android.apps.plus", "com.google.android.apps.photos.phone.PhotosHomeActivity")
        time.sleep(2)
        if self.d(text="Later").exists:
            self.d(text="Later").click()
            time.sleep(2)
        self._locator.wait_exist(self._locator.performance_tests)
        time.sleep(5)
        if self.d(text="No thanks").exists:
            self.d(text="No thanks").click()
            time.sleep(5)
        if self.d(text="Setting up...").exists:
            time.sleep(30)

    @staticmethod
    def stop_photos_am():
        """ Stop Photos via adb am command
        """
        print "Stop Photos by adb am"
        g_common_obj.stop_app_am("com.google.android.apps.plus")

    def delete_video(self):
        """ delete the video pushed
        """
        print "delete the video pushed"
        cmd = "rm -rf /sdcard/%s" % self.video
        g_common_obj.adb_cmd(cmd)
        cmd = "rm -rf /sdcard/DCIM/AndroVid/"
        g_common_obj.adb_cmd(cmd)
        cmd = "am broadcast -a \
        android.intent.action.MEDIA_MOUNTED -d file:///sdcard/"
        g_common_obj.adb_cmd(cmd)

    def save_video_as_720p(self):
        """ Modifie the resolution as 720P
        """
        self.d(className="android.widget.FrameLayout", index=0
               ).child(resourceId="com.androvid:id/home_scroll_menu_item_icon"
                       ).click()
        time.sleep(2)
        if self.d(className="android.widget.ImageButton").exists:
            self.d(className="android.widget.ImageButton").click()
        time.sleep(2)
        self.d(text="Choose Video File").click()
        time.sleep(2)
        self.d(resourceId="com.androvid:id/photo_frame_photo").click()
        time.sleep(2)
        if self.d(resourceId="com.androvid:id/video_effects_spinner_progress").exists:
            self.d.press.back()
            time.sleep(2)
            self.d(className="android.widget.FrameLayout", index=0
                   ).child(resourceId="com.androvid:id/home_scroll_menu_item_icon"
                           ).click()
            time.sleep(2)
            if self.d(className="android.widget.ImageButton").exists:
                self.d(className="android.widget.ImageButton").click()
            time.sleep(2)
            self.d(text="Choose Video File").click()
            time.sleep(2)
            self.d(resourceId="com.androvid:id/photo_frame_photo").click()
            time.sleep(2)
        self.d(resourceId="com.androvid:id/option_transcode").click()
        time.sleep(2)
        self.d(resourceId="android:id/text1").click()
        self.d(text="720p").click()
        self.d(resourceId="com.androvid:id/option_transcode").click()
        time.sleep(3)
        self.d(text="Continue").click()
        if self.d(text="Continue").exists:
            self.d(text="Continue").click()
        time.sleep(240)
        self.d(resourceId="com.androvid:id/ringtone_save_button").click()

    def save_video_as_sepia(self):
        """ Set Video color effect as Sepia
        """
        self.d(className="android.widget.FrameLayout", index=5
               ).child(resourceId="com.androvid:id/home_scroll_menu_item_icon"
                       ).click()
        time.sleep(2)
        if self.d(className="android.widget.ImageButton").exists:
            self.d(className="android.widget.ImageButton").click()
        time.sleep(2)
        self.d(text="Choose Video File").click()
        time.sleep(2)
        self.d(resourceId="com.androvid:id/photo_frame_photo").click()
        time.sleep(2)
        if self.d(resourceId="com.androvid:id/video_effects_spinner_progress").exists:
            self.d.press.back()
            time.sleep(2)
            self.d(className="android.widget.FrameLayout", index=5
                   ).child(resourceId="com.androvid:id/home_scroll_menu_item_icon"
                           ).click()
            time.sleep(2)
            if self.d(className="android.widget.ImageButton").exists:
                self.d(className="android.widget.ImageButton").click()
            time.sleep(2)
            self.d(text="Choose Video File").click()
            time.sleep(2)
            self.d(resourceId="com.androvid:id/photo_frame_photo").click()
            time.sleep(2)
        x = self.d.info["displayWidth"]
        y = self.d.info["displayHeight"]
        if x > y:
            self.d(scrollable=True).scroll.vert.to(text="Sepia")
        else:
            self.d(scrollable=True).scroll.horiz.to(text="Sepia")
        time.sleep(2)
        self.d(text="Sepia").up(resourceId="com.androvid:id/effect_galery_icon"
                                ).click()
        self.d(description="Apply").click()
        time.sleep(180)
        self.d(
            resourceId="com.androvid:id/ringtone_name_edit").set_text("video")
        self.d(resourceId="com.androvid:id/ringtone_save_button").click()

    def save_video_as_vintage(self):
        """ Set Video color effect as Vintage
        """
        self.d(className="android.widget.FrameLayout", index=5
               ).child(resourceId="com.androvid:id/home_scroll_menu_item_icon"
                       ).click()
        time.sleep(2)
        if self.d(className="android.widget.ImageButton").exists:
            self.d(className="android.widget.ImageButton").click()
        time.sleep(2)
        self.d(text="Choose Video File").click()
        time.sleep(2)
        self.d(resourceId="com.androvid:id/photo_frame_photo").click()
        time.sleep(2)
        if self.d(resourceId="com.androvid:id/video_effects_spinner_progress").exists:
            self.d.press.back()
            time.sleep(2)
            self.d(className="android.widget.FrameLayout", index=5
                   ).child(resourceId="com.androvid:id/home_scroll_menu_item_icon"
                           ).click()
            time.sleep(2)
            if self.d(className="android.widget.ImageButton").exists:
                self.d(className="android.widget.ImageButton").click()
            time.sleep(2)
            self.d(text="Choose Video File").click()
            time.sleep(2)
            self.d(resourceId="com.androvid:id/photo_frame_photo").click()
            time.sleep(2)
        x = self.d.info["displayWidth"]
        y = self.d.info["displayHeight"]
        if x > y:
            self.d(scrollable=True).scroll.vert.to(text="Vintage")
        else:
            self.d(scrollable=True).scroll.horiz.to(text="Vintage")
        time.sleep(2)
        self.d(text="Vintage").up(
            resourceId="com.androvid:id/effect_galery_icon"
        ).click()
        self.d(description="Apply").click()
        time.sleep(180)
        self.d(
            resourceId="com.androvid:id/ringtone_name_edit").set_text("video")
        self.d(resourceId="com.androvid:id/ringtone_save_button").click()

    def save_video_as_original(self):
        """ Set Video color effect as Original
        """
        self.d(className="android.widget.FrameLayout", index=5
               ).child(resourceId="com.androvid:id/home_scroll_menu_item_icon"
                       ).click()
        time.sleep(2)
        if self.d(className="android.widget.ImageButton").exists:
            self.d(className="android.widget.ImageButton").click()
        time.sleep(2)
        self.d(text="Choose Video File").click()
        time.sleep(2)
        self.d(resourceId="com.androvid:id/photo_frame_photo").click()
        time.sleep(3)
        if self.d(resourceId="com.androvid:id/video_effects_spinner_progress").exists:
            self.d.press.back()
            time.sleep(2)
            self.d(className="android.widget.FrameLayout", index=5
                   ).child(resourceId="com.androvid:id/home_scroll_menu_item_icon"
                           ).click()
            time.sleep(2)
            if self.d(className="android.widget.ImageButton").exists:
                self.d(className="android.widget.ImageButton").click()
            time.sleep(2)
            self.d(text="Choose Video File").click()
            time.sleep(2)
            self.d(resourceId="com.androvid:id/photo_frame_photo").click()
            time.sleep(3)
        self.d(text="Mirror").up(resourceId="com.androvid:id/effect_galery_icon"
                                 ).click()
        self.d(description="Apply").click()
        time.sleep(5)
        self.d(text="Cancel").click()
        time.sleep(2)
        self.d(text="Original").up(resourceId="com.androvid:id/effect_galery_icon"
                                   ).click()
        self.d(description="Apply").click()
        cmd = "du /sdcard/DCIM/AndroVid/video/ |awk '{print $1}'"
        size = g_common_obj.adb_cmd_capture_msg(cmd)
        assert int(size) == 8, "No Warning!"

    def save_video_as_1080p(self):
        """ Modifie the resolution as 1080P
        """
        self.d(className="android.widget.FrameLayout", index=0
               ).child(resourceId="com.androvid:id/home_scroll_menu_item_icon"
                       ).click()
        time.sleep(2)
        if self.d(className="android.widget.ImageButton").exists:
            self.d(className="android.widget.ImageButton").click()
        time.sleep(2)
        self.d(text="Choose Video File").click()
        time.sleep(2)
        self.d(resourceId="com.androvid:id/photo_frame_photo").click()
        time.sleep(2)
        if self.d(resourceId="com.androvid:id/video_effects_spinner_progress").exists:
            self.d.press.back()
            time.sleep(2)
            self.d(className="android.widget.FrameLayout", index=0
                   ).child(resourceId="com.androvid:id/home_scroll_menu_item_icon"
                           ).click()
            time.sleep(2)
            if self.d(className="android.widget.ImageButton").exists:
                self.d(className="android.widget.ImageButton").click()
            time.sleep(2)
            self.d(text="Choose Video File").click()
            time.sleep(2)
            self.d(resourceId="com.androvid:id/photo_frame_photo").click()
            time.sleep(2)
        self.d(resourceId="com.androvid:id/option_transcode").click()
        time.sleep(2)
        self.d(resourceId="android:id/text1").click()
        self.d(text="1080p").click()
        self.d(resourceId="com.androvid:id/option_transcode").click()
        time.sleep(3)
        self.d(text="Continue").click()
        if self.d(text="Continue").exists:
            self.d(text="Continue").click()
        time.sleep(500)
        self.d(resourceId="com.androvid:id/ringtone_save_button").click()

    def save_video_as_160p(self):
        """ Modifie the resolution as 160P
        """
        self.d(className="android.widget.FrameLayout", index=0
               ).child(resourceId="com.androvid:id/home_scroll_menu_item_icon"
                       ).click()
        time.sleep(2)
        if self.d(className="android.widget.ImageButton").exists:
            self.d(className="android.widget.ImageButton").click()
        time.sleep(2)
        self.d(text="Choose Video File").click()
        time.sleep(2)
        self.d(resourceId="com.androvid:id/photo_frame_photo").click()
        time.sleep(2)
        if self.d(resourceId="com.androvid:id/video_effects_spinner_progress").exists:
            self.d.press.back()
            time.sleep(2)
            self.d(className="android.widget.FrameLayout", index=0
                   ).child(resourceId="com.androvid:id/home_scroll_menu_item_icon"
                           ).click()
            time.sleep(2)
            if self.d(className="android.widget.ImageButton").exists:
                self.d(className="android.widget.ImageButton").click()
            time.sleep(2)
            self.d(text="Choose Video File").click()
            time.sleep(2)
            self.d(resourceId="com.androvid:id/photo_frame_photo").click()
            time.sleep(2)
        self.d(resourceId="com.androvid:id/option_transcode").click()
        time.sleep(2)
        self.d(resourceId="android:id/text1").click()
        self.d(text="160p").click()
        self.d(resourceId="com.androvid:id/option_transcode").click()
        time.sleep(3)
        self.d(text="Continue").click()
        if self.d(text="Continue").exists:
            self.d(text="Continue").click()
        time.sleep(90)
        self.d(resourceId="com.androvid:id/ringtone_save_button").click()

    def save_video_as_rotation(self):
        """ Modifie the resolution as rotation
        """
        self.d(resourceId="com.androvid:id/home_latest_video1").click()
        time.sleep(2)
        if self.d(resourceId="com.androvid:id/video_effects_spinner_progress").exists:
            self.d.press.back()
            time.sleep(2)
            self.d(resourceId="com.androvid:id/home_latest_video1").click()
            time.sleep(2)
        self.d(
            resourceId="com.androvid:id/video_player_menu_videoview").click()
        self.d(resourceId="com.androvid:id/scrolling_menu_bar").swipe.left()
        self.d(text="Rotate").click()
        self.d(textContains="CCW").click()
        self.d(text="True Rotation (Encode Video)").click()
        self.d(text="Continue").click()
        time.sleep(60)
        self.d(
            resourceId="com.androvid:id/ringtone_name_edit").set_text("video")
        self.d(resourceId="com.androvid:id/ringtone_save_button").click()
        self.d(resourceId="com.androvid:id/overlayPlayButton").click()
        time.sleep(10)
        assert self.d(resourceId="com.androvid:id/video_player_menu_videoview"
                      ).exists, "Video didn't playing"

    def save_video_as_mirror_rotation(self):
        """ Set Video color effect as mirror and rotation
        """
        self.d(className="android.widget.FrameLayout", index=5
               ).child(resourceId="com.androvid:id/home_scroll_menu_item_icon"
                       ).click()
        time.sleep(2)
        if self.d(className="android.widget.ImageButton").exists:
            self.d(className="android.widget.ImageButton").click()
        time.sleep(2)
        self.d(text="Choose Video File").click()
        time.sleep(2)
        self.d(resourceId="com.androvid:id/photo_frame_photo").click()
        time.sleep(2)
        if self.d(resourceId="com.androvid:id/video_effects_spinner_progress").exists:
            self.d.press.back()
            time.sleep(2)
            self.d(className="android.widget.FrameLayout", index=5
                   ).child(resourceId="com.androvid:id/home_scroll_menu_item_icon"
                           ).click()
            time.sleep(2)
            if self.d(className="android.widget.ImageButton").exists:
                self.d(className="android.widget.ImageButton").click()
            time.sleep(2)
            self.d(text="Choose Video File").click()
            time.sleep(2)
            self.d(resourceId="com.androvid:id/photo_frame_photo").click()
            time.sleep(2)
        self.d(text="Mirror").up(resourceId="com.androvid:id/effect_galery_icon"
                                 ).click()
        self.d(description="Apply").click()
        time.sleep(120)
        garbage = self.d(resourceId="com.androvid:id/ringtone_name_edit").text
        self.d(resourceId="com.androvid:id/ringtone_save_button").click()
        assert self.d(textContains="Saved Successfully"
                      ).exists, "Save video as Mirror not success"
        self.d.press.menu()
        self.d(text="Rotate").click()
        self.d(textContains="CCW").click()
        self.d(text="True Rotation (Encode Video)").click()
        self.d(text="Continue").click()
        time.sleep(60)
        self.d(
            resourceId="com.androvid:id/ringtone_name_edit").set_text("video")
        self.d(resourceId="com.androvid:id/ringtone_save_button").click()
        cmd = "rm -rf /sdcard/DCIM/AndroVid/video/%s.mp4" % garbage
        g_common_obj.adb_cmd(cmd)

    def diff_md5(self, id):
        """ diff the video's md5
        """
        self.id = self.arti.get(self.md5_id.get(id))
        g_common_obj.adb_cmd_common('push ' + self.id + ' ' + self.check_path)
        cmd = '"cd /sdcard/DCIM/AndroVid/video/;/data/busybox md5sum -c %s|grep OK"' % id
        assert g_common_obj.adb_cmd(cmd) == 0, "The video not match"
