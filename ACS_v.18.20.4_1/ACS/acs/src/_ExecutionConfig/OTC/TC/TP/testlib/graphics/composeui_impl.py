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
@summary: Class for ComposeUi operation
@since: 03/31/2015
@author: Yingjun Jin
'''


import time
import os
import tempfile
import testlib.graphics.decorator as decorator
from testlib.util.common import g_common_obj
from testlib.util.config import TestConfig
from testlib.util.repo import Artifactory
from testlib.graphics.tools import ConfigHandle
from testlib.graphics.common import qrcode_obj
from testlib.graphics.compare_pic_impl import compare_pic
from testlib.graphics.common import id_generator, remove_temp_file
from testlib.graphics.photos_impl import get_photo_implement
from testlib.common.base import getTmpDir


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


class ComposeUiImpl:

    '''
    classdocs
    '''

    def __init__(self):
        self.tmpdir = getTmpDir()
        self.d = g_common_obj.get_device()
        self._locator = Locator(self.d)
        self.photos = get_photo_implement()

    def install_apk(self, apk_name):
        """ Install the apk from artifactory
        """
        config = TestConfig()
        cfg_file = 'tests.tablet.composeui.conf'
        cfg_arti = config.read(cfg_file, 'artifactory')
        config_handle = ConfigHandle()
        cfg_arti["location"] = config_handle.read_configuration('artifactory', 'location', '/etc/oat/', 'sys.conf')
        arti = Artifactory(cfg_arti.get('location'))
        cfg_apk = config.read(cfg_file, apk_name)
        apk_name = cfg_apk.get("name")
        file_path = arti.get(apk_name)
        g_common_obj.adb_cmd_common('install ' + file_path)

    def launch_settings_am(self):
        """ Launch Settings via adb am command
        """
        print "Launch Settings by adb am"
        g_common_obj.launch_app_am(
            "com.android.settings", "com.android.settings.Settings")
        self._locator.wait_exist(self._locator.performance_tests)

    @staticmethod
    def stop_settings_am():
        """ Stop Settings via adb am command
        """
        print "Stop Settings by adb am"
        g_common_obj.stop_app_am("com.android.settings")

    def Rotate_clockwise_5times(self):
        """ Rotate screen and capture screenshot
        """
        for _ in range(0, 2):
            self.d.orientation = "l"
            time.sleep(1)
            self.d.orientation = "r"
            time.sleep(1)
            self.d.orientation = "n"
            time.sleep(1)

    def play_video_command_mx(self, video_path):
        """
        Play video via command line
        """
        result = g_common_obj.adb_cmd_common("shell pm list package | grep %s" % ("com.mxtech.videoplayer.ad"))
        if result == 0:
            self.install_apk('Mxplayer')
        g_common_obj.adb_cmd("am start -a android.intent.action.VIEW -d file://%s -t video/* \
        -n com.mxtech.videoplayer.ad/com.mxtech.videoplayer.ad.ActivityScreen" % (video_path))
        if self.d(text="Start over").wait.exists(timeout=5000):
            self.d(text="Start over").click.wait()

    def mx_rotate_screen(self):
        """
        Rotate DUT screen during MX player video playback.
        """
        self.d.press.menu()
        self.d(text="Tools").click.wait()
        self.d(text="Settings").click.wait()
#         self.d(text="Player").click.wait()
#         self.d(text="Screen").click.wait()
#         self.d(resourceId="com.mxtech.videoplayer.ad:id/orientation").click.wait()
#         self.d(text="Auto rotation").click.wait()
#         self.d(text="OK").click.wait()
#         self.d.press.back()
        self.d.press.back()
        self.d.press.center()
        if self.d(resourceId="com.mxtech.videoplayer.ad:id/rotate_screen").exists:
            self.d(resourceId="com.mxtech.videoplayer.ad:id/rotate_screen").click()
        self.d(resourceId="com.mxtech.videoplayer.ad:id/playpause").click.wait()
        time.sleep(1)

    def launch_workload_am(self):
        """ Launch workload via adb am command
        """
        print "Launch workload by adb am"
        g_common_obj.launch_app_am(
            "com.intel.aws.workload.applaunch",
            "com.intel.aws.workload.applaunch.StartupWorkloadMain")
        self._locator.wait_exist(self._locator.performance_tests)
        if self.d(text="OK").exists:
            self.d(text="OK").click()

    @staticmethod
    def stop_workload_am():
        """ Stop workload via adb am command
        """
        print "Stop workload by adb am"
        g_common_obj.stop_app_am("com.intel.aws.workload.applaunch")

    def test_config_apps(self):
        """ test chrome photos and settings
        """
        self.d(text="Configure").click.wait()
        self.d(text="Chrome").click.wait()
        self.d(text="Photos").click.wait()
        self.d(text="Settings").click.wait()
        self.d(text="Done").click.wait()
        time.sleep(1)
        self.d(text="Start").click.wait()
        time.sleep(15)
        import re
        result = self.d(
            resourceId="com.intel.aws.workload.applaunch:id/history_result"
        ).text
        assert re.match(
            r'[\s\S]*START_SUCCESS[\s\S]*START_SUCCESS[\s\S]*START_SUCCESS', result), "The test failed"

    @staticmethod
    def uninstall_workload():
        """ uninstall the workload
        """
        print "Uninstall the workload"
        cmd = 'uninstall com.intel.aws.workload.applaunch'
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
        if self.d(text="No thanks").exists:
            self.d(text="No thanks").click()
            time.sleep(3)

    @staticmethod
    def stop_photos_am():
        """ Stop Photos via adb am command
        """
        print "Stop Photos by adb am"
        g_common_obj.stop_app_am("com.google.android.apps.plus")

    def init_local_video(self):
        """ push video to DUT
        """
        config = TestConfig()
        cfg_file = "tests.tablet.composeui.conf"
        # cfg_arti = config.read(cfg_file, 'artifactory')
        # config_handle = ConfigHandle()
        # cfg_arti["location"] = config_handle.read_configuration('artifactory', 'location', '/etc/oat/', 'sys.conf')
        # arti = Artifactory(cfg_arti.get('location'))
        cfg_video = config.read(cfg_file, 'video')
        # video_name = cfg_video.get("name")
        # video_path = arti.get(video_name)
        # g_common_obj.adb_cmd_common('push ' + video_path + ' /sdcard/Movies/')
        # cmd = "am broadcast -a android.intent.action.MEDIA_SCANNER_SCAN_FILE \
        #  -d file:///mnt/sdcard/Movies/%s" % video_name
        # g_common_obj.adb_cmd(cmd)
        self.photos.deploy_photo_content("Videos", "video_006", "/sdcard/Movies")
        self.video = cfg_video.get("filename")

    def delete_local_video(self):
        """ delete the video pushed
        """
        print "delete the video pushed"
        cmd = "rm -rf /sdcard/Movies/%s" % self.video
        g_common_obj.adb_cmd(cmd)
        cmd = "am broadcast -a android.intent.action.MEDIA_MOUNTED --ez read-only false -d file://storage/sdcard1"
        g_common_obj.adb_cmd(cmd)

    def play_video_and_slide_processbar(self):
        """ play video in photos
        """
        print "play video in photos"
        self.photos.play_video()
        time.sleep(3)
        self.photos.slide_video_processbar_forward()
        self.photos.slide_video_processbar_backward()

    @decorator.restart_uiautomator
    def launch_mxtech_am(self):
        """ Launch mxtech via adb am command
        """
        print "Launch mxtech by adb am"
        g_common_obj.launch_app_am(
            "com.mxtech.videoplayer.ad",
            "com.mxtech.videoplayer.ad.ActivityMediaList")
        self._locator.wait_exist(self._locator.performance_tests)
        time.sleep(3)
        for i in range(3):
            if self.d(textContains="update").wait.exists(timeout=3000)\
                    and self.d(text="CANCEL").wait.exists(timeout=3000):
                self.d(text="CANCEL").click.wait()
            if self.d(text="Hardware acceleration").wait.exists(timeout=3000)\
                    and self.d(text="OK").wait.exists(timeout=3000):
                self.d(text="OK").click.wait()
            if self.d(textMatches="CANCEL|Cancel").exists:
                self.d(textMatches="CANCEL|Cancel").click()

    @staticmethod
    def stop_mxtech_am():
        """ Stop mxtech via adb am command
        """
        print "Stop mxtech by adb am"
        g_common_obj.stop_app_am("com.mxtech.videoplayer.ad")

    @staticmethod
    def uninstall_mxtech():
        """ uninstall the mxtech
        """
        print "Uninstall the mxtech"
        cmd = 'uninstall com.mxtech.videoplayer.ad'
        g_common_obj.adb_cmd_common(cmd)

    def check_notification_during_playing(self):
        """ check notification during play video
        """
        self.d(descriptionContains="Refresh").click.wait(timeout=6000)
        time.sleep(2)
        self.d(textContains="Movies").click.wait()
        time.sleep(2)
        self.d.press("recent")
        time.sleep(2)
        self.d.press("back")
        time.sleep(2)
        if self.d(resourceId="com.mxtech.videoplayer.ad:id/thumb"):
            self.d(resourceId="com.mxtech.videoplayer.ad:id/thumb").click.wait()
        time.sleep(1)
        if self.d(text="Dark"):
            self.d(text="Dark").click.wait()
        self.d.click(400, 400)
        assert self.d(resourceId="com.mxtech.videoplayer.ad:id/playpause"), "movie is not playing"
        if self.d(text="Start over").exists:
            self.d(text="Start over").click()
        x = self.d.info["displayWidth"]
        y = self.d.info["displayHeight"]
        time.sleep(5)
        cmd = "dumpsys SurfaceFlinger | grep -E 'StatusBar| NavigationBar' | grep -Ev '\:|\[|\('"
        msg1 = g_common_obj.adb_cmd_capture_msg(repr(cmd))
        print msg1
        assert msg1 == "", "The notification or statusbar is popuped"
        for _ in range(0, 5):
            self.d.swipe(x / 2, 0, x / 2, y / 2)
            self.d.open.quick_settings()
            time.sleep(1)
            if self.d(resourceId="com.android.systemui:id/quick_settings_container").exists:
                break
        time.sleep(3)
        msg2 = g_common_obj.adb_cmd_capture_msg(repr(cmd))
        print msg2
        assert msg2 != "", "The notification not popup"
        self.d.orientation = "n"

    def check_scaling_rotation_during_playing(self):
        """ check Scaling and Rotation during play video
        """
        self.d(textContains="Movies").click()
        self.d(description="More options").click.wait()
        self.d(text="Settings").click.wait()
        self.d(text="Player").click.wait()
        self.d(text="Screen").click.wait()
        self.d(resourceId="com.mxtech.videoplayer.ad:id/orientation").click.wait()
        self.d(text="Auto rotation").click.wait()
        self.d(text="OK").click.wait()
        self.d.press.back()
        self.d.press.back()
        time.sleep(1)
        self.d(resourceId="com.mxtech.videoplayer.ad:id/thumb").click()
        if self.d(text="Start over").exists:
            self.d(text="Start over").click()
        time.sleep(2)
        orientation = self.d.info["displayRotation"]
        x = self.d.info["displayWidth"]
        y = self.d.info["displayHeight"]
        print "rotate to left"
        self.d.orientation = "l"
        time.sleep(1)
        g_common_obj.assert_exp_happens()
        print "rotate to right"
        self.d.orientation = "r"
        time.sleep(1)
        g_common_obj.assert_exp_happens()
        if x > y and orientation == 0:
            self.d.orientation = "r"
        elif x > y and orientation > 0:
            self.d.orientation = "n"
        self.d.freeze_rotation()
        time.sleep(6)

        self.d(resourceId="com.mxtech.videoplayer.ad:id/thumb").click()
        if self.d(text="Start over").exists:
            self.d(text="Start over").click()
        time.sleep(2)
        print "Zoom in"
        self.d().gesture((x / 2, y / 2), (x / 2, y / 2)).to((x / 4, y / 4), (x * 3 / 4, y * 3 / 4))
        g_common_obj.assert_exp_happens()
        print "Zoom out"
        self.d().gesture((x / 4, y / 4), (x * 3 / 4, y * 3 / 4)).to((x / 2, y / 2), (x / 2, y / 2))
        g_common_obj.assert_exp_happens()

    def play_video_and_pause_resume(self):
        """ play video in photos and pause resume
        """
        print "play video in photos"
        self.photos.play_video()
        self.photos.pause_play_video()
        time.sleep(3)
        self.photos.resume_play_video()
        g_common_obj.assert_exp_happens()

    def capture_screen_shot_with_png_and_check(self):
        """ play video in photos and pause resume
        """
        print "capture screen shot and check"
#         self.photos.open_a_picture()
#         time.sleep(2)
        cmd = "screencap -p /sdcard/Pictures/screen.png"
        g_common_obj.adb_cmd(cmd)
        cmd = "pull /sdcard/Pictures/screen.png /tmp/"
        g_common_obj.adb_cmd_common(cmd)
        ref = g_common_obj.adb_cmd("wm size |awk  '{print $3}'")
        cmd = "file /tmp/screen.png | grep '%s'" % ref
        assert os.system(cmd) == 0, "The screen shot Dimensinos not correct"

    def capture_screen_shot_during_video_playback(self):
        """ capture screen shot during play video
        """
        print "capture screen shot during play video"
#         self.photos.play_video()
#         time.sleep(5)
        cmd = "screencap -p /sdcard/Pictures/screen.png"
        assert g_common_obj.adb_cmd(
            cmd) == 0, "The screen shot not captured correct"

    def capture_screenshot_compare_screenshot(self):
        """ capture screen shot and compare with the screenshot whitch python take
        """
        time.sleep(2)
        self.d.screenshot("compare.png")
        self.capture_screen_shot(remote_path="/sdcard/Pictures/sample.png")
        time.sleep(2)
        path = os.getcwd()
        cmd = 'mv ' + path + '/compare.png ' + self.tmpdir + '/compare.png'
        print cmd
        os.system(cmd)
        g_common_obj.adb_cmd_common('pull ' + '/sdcard/Pictures/sample.png' + ' ' + self.tmpdir + '/sample.png')
        print self.tmpdir + "/sample.png"
        print self.tmpdir + "/compare.png"
        time.sleep(1)
        rms = compare_pic.compare_pic_with_redrawLhist(self.tmpdir + "/compare.png", self.tmpdir + "/sample.png")
        print "The Similarity value of 2 screenshot is %s" % rms
        assert rms <= 20, "The screenshot is diffrent with python screenshot"

    def capture_screen_shot(self, remote_path="/sdcard/Pictures/screen.png"):
        """ capture screen shot
        """
        cmd = "screencap -p " + remote_path
        assert g_common_obj.adb_cmd(
            cmd) == 0, "The screen shot not captured correct"

    def capture_screenshot(self, qrcode):
        """capture screenshot and check qrcode
        """
        self.photos.open_a_picture()
        time.sleep(2)
        print "capture template screenshot"
        self.verify_screenshot_qrcode(qrcode)

    def verify_screenshot_qrcode(self, qrcode):
        """ capture screenshot and decode qrcode
        """
        home_screen = tempfile.mktemp(suffix='.png', prefix='screen_', dir='/tmp')
        self.d.screenshot(home_screen)
        _, decode = qrcode_obj.decode_image_qrcode(home_screen)
        print "[Debug] verify_wallpaper qrcode:%s decode:%s" % (qrcode, decode)
        assert decode == qrcode, \
            "[FAILURE] Failed verify wallpaper qrcode:%s decode:%s" % (qrcode, decode)
        remove_temp_file(home_screen)

    def addeffects_check_screenshot(self):
        """ Add Effects and take Screenshot to compare
        """
        self.photos.open_a_picture()
        time.sleep(2)
        print "catch screenshot before_effect.png"
        self.d.screenshot("before.png")
        time.sleep(1)
        cmd = 'mv before.png ' + self.tmpdir + '/before.png'
        os.system(cmd)
        self.photos.apply_filter_to_photos("filter Deimos")
        print "catch screenshot after_effect.png"
        self.d.screenshot("after.png")
        time.sleep(1)
        cmd = 'mv after.png ' + self.tmpdir + '/after.png'
        os.system(cmd)
        time.sleep(1)
        rms = compare_pic.compare_pic(self.tmpdir + "/before.png", self.tmpdir + "/after.png")
        time.sleep(1)
        print "The Similarity value of 2 screenshot is %s" % rms
        assert rms > 5, "The screenshot is same with before add effect"
        time.sleep(5)
        self.d.press.back()
        time.sleep(1)

    def suspend_and_resume(self):
        """ suspend and resume the DUT
        """
        self.d.press.power()
        time.sleep(5)
        self.d.press.power()
        time.sleep(2)

    def check_photos_format(self):
        """ check the photo format
        """
#         self.photos.open_a_picture()
        self.photos.view_picture_details()
        time.sleep(1)
        assert self.d(
            textContains=".png").exists, 'The screenshot format not png'

    def rotate_to_left(self):
        """ rotate screen in app
        """
        self.d.orientation = "l"

    def roate_to_n(self):
        """ rotate screen in normal
        """
        self.d.orientation = "n"

    def popup_notification_bar(self):
        """ slide screen to show the notification bar
        """
        x = self.d.info["displayWidth"]
        y = self.d.info["displayHeight"]
        self.d.swipe(x / 2, 0, x / 2, y / 2)
        time.sleep(3)

    def rotate_20times(self):
        """ rotate 20 times
        """
        for i in range(1, 21):
            self.d.orientation = "r"
            time.sleep(2)
            self.d.orientation = "l"
            time.sleep(2)
            print "rotate %s times" % i

    def rotate_during_video_playback(self):
        """ rotateduring play video
        """
        print "Rotate during play video"
        self.photos.play_video()
        time.sleep(5)
        self.d.orientation = "l"
        time.sleep(2)
        self.d.orientation = "r"

    def swipedown_notification_and_rotate_during_video_playback(self):
        """ rotateduring play video
        """
        print "swipe notification and rotate during play video"
        x = self.d.info["displayWidth"]
        y = self.d.info["displayHeight"]
        self.d.click(x / 2, y / 2)
        time.sleep(1)
        self.d.swipe(x / 2, 0, x / 2, y / 2)
        time.sleep(3)
        self.d.orientation = "l"
        time.sleep(2)
        self.d.orientation = "r"

    def rotate_anticlockwise_5times(self):
        """ rotate screen in Anticlockwise_5times
        """
        print "rotate screen in Anticlockwise_5times"
        self.d.orientation = "l"
        time.sleep(2)
        self.d.orientation = "r"
        time.sleep(2)
        self.d.orientation = "n"
        time.sleep(2)
        self.d.orientation = "l"
        time.sleep(2)
        self.d.orientation = "r"
        time.sleep(2)

    def init_local_pdf(self):
        """ push PDF file to DUT
        """
        config = TestConfig()
        cfg_file = 'tests.tablet.composeui.conf'
        cfg_arti = config.read(cfg_file, 'artifactory')
        config_handle = ConfigHandle()
        cfg_arti["location"] = config_handle.read_configuration('artifactory', 'location', '/etc/oat/', 'sys.conf')
        arti = Artifactory(cfg_arti.get('location'))
        cfg_pdf = config.read(cfg_file, 'PDF')
        pdf_name = cfg_pdf.get("name")
        pdf_path = arti.get(pdf_name)
        g_common_obj.adb_cmd_common('push ' + pdf_path + ' /sdcard/')
        cmd = "am broadcast -a android.intent.action.MEDIA_MOUNTED\
         -d file:///sdcard/"
        g_common_obj.adb_cmd(cmd)
        self.pdf = cfg_pdf.get("filename")

    def delete_local_pdf(self):
        """ delete the pdf pushed
        """
        print "delete the pdf pushed"
        cmd = "rm -rf /sdcard/%s" % self.pdf
        g_common_obj.adb_cmd(cmd)
        cmd = "am broadcast -a android.intent.action.MEDIA_MOUNTED\
         -d file:///sdcard/"
        g_common_obj.adb_cmd(cmd)

    def launch_fbreader_and_pdfplugin(self):
        """ Launch fbreader and pdfplugin via adb am command
        """
        print "Launch FBReader and PDFplugin by adb am"
        g_common_obj.launch_app_am(
            "org.geometerplus.zlibrary.ui.android",
            "org.geometerplus.android.fbreader.FBReader")
        self._locator.wait_exist(self._locator.performance_tests)
        if self.d(resourceId="com.android.packageinstaller:id/permission_allow_button"):
            self.d(resourceId="com.android.packageinstaller:id/permission_allow_button").click.wait()
            time.sleep(1)
        x = self.d.info["displayWidth"]
        y = self.d.info["displayHeight"]
        if not self.d(description="Show book menu").exists:
            self.d.click(x / 2, y / 2)
            time.sleep(2)
        for _ in range(0, 3):
            if not self.d(description="Show book menu").exists:
                self.d.click(x / 2, y * 3 / 4)
                time.sleep(2)
            else:
                break
        self.d(description="Show book menu").click.wait()
        self.d(text="Library").click.wait()
        self.d(resourceId="android:id/list")\
            .child(className="android.widget.LinearLayout", index=7).click()
        time.sleep(2)
        self.d(resourceId="android:id/list")\
            .child(className="android.widget.LinearLayout", index=2).click()
        time.sleep(2)
        if not self.d(text="FBReader_test.pdf").exists:
            self.d(scrollable=True).scroll.to(text="FBReader_test.pdf")
            time.sleep(2)
        self.d(text="FBReader_test.pdf").click()
        time.sleep(2)
        if self.d(text="Read"):
            self.d(text="Read").click()
        if self.d(text="READ"):
            self.d(text="READ").click()
        if self.d(resourceId="com.android.packageinstaller:id/permission_allow_button"):
            self.d(resourceId="com.android.packageinstaller:id/permission_allow_button").click.wait()
        time.sleep(6)

    @staticmethod
    def stop_fbreader_and_pdfplugin():
        """ Stop fbreader and pdfplugin via adb am command
        """
        print "Stop fbreader and pdfplugin by adb am"
        g_common_obj.stop_app_am("org.geometerplus.zlibrary.ui.android")
        g_common_obj.stop_app_am("org.geometerplus.fbreader.plugin.pdf")

    @staticmethod
    def uninstall_fbreader_and_pdfplugin():
        """ uninstall the fbreader and pdfplugin
        """
        print "Uninstall the fbreader and pdfplugin"
        cmd = 'uninstall org.geometerplus.fbreader.plugin.pdf'
        g_common_obj.adb_cmd_common(cmd)
        cmd = 'uninstall org.geometerplus.zlibrary.ui.android'
        g_common_obj.adb_cmd_common(cmd)

    def check_notification_in_pdfreader(self):
        """ check notification during reading in FBReader
        """
        if self.d(text="Allow").exists:
            self.d(text="Allow").click.wait()
            time.sleep(1)
        if self.d(text="ALLOW").exists:
            self.d(text="ALLOW").click.wait()
            time.sleep(1)
        print "------------GFX_fbreader1-------------"
        # filter out not-match info
        cmd = "dumpsys SurfaceFlinger | grep -E 'StatusBar| NavigationBar' | grep -Ev '\:|\[|\('"
        msg1 = g_common_obj.adb_cmd_capture_msg(repr(cmd))
        print msg1
        assert msg1 == "", "The notification or statusbar is popuped"
        x = self.d.info["displayWidth"]
        y = self.d.info["displayHeight"]
        for _ in range(0, 5):
            self.d.swipe(x / 2, 0, x / 2, y)
            self.d.open.quick_settings()
            time.sleep(1)
            if self.d(resourceId="com.android.systemui:id/quick_settings_container").exists:
                break
        time.sleep(3)
        print "------------GFX_fbreader2-------------"
        msg2 = g_common_obj.adb_cmd_capture_msg(repr(cmd))
        print msg2
        assert msg2 != "", "The notification not popup"
