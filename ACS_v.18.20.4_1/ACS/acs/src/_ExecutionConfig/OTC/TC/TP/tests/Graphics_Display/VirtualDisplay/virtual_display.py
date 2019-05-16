# -*- coding: utf-8 -*-
'''
@summary: Virtual Display test
@since: 09/19/2017
@author: Rui
'''
from testlib.util.uiatestbase import UIATestBase
from testlib.util.log import Logger
from testlib.graphics.common import g_common_obj, pkgmgr, get_resource_from_atifactory, adb32, multi_display
from testlib.graphics.sample_apidemo import SampleApiDemoImpl
from testlib.graphics.compare_pic_impl import compare_pic
from testlib.graphics.photos_impl import get_photo_implement
import time
import threading


CONFIG_FILE = "tests.tablet.artifactory.conf"
SECTION = "content_virtual_display"
CLASS_NAME = "android.display.cts.VirtualDisplayTest"
PACKAGE_NAME = "com.android.cts.display"

LOG = Logger.getlogger(__name__)

class VirtualDisplay(UIATestBase):

    def setUp(self):
        super(VirtualDisplay, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.apk = get_resource_from_atifactory(CONFIG_FILE, SECTION, 'name')
        self.d = g_common_obj.get_device()
        self.apiDemoImpl = SampleApiDemoImpl()
        self.photoImpl = get_photo_implement()
        self.photoImpl.stop_photos_am()
        self.photoImpl.rm_delete_photos()
        self.photoImpl.refresh_sdcard()

    def tearDown(self):
        super(VirtualDisplay, self).tearDown()
        print "[Teardown]: %s" % self._test_name
        self.photoImpl.stop_photos_am()
        self.photoImpl.rm_delete_photos()
        self.photoImpl.refresh_sdcard()

    def test_Virtual_DisplayTest(self):
        if not pkgmgr._package_installed(PACKAGE_NAME):
            g_common_obj.adb_cmd_common("install -r %s" % self.apk)
        assert adb32.run_instrument_test(CLASS_NAME, PACKAGE_NAME), "Test failed."

    def test_VirtualDisplay_HDMI(self):
        self.apiDemoImpl.install_apk()
        multi_display.is_multi_displayed()
        self.apiDemoImpl.launch_app_am()
        self.apiDemoImpl.perform_presentation_through_mediarouter()
        multi_display.is_presentation_with_mediarouter()
        self.apiDemoImpl.start_presentation()
        multi_display.is_presentation()

    def test_VirtualDispaly_screenshots(self):
        # get right top corner for capturing time date.
        x, y = self.d.info['displayWidth'], self.d.info['displayHeight']
        sx, sy, ex, ey = x - 300, 0, x, 100
        target_strs = ['1', '2', '3', '4', '5', '6', '7', '8', '9', '0', ':']
        img_text = compare_pic.extract_strings_from_croped_screen_shot(0, sx, sy, ex, ey)
        assert (i in img_text for i in target_strs), 'Fail to get correct screenshot.'

    def test_VirtualDispaly_RecordingVideo(self):
        video_record_file = '/sdcard/Movies/demo.mp4'
        g_common_obj.shell_cmd('adb shell rm %s' % video_record_file)
        # do something during video recording. or not duration will be 0.
        def launch_settings_app():
            from testlib.graphics.common import launch_settings_am
            LOG.info("Launch settings app during video recording.")
            _d = g_common_obj.get_device()
            for i in range(2):
                launch_settings_am()
                _d.press.home()
                time.sleep(1)

        thread = threading.Timer(2, launch_settings_app)
        thread.start()

        LOG.info("Start video recording for 20 seconds.")
        g_common_obj.adb_cmd_capture_msg('screenrecord %s --time-limit 20' % video_record_file)
        time.sleep(2)
        self.photoImpl.play_video_command(video_record_file, 20)
        g_common_obj.assert_exp_happens()
