# -*- coding: utf-8 -*-
'''
@summary: This case will not throw Exception,
need framework support check the crash issue.
@since: 03/31/2015
@author: Yingjun Jin
'''
from time import sleep
import threading
from testlib.util.uiatestbase import UIATestBase
from testlib.graphics.common import get_resource_from_atifactory, pkgmgr, multi_display
from testlib.util.common import g_common_obj
from testlib.graphics.compare_pic_impl import compare_pic


class ComposeUI(UIATestBase):

    @classmethod
    def setUpClass(self):
        """
        init enviroment
        """
        super(ComposeUI, self).setUpClass()
        self.case_cfg = 'tests.tablet.gits.conf'
        self.activity_name = ".MainActivity"
        self.basic_pkg_name = "com.example.android.elevationbasic"
        self.drag_pkg_name = "com.example.android.elevationdrag"

    def setUp(self):
        super(ComposeUI, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.d = g_common_obj.get_device()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(ComposeUI, self).tearDown()

    def test_shadow_sampleapkelevationbasic(self):
        """
        refer TC test_Shadow_SampleAPKElevationBasic
        """
        print "[RunTest]: %s" % self.__str__()
        # install apk if not exists in package list
        if self.basic_pkg_name not in pkgmgr.get_packages():
            apk_path = get_resource_from_atifactory(self.case_cfg, 'ElevationBasic', 'apk')
            pkgmgr.apk_install(apk_path)
        # launch app
        run_cmd = "am start -S -n %s/%s" % (self.basic_pkg_name, self.activity_name)
        g_common_obj.adb_cmd_capture_msg(run_cmd)
        sleep(2)
        # screen capture before operation
        sx, sy, ex, ey = self.d.info["displayWidth"] / 2, \
                         0, \
                         self.d.info["displayWidth"], \
                         self.d.info["displayHeight"]
        img_before = multi_display.get_screenshot_multidisplay_croped(0, sx, sy, ex, ey)
        # define operation type
        def long_tap_content():
            d = g_common_obj.get_device()
            d(resourceId="com.example.android.elevationbasic:id/floating_shape_2").long_click()
        # start test
        thread = threading.Timer(1.5, long_tap_content)
        thread.start()
        # screen capture after operation
        sleep(2)
        img_after = multi_display.get_screenshot_multidisplay_croped(0, sx, sy, ex, ey)
        assert compare_pic.compare_pic(img_before, img_after) > 4, "No diff after operation"

    def test_shadow_sampleapkelevationdrag(self):
        """
        refer TC test_Shadow_SampleAPkElevationDrag
        """
        print "[RunTest]: %s" % self.__str__()
        if self.drag_pkg_name not in pkgmgr.get_packages():
            apk_path = get_resource_from_atifactory(self.case_cfg, 'ElevationDrag', 'apk')
            pkgmgr.apk_install(apk_path)
        # launch app
        run_cmd = "am start -S -n %s/%s" % (self.drag_pkg_name, self.activity_name)
        g_common_obj.adb_cmd_capture_msg(run_cmd)
        sleep(2)
        # screen capture before operation
        sx, sy, ex, ey = self.d.info["displayWidth"] / 2, \
                         0, \
                         self.d.info["displayWidth"], \
                         self.d.info["displayHeight"]
        img_before = multi_display.get_screenshot_multidisplay_croped(0, sx, sy, ex, ey)
        # define operation type
        def drag_content():
            d = g_common_obj.get_device()
            d(resourceId="com.example.android.elevationdrag:id/circle").drag.to(sx, sy + 100)
        # start test
        thread = threading.Timer(1.5, drag_content)
        thread.start()
        # screen capture after operation
        sleep(2)
        img_after = multi_display.get_screenshot_multidisplay_croped(0, sx, sy, ex, ey)
        assert compare_pic.compare_pic(img_before, img_after) > 4, "No diff after operation"
