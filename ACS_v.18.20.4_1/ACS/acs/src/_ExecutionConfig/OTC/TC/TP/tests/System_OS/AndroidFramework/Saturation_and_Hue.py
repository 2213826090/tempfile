# -*- coding:utf-8 -*-

'''
@summary: Android Saturation and Hue test.
@since: 07/06/2016
@author: Lijin Xiong
'''

from testlib.util.common import g_common_obj
from testlib.util.uiatestbase import UIATestBase
from testlib.androidframework.fetch_resources import resource


class SaturationHue(UIATestBase):

    def setUp(self):
        super(SaturationHue, self).setUp()
        self.__test_name = __name__
        print "Setup: %s" % self.__test_name
        resource.disable_app_verification()
        _apk_path = resource.get_resource_from_atifactory\
        ("tests.tablet.artifactory.conf", "SDK_API", "api_test")
        g_common_obj.adb_cmd_common("install -r %s" % _apk_path)
        _apk_path = resource.get_resource_from_atifactory\
        ("tests.tablet.artifactory.conf", "CTS_APKS", "graphics_test")
        g_common_obj.adb_cmd_common("install -r %s" % _apk_path)

    def test_verify_saturation_and_hue(self):
        test_cmds = ["am instrument -e class com.intel.test.apitests.tests.ColorTestsDriver#testColorToHsv"
                     " -w com.intel.test.apitests/com.intel.test.apitests.runners.GenericArgumentPassingTestRunner",
                     "am instrument -e class android.graphics.cts.ColorTest#testHSVToColor1"
                     " -w com.android.cts.graphics/android.support.test.runner.AndroidJUnitRunner",
                     "am instrument -e class android.graphics.cts.ColorTest#testHSVToColor2"
                     " -w com.android.cts.graphics/android.support.test.runner.AndroidJUnitRunner",
                     "am instrument -e class android.graphics.cts.ColorMatrixTest#testSetSaturation"
                     " -w com.android.cts.graphics/android.support.test.runner.AndroidJUnitRunner"]
        for cmd in test_cmds:
            print cmd
            output = g_common_obj.adb_cmd_capture_msg(cmd)
            self.assertTrue("OK" in output, "some saturation and hue instrumentation tests failed")