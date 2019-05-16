# -*- coding:utf-8 -*-

'''
@summary: Android animation test.
@since: 06/30/2016
@author: Lijin Xiong
'''

from testlib.util.common import g_common_obj
from testlib.util.uiatestbase import UIATestBase
from testlib.androidframework.cts_test_impl import CTS_Impl
from testlib.androidframework.fetch_resources import resource
from testlib.androidframework.common import EnvironmentUtils

class Animation(UIATestBase):

    def setUp(self):
        super(Animation, self).setUp()
        self.__test_name = __name__
        print "Setup: %s" % self.__test_name
        self._cts_test = CTS_Impl()
        resource.disable_app_verification()
        if EnvironmentUtils.get_android_version() == 'N':
            _apk_path = resource.get_resource_from_atifactory\
            ("tests.tablet.artifactory.conf", "CTS_APKS", "animation_n")
        else:
            _apk_path = resource.get_resource_from_atifactory\
            ("tests.tablet.artifactory.conf", "CTS_APKS", "animation")
        g_common_obj.adb_cmd_common("install -r %s" % _apk_path)

    def test_android_animation(self):
        if EnvironmentUtils.get_android_version() == 'N':
            self._cts_test.run_cts_test_animation_n("animation")
        else:
            self._cts_test.run_cts_test_animation("animation")
