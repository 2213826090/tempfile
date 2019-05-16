# -*- coding:utf-8 -*-

'''
@summary: Android acceleration test.
@since: 06/30/2016
@author: Lijin Xiong
'''

from testlib.util.common import g_common_obj
from testlib.util.uiatestbase import UIATestBase
from testlib.androidframework.cts_test_impl import CTS_Impl
from testlib.androidframework.fetch_resources import resource

class Acceleration(UIATestBase):

    def setUp(self):
        super(Acceleration, self).setUp()
        self.__test_name = __name__
        print "Setup: %s" % self.__test_name
        self._cts_test = CTS_Impl()
        resource.disable_app_verification()
        for i in ["acceleration_stubs", "acceleration"]:
            _apk_path = resource.get_resource_from_atifactory\
            ("tests.tablet.artifactory.conf", "CTS_APKS", i)
            g_common_obj.adb_cmd_common("install -r %s" % _apk_path)

    def test_android_acceleration(self):
        self._cts_test.run_cts_test_acceleration("acceleration")
