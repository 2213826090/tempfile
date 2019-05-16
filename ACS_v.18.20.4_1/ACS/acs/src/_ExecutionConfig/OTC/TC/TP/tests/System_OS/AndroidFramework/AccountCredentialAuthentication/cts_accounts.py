# -*- coding:utf-8 -*-

'''
@summary: Android accounts test.
@since: 06/30/2016
@author: Lijin Xiong
'''

from testlib.util.common import g_common_obj
from testlib.util.uiatestbase import UIATestBase
from testlib.androidframework.cts_test_impl import CTS_Impl
from testlib.androidframework.fetch_resources import resource

class Accounts(UIATestBase):

    def setUp(self):
        super(Accounts, self).setUp()
        self.__test_name = __name__
        print "Setup: %s" % self.__test_name
        self._cts_test = CTS_Impl()
        resource.disable_app_verification()
        _apk_path = resource.get_resource_from_atifactory\
        ("tests.tablet.artifactory.conf", "CTS_APKS", "accounts")
        g_common_obj.adb_cmd_common("install -r %s" % _apk_path)

    def test_android_accounts(self):
        self._cts_test.run_cts_test_accounts("accounts")