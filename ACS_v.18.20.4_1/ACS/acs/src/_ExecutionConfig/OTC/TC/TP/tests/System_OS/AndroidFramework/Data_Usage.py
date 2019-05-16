# -*- coding:utf-8 -*-

'''
@summary: Android data usage test.
@since: 07/06/2016
@author: Lijin Xiong
'''

import time
from string import Template
from testlib.util.common import g_common_obj
from testlib.util.uiatestbase import UIATestBase
from testlib.androidframework.cts_test_impl import CTS_Impl
from testlib.androidframework.common import Settings,ApiTestsInterface,UiAutomatorUtils
from testlib.androidframework.fetch_resources import resource
from testlib.util.log import Logger

LOG = Logger.getlogger(__name__)

class DataUsage(UIATestBase):

    def setUp(self):
        super(DataUsage, self).setUp()
        self.__test_name = __name__
        print "Setup: %s" % self.__test_name
        self._cts_test = CTS_Impl()
        UiAutomatorUtils.unlock_screen()
        resource.disable_app_verification()
        _apk_path = resource.get_resource_from_atifactory\
        ("tests.tablet.artifactory.conf", "SDK_API", "api_test")
        g_common_obj.adb_cmd_common("install -r %s" % _apk_path)

    def test_data_usage(self):
        data_usage = Settings.get_data_usage_info()
        print data_usage
        self.assertTrue(len(data_usage) > 0)

    def test_data_usage_new_installed_app(self):
        ApiTestsInterface.args_template = Template('$args')
        instrumentation_args = '-e apTestPage "https://www.youtube.com/" -e waitBeforeLogCheck 15000'
        surf_internet_result = ApiTestsInterface.run_instrumentation(class_name="WebViewTestsDriver",
                                                                     method_name="testStreamPlayback",
                                                                     instrumentation_args=instrumentation_args,
                                                                     runner_name="WifiEncryptionTestRunner")
        ApiTestsInterface.was_instrumentation_test_successful(surf_internet_result)
        self.assertTrue(ApiTestsInterface.was_instrumentation_test_successful(surf_internet_result),
                        "Internet browsing was not successful")
        try:
            data_usage = Settings.get_data_usage_values()
            Settings.go_to_main_screen()
            Settings.disable_wifi()
            Settings.go_to_main_screen()
            time.sleep(5)
            final_data_usage = Settings.get_data_usage_values()
            self.assertTrue("ApiTests" in data_usage.keys(), "could not find browsing app in initial data usage stats")
            LOG.info("initial ApiTests data usage: " + str(data_usage["ApiTests"]))
            self.assertTrue("ApiTests" in final_data_usage.keys(),
                            "could not find browsing app in final data usage stats")
            LOG.info("final ApiTests data usage: " + str(final_data_usage["ApiTests"]))
        finally:
            Settings.go_to_main_screen()
            Settings.enable_wifi()