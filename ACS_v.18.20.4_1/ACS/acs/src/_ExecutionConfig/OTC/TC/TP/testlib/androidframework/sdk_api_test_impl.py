# -*- coding:utf-8 -*-

'''
@summary: Android SDK API test implementation.
@since: 06/30/2016
@author: Lijin Xiong
'''

from testlib.util.common import g_common_obj


class SDK_API_Impl(object):

    def run_sdk_api_test(self, case_name):
        _cmd_line = "am instrument \
        -e class com.intel.test.apitests.tests.%s \
        -w com.intel.test.apitests/android.test.InstrumentationTestRunner" % (case_name)
        output = g_common_obj.adb_cmd_capture_msg(_cmd_line)
        assert 'OK' in output, "%s failed!" % case_name.split('#')[1]