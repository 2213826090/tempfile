# -*- coding:utf-8 -*-

'''
@summary: Android CTS test implementation.
@since: 06/30/2016
@author: Lijin Xiong
'''

from testlib.util.common import g_common_obj


class CTS_Impl(object):

    def run_cts_test_accounts(self, ctstest_name):
        test_cmd = "am instrument -e class android.%s.cts.AccountManagerTest" \
                   " -w android.%s.cts/android.support.test.runner.AndroidJUnitRunner" % (ctstest_name, ctstest_name)
        output = g_common_obj.adb_cmd_capture_msg(test_cmd, time_out=600)
        assert 'OK' in output, "Cts test %s failed!" % (ctstest_name)

    def run_cts_test_animation(self, ctstest_name):
        test_cmd = "am instrument -e package android.%s.cts" \
                   " -w com.android.cts.%s/android.support.test.runner.AndroidJUnitRunner" % (ctstest_name, ctstest_name)
        output = g_common_obj.adb_cmd_capture_msg(test_cmd, time_out=600)
        assert 'OK' in output, "Cts test %s failed!" % (ctstest_name)

    def run_cts_test_animation_n(self, ctstest_name):
        test_cmd = "am instrument -e package android.%s.cts" \
                   " -w android.%s.cts/android.support.test.runner.AndroidJUnitRunner" % (ctstest_name, ctstest_name)
        output = g_common_obj.adb_cmd_capture_msg(test_cmd, time_out=600)
        assert 'OK' in output, "Cts test %s failed!" % (ctstest_name)

    def run_cts_test_acceleration(self, ctstest_name):
        test_cmd = "am instrument -e package android.%s.cts" \
                   " -w com.android.cts.%s/android.support.test.runner.AndroidJUnitRunner" % (ctstest_name, ctstest_name)
        output = g_common_obj.adb_cmd_capture_msg(test_cmd, time_out=600)
        assert 'OK' in output, "Cts test %s failed!" % (ctstest_name)