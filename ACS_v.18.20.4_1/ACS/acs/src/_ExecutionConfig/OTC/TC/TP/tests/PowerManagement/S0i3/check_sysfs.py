#! /usr/bin/env python
# coding:utf-8

from testlib.util.uiatestbase import UIATestBase
from testlib.util.common import g_common_obj

class CheckSysfs(UIATestBase):
    def setUp(self):
        self._test_name = __name__
        print
        print "[Setup]: %s" % self._test_name
        g_common_obj.root_on_device()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name

    def test_check_sysfs_cpufreq(self):
        """
        check sysfs cpufreq
        """
        print "[RunTest]: %s" % self.__str__()
        cpu_dir = "cat /sys/devices/system/cpu/cpu%d/cpufreq/"
        cur_freq = cpu_dir + "scaling_cur_freq"
        governor = cpu_dir + "scaling_governor"
        time_in_state = cpu_dir + "stats/time_in_state"
        for i in range(4):
            msg = g_common_obj.adb_cmd_capture_msg(cur_freq % i)
            # verify the result is a number
            int(msg)
            msg = g_common_obj.adb_cmd_capture_msg(governor % i)
            assert msg == "interactive"
            msg = g_common_obj.adb_cmd_capture_msg(time_in_state % i)
            # verify there are only numbers in the result
            for item in msg.split():
                int(item)

    def test_check_sysfs_cpuidle(self):
        """
        check sysfs cpuidle
        """
        print "[RunTest]: %s" % self.__str__()
        cmd = "cat /sys/devices/system/cpu/cpu0/cpuidle/state0/time"
        msg = g_common_obj.adb_cmd_capture_msg(cmd)
        # verify msg is a number
        int(msg)

    def write_value(self, str_value):
        for i in [1,2,3]:
            cmd = "'cd /sys/devices/system/cpu/cpu%d; echo %s > online; cat online'" % (i, str_value)
            msg = g_common_obj.adb_cmd_capture_msg(cmd)
            assert msg == str_value

    def test_check_sysfs_hotplug(self):
        """
        check sysfs hotplug
        """
        print "[RunTest]: %s" % self.__str__()
        self.write_value('0')
        self.write_value('1')

