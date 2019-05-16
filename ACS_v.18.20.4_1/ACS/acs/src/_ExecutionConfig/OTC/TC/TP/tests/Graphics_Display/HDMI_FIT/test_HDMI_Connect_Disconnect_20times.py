# -*- coding: utf-8 -*-
'''
Created on 03/13/2015
@author: Ding, JunnanX
'''

import re

from testlib.util.common import g_common_obj
from testlib.util.uiatestbase import UIATestBase
from testlib.graphics.common import logcat
from testlib.graphics.extend_systemui_impl import SystemUiExtendImpl
from testlib.graphics.display_cable_switch_impl import DisplayCableSwitchImpl


class HDMIConnectDisconnectTest(UIATestBase):

    @classmethod
    def setUpClass(cls):
        print "[setUpClass]: %s" % cls.__name__
        super(HDMIConnectDisconnectTest, cls).setUpClass()
        cls.d = g_common_obj.get_device()
        g_common_obj.root_on_device()
        g_common_obj.remount_device()

    def setUp(self):
        print "[Setup]:%s" % self.__class__.__name__
        super(HDMIConnectDisconnectTest, self).setUp()

        cmd = 'date +"%m-%d %H:%M:%S"'
        self.test_time_mark = g_common_obj.adb_cmd_capture_msg(repr(cmd))

        self.systemui = SystemUiExtendImpl()
        self.display_cable_switch = DisplayCableSwitchImpl()
        self.display_cable_switch.setup()
        self.display_cable_switch.switch_off()

        self.systemui.unlock_screen()
        self.d.screen.on()
        self.d.orientation = 'natural'

    def tearDown(self):
        print "[tearDown]:%s" % self.__class__.__name__
        cmd = 'logcat -d -v time -t "%s.000" *:F|grep -i Fatal' % (self.test_time_mark)
        fatal_msg = g_common_obj.adb_cmd_common(cmd)
        assert not fatal_msg,\
            "occurred Fatal error during testing:%s" % (fatal_msg)
        super(HDMIConnectDisconnectTest, self).tearDown()

    @classmethod
    def tearDownClass(cls):
        print "[tearDownClass]: %s" % cls.__name__
        super(HDMIConnectDisconnectTest, cls).tearDownClass()

    def test_HDMI_Connect_Disconnect_20times(self):
        """
        test_HDMI_Connect_Disconnect_20times

        Steps:
        1. hdmi cable connected and disconnected check hdmi display and hdmi status
            Display is OK,no error and crash.
        """
        print "[RunTest]: %s" % self.__str__()

        print """[Step] 1. hdmi cable connected and disconnected check hdmi display and hdmi status
                            Display is OK,no error and crash."""
        for count in range(1, 21):
            print "count %s" % count
            self.d.screen.on()
            mark_time = logcat.get_device_time_mark()
            self.display_cable_switch.switch_on()
            dut_log = logcat.get_device_log(mark_time)
            catch_log = re.findall('.+WiredAccessoryManager.+device hdmi connected.+', dut_log)
            print '-' * 60 + '\n'
            print catch_log
            print '-' * 60 + '\n'
            assert catch_log,\
                "[FAILURE] Connect failed"

            mark_time = logcat.get_device_time_mark()
            self.display_cable_switch.switch_off()
            dut_log = logcat.get_device_log(mark_time)
            catch_log = re.findall('.+WiredAccessoryManager.+device hdmi disconnected.+', dut_log)
            print '-' * 60 + '\n'
            print catch_log
            print '-' * 60 + '\n'
            assert catch_log,\
                "[FAILURE] Disconnect failed"
