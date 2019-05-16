# -*- coding: utf-8 -*-
'''
Created on 04/22/2015
@author: Ding, JunnanX
'''

import time

from testlib.util.common import g_common_obj
from testlib.util.uiatestbase import UIATestBase
from testlib.graphics.common import logcat
from testlib.graphics.dumpsys_impl import DumpSysImpl
from testlib.graphics.extend_systemui_impl import SystemUiExtendImpl
from testlib.graphics.system_settings_impl import SystemSettingsImpl


class TouchScreenDuringDUTDimTest(UIATestBase):

    @classmethod
    def setUpClass(cls):
        print "[setUpClass]: %s" % cls.__name__
        super(TouchScreenDuringDUTDimTest, cls).setUpClass()
        cls.d = g_common_obj.get_device()
        g_common_obj.root_on_device()
        g_common_obj.remount_device()

    def setUp(self):
        print "[Setup]:%s" % self.__class__.__name__
        super(TouchScreenDuringDUTDimTest, self).setUp()
        self.mark_time = logcat.get_device_time_mark()

        self.sys_setting = SystemSettingsImpl()
        self.systemui = SystemUiExtendImpl()
        self.dumpsys = DumpSysImpl()

        self.systemui.unlock_screen()
        self.d.screen.on()
        self.sleep_time = 15

        self.org_screen_brightness =\
            self.sys_setting.get_system_setting('screen_brightness')
        self.org_screen_off_timeout = \
            self.sys_setting.get_system_setting('screen_off_timeout')

    def tearDown(self):
        print "[tearDown]:%s" % self.__class__.__name__

        self.sys_setting.put_system_setting('screen_brightness',
                                            self.org_screen_brightness)
        self.sys_setting.put_system_setting('screen_off_timeout',
                                            self.org_screen_off_timeout)
        # Task:Specified watching.
#         fatal_msg = logcat.get_device_log(self.mark_time, filters='*:F')
#         assert len(fatal_msg) == 0,\
#             "occurred Fatal error during testing:\n%s" % (fatal_msg)
#         super(TouchScreenDuringDUTDimTest, self).tearDown()

    @classmethod
    def tearDownClass(cls):
        print "[tearDownClass]: %s" % cls.__name__
        super(TouchScreenDuringDUTDimTest, cls).tearDownClass()

    def test_Display_TouchScreenDuringDUTDim_10times(self):
        """
        test_Display_TouchScreenDuringDUTDim_10times

        Steps:
        1. Go to Settings > Display > Sleep and set a small timeout for the display
            Display timeout is set.
        2. Wait for the timeout to expires and for the screen to begin to darken.
            Screen begins to darken (brightness is rapidly diminished).
        3. Touch the screen while the display darkens but before it turns off.
            The transition to sleep is interrupted and it will start again after the timeout set in Step 1.
        4. Repeat from Step 2 for 10 times (or more if you have the test automated).
            Each time the transition to sleep is interrupted and at the end of the test DUT continues to work as usual.
        """
        print "[RunTest]: %s" % self.__str__()

        print """[Step] 1. Go to Settings > Display > Sleep and set a small timeout for the display
            Display timeout is set."""
        self.sys_setting.put_system_setting('screen_brightness', '255')
        self.sys_setting.put_system_setting('screen_off_timeout',
                                            str(self.sleep_time * 1000))
        self.d.press.home()

        print """[Step] 2. Wait for the timeout to expires and for the screen to begin to darken.
            Screen begins to darken (brightness is rapidly diminished)."""
        print """[Step] 3. Touch the screen while the display darkens but before it turns off.
            The transition to sleep is interrupted and it will start again after the timeout set in Step 1."""
        print """[Step] 4. Repeat from Step 2 for 10 times (or more if you have the test automated).
            Each time the transition to sleep is interrupted and at the end of the test DUT continues to work as usual."""
        for i in range(1, 31):
            print "[Debug] times:%s" % (i)
            start_time = time.time()
            end_time = start_time + self.sleep_time
            screen_dimed = False
            while time.time() < end_time:
                current_time = time.time()
                dump_info = self.dumpsys\
                    .dump_display_power_state()
                policy_state = dump_info['policy']
                assert policy_state != 'OFF',\
                    "[FAILURE] Abnormal state screen off, %s times" % (i)
                screen_dimed = True if policy_state == 'DIM' else False
                if screen_dimed:
                    print "[Debug] DIM at second %.3f" % (current_time - start_time)
                    time.sleep(2)
                    self.d.press.home()
                    break
                time.sleep(0.5)
            assert screen_dimed == True,\
                "[FAILURE] Screen not into DIM mode, %s times" % (i)
