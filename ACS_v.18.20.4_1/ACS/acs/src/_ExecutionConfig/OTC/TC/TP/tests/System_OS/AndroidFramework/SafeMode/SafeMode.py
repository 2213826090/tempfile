# -*- coding:utf-8 -*-

'''
@summary: Android safe mode test.
@since: 07/11/2016
@author: Lijin Xiong
'''

import time

from testlib.util.uiatestbase import UIATestBase
from testlib.util.common import g_common_obj
from testlib.androidframework.adb_utils import AdbUtils
from testlib.androidframework.fetch_resources import resource
from testlib.androidframework.common import UiAutomatorUtils
from testlib.util.log import Logger

LOG = Logger.getlogger(__name__)

# to do:
# below function is implemented because UiAutomatorUtils.unlock_screen() will
# not work in Broxton-M. This function will be soon moved to common class

def unlock_screen():
    menu_cmd = "input keyevent MENU"
    home_cmd = "input keyevent HOME"

    g_common_obj.adb_cmd(menu_cmd)
    g_common_obj.adb_cmd(menu_cmd)
    g_common_obj.adb_cmd(home_cmd)

class Safe_Mode(UIATestBase):

    def setUp(self):
        super(Safe_Mode, self).setUp()
        self.__test_name = __name__
        print "Setup: %s" % self.__test_name
        self.d = g_common_obj.get_device()
        resource.disable_app_verification()
        _apk_path = resource.get_resource_from_atifactory\
        ("tests.tablet.artifactory.conf", "SDK_API", "api_demos")
        g_common_obj.adb_cmd_common("install -r %s" % _apk_path)

    def test_safe_mode_by_property(self):
        g_common_obj.root_on_device()
        set_safemode_cmd = "setprop persist.sys.safemode 1"
        g_common_obj.adb_cmd(set_safemode_cmd)
        AdbUtils.reboot_device()
        UiAutomatorUtils.unlock_screen()
        g_common_obj.root_on_device()
        try:
            UiAutomatorUtils.launch_app_from_apps_menu("API Demos")
        except:
            print "Pass"
        else:
            assert None, "The 3rd party app is still visible in all apps menu when DUT in safe mode!"

        AdbUtils.reboot_device()
        UiAutomatorUtils.unlock_screen()
        g_common_obj.root_on_device()
        try:
            UiAutomatorUtils.launch_app_from_apps_menu("API Demos")
        except:
            assert None, "The 3rd party app is invisible in all apps menu when DUT is not in safe mode!"
        else:
            print "Pass"

    def test_safe_mode_by_ui(self):
        """Test case to start DUT in safe mode through UI.

        :return: None
        """
        unlock_screen()
        g_common_obj.root_on_device()

        # Below command will work in Android release above 'M' for longpress
        #  POWER
        # longpress_power = "input keyevent --longpress POWER"
        # g_common_obj.adb_cmd(longpress_power)

        # simulating longpress POWER through sendevent
        cmd1 = 'sendevent /dev/input/event0 1 116 1'
        cmd2 = 'sendevent /dev/input/event0 0 0 0'
        cmd3 = 'sendevent /dev/input/event0 1 116 0'

        while [0-3]:
            g_common_obj.adb_cmd(cmd1)
            g_common_obj.adb_cmd(cmd2)
            time.sleep(2)
            g_common_obj.adb_cmd(cmd3)
            g_common_obj.adb_cmd(cmd2)
            if g_common_obj.d(text="Power off").exists:
                g_common_obj.d(text="Power off").long_click()
                g_common_obj.d(text="OK").click()
                break

        timeout = 300
        count = 6
        sleep_time = 5
        time.sleep(sleep_time)
        while count < timeout:
            time.sleep(sleep_time)
            prop_val = g_common_obj.adb_cmd_capture_msg('getprop '
                                                        'sys.boot_completed')
            if '1' in prop_val:
                LOG.debug("dut booted in safe mode successfully")
                break
            count += sleep_time

        g_common_obj.root_on_device()
        unlock_screen()
        failed = False
        try:
            UiAutomatorUtils.launch_app_from_apps_menu("API Demos")
        except:
            LOG.debug("3rd party apps are not seen in Safe mode")
        else:
            LOG.error("3rd party app is still visible in all apps menu when "
                      "DUT is in safe mode!")
            # below is added to not exit code
            failed = True

        AdbUtils.reboot_device()
        time.sleep(sleep_time+sleep_time)
        g_common_obj.root_on_device()
        unlock_screen()
        if failed == False:
            try:
                UiAutomatorUtils.launch_app_from_apps_menu("API Demos")
            except:
                LOG.error("3rd party app is invisible in all apps menu when "
                          "DUT is not in safe mode!")
                assert False, "3rd party app is invisible in all apps menu " \
                              "when DUT is not in safe mode!"
            else:
                LOG.debug("3rd party apps are visible in all apps menu when "
                          "DUT is not in safe mode")
                assert True
        else:
            assert False, "3rd party app is still visible in all apps menu " \
                          "when DUT is in safe mode!"