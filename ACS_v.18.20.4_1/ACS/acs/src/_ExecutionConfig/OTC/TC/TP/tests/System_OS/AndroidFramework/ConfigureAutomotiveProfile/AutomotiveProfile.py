"""
@summary: Android Automotive profile test
@since: 04/07/2017
@author: Abbas, Saddam hussain
"""

from testlib.util.uiatestbase import UIATestBase
from testlib.androidframework.common import UiAutomatorUtils
from testlib.util.log import Logger
from testlib.util.common import g_common_obj
from testlib.settings.settings import Settings

LOG = Logger.getlogger(__name__)


class AutomotiveProfile(UIATestBase):
    """Test related to AOSP Launcher.
    """
    def setUp(self):
        super(AutomotiveProfile, self).setUp()
        self.__test_name = __name__
        LOG.debug("Setup: %s" % self.__test_name)
        UiAutomatorUtils.unlock_screen()

    def tearDown(self):
        UiAutomatorUtils.close_all_tasks()

    def test_check_aosp_launcher(self):
        """Test case to check AOSP launcher is available.

        :return: None
        """
        launcher_package_name = 'com.android.launcher'
        cmd = "pm list packages | grep {}".format(launcher_package_name)
        msg = g_common_obj.adb_cmd_capture_msg(cmd)
        if msg.find('package:%s' % launcher_package_name) != -1:
            LOG.debug("AOSP launcher is available")
            assert True
        else:
            LOG.error("AOSP laucher is not available")
            assert False, "AOSP laucher is not available"

    def test_default_aosp_launcher(self):
        """Test case to check AOSP launcher is the default launcher.

        :return: None
        """
        launcher_package_name = "com.android.launcher3"
        settings_object = Settings()
        settings_object.launchSetting()
        g_common_obj.d.press.home()

        if g_common_obj.d(description="Apps",
                          packageNameMatches=launcher_package_name):
            LOG.debug("AOSP launcher is the default launcher")
            assert True
        else:
            LOG.error("AOSP launcher is not the default launcher")
            assert False, "AOSP launcher is not the default launcher"

    def test_ui_mode(self):
        """Testcase to check UiMode for automotive.

        :return: None
        """
        cmd = "dumpsys activity processes | grep mConfiguration"
        msg = g_common_obj.adb_cmd_capture_msg(cmd)
        if msg.find('car') != -1:
            LOG.debug("Found Android Auto UiMode")
            assert True
        else:
            LOG.error("Not able to find Android Auto UiMode")
            assert False, "Not able to find Android Auto UiMode"

    def test_hardware_type(self):
        """Testcase to check the hardware type for automative.

        :return: None
        """
        hardware_package_name = "android.hardware.type.automotive"
        cmd = "pm list features | grep feature:{}".format(
            hardware_package_name)
        msg = g_common_obj.adb_cmd_capture_msg(cmd)
        if msg.find(hardware_package_name) != -1:
            LOG.debug("Found Automotive hardware type")
            assert True
        else:
            LOG.error("Not able to find Automotive hardware type")
            assert False, "Not able to find Automotive hardware type"
