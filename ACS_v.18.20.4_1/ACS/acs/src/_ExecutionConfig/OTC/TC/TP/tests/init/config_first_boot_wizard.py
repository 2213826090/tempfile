from testlib.util.uiatestbase import UIATestBase
from testlib.dut_init.dut_init_impl import Function
from testlib.util.common import g_common_obj
import time
import os

class ConfigFirstBootWizard(UIATestBase):

    def setUp(self):
        self._test_name = __name__
        print
        print "[Setup]: %s" % self._test_name
        cfg_file = 'tests.tablet.dut_init.conf'
        self.retry_num = self.config.read(cfg_file, 'init_list').get("configure_first_boot_wizard")
        self.retry_num = int(self.retry_num)
        self.d = g_common_obj.get_device()
        self.x = self.d.info["displayWidth"]
        self.y = self.d.info["displayHeight"]
        self.serial = self.d.server.adb.device_serial()
        self.func = Function()


    def tearDown(self):
        print "[Teardown]: %s" % self._test_name

    def testConfigFirstBootWizard(self):
        """
        Config first boot wizard
        """
        print "[RunTest]: %s" % self.__str__()

        self.d.wakeup()
        self.d.press("home")
        time.sleep(2)
        if self.d(textContains="Drive safely").exists:
            self.d(text="Owner").click.wait()
            return
        android_version_release = g_common_obj.adb_cmd_capture_msg("getprop |grep 'version.release'").split(':')[-1].strip()
        ro_boot_hardware = g_common_obj.adb_cmd_capture_msg("getprop |grep 'ro.boot.hardware'").split(':')[-1].strip()
        if (android_version_release == '[8.0.0]') or (android_version_release == '[8.1.0]'):
            if ro_boot_hardware != '[androidia_64]':
                return
        if self.d(textContains="Chrome") or self.d(textContains="Play Store"):
            return
        self.d.wakeup()
        if self.d(resourceId="com.android.systemui:id/lock_icon"):
            self.func.close_lock_screen()
            return

        succeed = False
        for i in range(self.retry_num):
            try:
                if self.d(text="OK").exists:
                    self.d(text="OK").click.wait()

                if self.d(text="Welcome") or \
                        self.d(resourceId="com.google.android.setupwizard:id/welcome_title") or \
                        self.d(resourceId="com.google.android.setupwizard:id/start"):
                    self.d.click(100, 100)
                    self.d.click(self.x-100, 100)
                    self.d.click(self.x-100, self.y-100)
                    self.d.click(100, self.y-100)
                    time.sleep(2)
                    if self.d(text="OK").exists:
                        self.d(text="OK").click.wait()
                        self.d(text="OK").wait.gone(timeout=3000)
                    self.func.close_lock_screen()
                    succeed = True
                    return

                if self.d(text="Welcome") or \
                        self.d(resourceId="com.google.android.setupwizard:id/welcome_title") or \
                        self.d(resourceId="com.google.android.setupwizard:id/start"):
                    self.func.setup_guideline()
                    if self.d(text="OK").exists:
                        self.d(text="OK").click.wait()
                    succeed = True
                break
            except Exception as e:
                print e
        assert succeed

