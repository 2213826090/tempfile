import os.path
from time import sleep
import sys
from subprocess import Popen, PIPE
from test_utils import *

if os.getenv('ACS_EXECUTION_CONFIG_PATH') is not None:
    sys.path.insert(0, (os.getenv('ACS_EXECUTION_CONFIG_PATH') + '/TC/TP'))
else:
    sys.path.insert(0,(os.getcwd() + '/_ExecutionConfig/OTC/TC/TP'))

from testlib.util.common import g_common_obj
from testlib.androidframework.common import *
from testlib.androidframework.dut_manager import dut_manager
from adb_helper.adb_utils import *

class USBProtectionEnabled(unittest.TestCase):
    '''
    Description: Check for apps install/uninstall when verify apps over USB
                 is enabled.

    Usage: Functionality checking when verify apps over USB is enabled.

    '''

    TAG = "USBProtectionEnabled"

    def setUp(self):
        print self.TAG, "setUp"
        self.d = g_common_obj.get_device(dut_manager.active_uiautomator_device_serial)

    def tearDown(self):
        print self.TAG, "tearDown"

    def test_adb_install_usb_protection(self):
        goto_settings = "am start com.android.settings"
        AdbUtils.run_adb_cmd(goto_settings)

        self.d(scrollable=True).scroll.toEnd()
        dev_option = self.d(text='Developer options')
        if ( dev_option.exists ):
            self.enable_verify_apps_over_usb()
        else:
            self.d(textContains="About").click()
            self.d(scrollable=True).fling.toEnd()
            if self.d(textContains="Build number"):
                for i in range(8):
                    self.d(text="Build number").click()
                    sleep(1)
            self.d.press.back()
            if not dev_option.exists:
                self.assertFalse("Developer options is not enabled")
            self.enable_verify_apps_over_usb()

    def enable_verify_apps_over_usb(self):
        dev_option = self.d(text='Developer options')
        dev_option.click()
        '''
        self.d(packageName='com.android.settings',
               resourceId='com.android.settings:id/list') \
            .child_by_text("Verify apps over USB", allow_scroll_search=True,
                           resourceId='android:id/title').exists
        '''
        self.d(scrollable=True).scroll.to(text="Verify apps over USB")
        chk = self.d(textContains="Verify apps over USB").right(className="android.widget.Switch")
        if not chk.checked:
            chk.click()
            print "Verify apps over USB is enabled"
        elif chk.checked:
            print "Verify apps over USB is already enabled"
        else:
            self.assertTrue(
                "Verify apps over USB unable to change the state")

    def test_adb_install_usb_protection_disabled(self):
        goto_settings = "am start com.android.settings"
        AdbUtils.run_adb_cmd(goto_settings)

        self.d(scrollable=True).scroll.toEnd()
        dev_option = self.d(text='Developer options')
        if ( dev_option.exists ):
            self.disable_verify_apps_over_usb()
        else:
            self.d(textContains="About").click()
            self.d(scrollable=True).fling.toEnd()
            if self.d(textContains="Build number"):
                for i in range(8):
                    self.d(text="Build number").click()
                    sleep(1)
            self.d.press.back()
            if not dev_option.exists:
                self.assertFalse("Developer options is not enabled")
            self.disable_verify_apps_over_usb()

    def disable_verify_apps_over_usb(self):
        dev_option = self.d(text='Developer options')
        dev_option.click()
        self.d(scrollable=True).scroll.to(text="Verify apps over USB")
        chk = self.d(textContains="Verify apps over USB").right(className="android.widget.Switch")
        if not chk.checked:
            print "Verify apps over USB is already disabled"
        elif chk.checked:
            chk.click()
            print "Verify apps over USB is disabled"
        else:
            self.assertTrue("Verify apps over USB unable to change the state")

    def install_apk(self):
        print "Going to Install an APK"
        apk_file = 'angrybirds4p2p1.apk'
        nav_dir = os.path.join(os.path.expanduser("~/"), ".acs/Artifacts/BENCHMARKS/ANGRYBIRDS/")
        cur_dir = os.getcwd()
        s_num = AdbUtils.run_adb_cmd('getprop ro.serialno')
        # changing directory to apk file location
        os.chdir(nav_dir)
        if os.path.exists(apk_file):
            print "APK file found"
            output = Popen(['adb', '-s',str(s_num),'install', apk_file], stdin=PIPE,stdout=PIPE, stderr=PIPE)
            out, err = output.communicate()
            # changing back the directory to script location
        os.chdir(cur_dir)

    def remove_apk(self):
        print "Going to Uninstall an APK"
        cmd = "pm uninstall com.rovio.angrybirds"
        t = AdbUtils.run_adb_cmd(cmd)

    def test_adb_multiple_boards_connected(self):
        self.assertTrue(len(dut_manager.devices_detected) > 1, "There must be at least 2 DUTS connected for this test")

        goto_settings = "am start com.android.settings"
        AdbUtils.run_adb_cmd(goto_settings)

        self.d(scrollable=True).swipe.up()
        dev_option = self.d(text='Developer options')
        if (dev_option.exists):
            self.enable_usb_debugging()
        else:
            self.d(textContains="About").click()
            sleep(1)
            if (self.d(text='Build number').exists) == False:
                 self.d(scrollable=True).fling.toEnd()
            if (self.d(textContains="Build number").exists):
                for i in range(8):
                    self.d(text="Build number").click()
                    sleep(1)
            self.d.press.back()
            sleep(1)
            if not dev_option.exists:
                self.assertFalse("Developer options is not enabled")
            self.enable_usb_debugging()

        initial_detected_devices = dut_manager.devices_detected
        print "Multiple Devices: " + str(initial_detected_devices)

        close_pkg = "pm clear com.android.settings"
        AdbUtils.run_adb_cmd(close_pkg)

    def enable_usb_debugging(self):
        dev_option = self.d(text='Developer options')
        dev_option.click()
        sleep(2)
        self.d(scrollable=True).scroll()
        self.d(scrollable=True).scroll.to(text="USB debugging")
        chk = self.d(textContains="USB debugging").right(
            className="android.widget.Switch")
        if not chk.checked:
            chk.click()
            if self.d(text='OK').exists:
                self.d(text='OK').click()
            print "USB debugging is enabled"
        elif chk.checked:
            print "USB debugging is already enabled"
        else:
            self.assertTrue(
                "USB debugging unable to change the state")