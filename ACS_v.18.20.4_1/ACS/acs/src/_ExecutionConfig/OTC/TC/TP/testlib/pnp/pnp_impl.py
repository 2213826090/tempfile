"""
pnp test lib
"""
import os
import time
import commands
from testlib.common.common import g_common_obj
from testlib.util.log import Logger
from testlib.util.config import TestConfig


BASE_PATH = os.path.dirname(__file__)
class PnpImpl(object):
    """
    pnp functions.
    """
    def __init__ (self):
        self.d = g_common_obj.get_device()
        cfg_file = os.path.join(os.environ.get('TEST_REPO_ROOT', ''), 'tests.tablet.pnp.conf')
        self.cfg = TestConfig().read(cfg_file, 'pnpsetting')
        self.logger = Logger.getlogger(__name__)

    def save_log(self, log_dir, test_list, log_dic, result_dic):
        """
        save antutu log
        """
        self.logger.info('save log to %s' % log_dir)
        for key in test_list:
            case_log_dir = os.path.join(log_dir, log_dic[key])
            log_file = "%s.log" % os.path.join(case_log_dir, log_dic[key])
            if not os.path.isdir(case_log_dir):
                os.system('mkdir -p %s' % case_log_dir)
            file_handle = open(log_file, 'w')
            file_handle.write("%s\n" % log_dic[key])
            file_handle.write('value=%s\n' % result_dic[key])
            file_handle.close()

    def check_device(self):
        """
        check device status
        """
        self.logger.info("check device statu")
        #check whether adb could use on device
        _, output = commands.getstatusoutput("timeout 3s adb shell ps | grep com.android.systemui")
        if output.find("com.android.systemui") >= 0:
            self.logger.info("device has connected by 'adb'")
            return True
        g_common_obj.root_on_device()
        g_common_obj.remount_device()
        self.unlock_screen()
        self.check_crash()
        _, output = commands.getstatusoutput("timeout 3s adb shell ps | grep com.android.systemui")
        if output.find("com.android.systemui") >= 0:
            self.logger.info("device has connected by 'adb'")
            return True
        self.logger.error("hardware reboot failed")
        return False

    def unlock_screen(self):
        """
        unlock screen
        """
        self.d.screen.on()
        time.sleep(2)
        os.system("adb shell input keyevent 82")
        time.sleep(5)

    def reboot_device_to_home(self):
        """
        reboot device
        """
        self.logger.info("reboot device and it will take about 35 seconds")
        g_common_obj.reboot_device()
        self.check_device()
        g_common_obj.root_on_device()
        g_common_obj.remount_device()
        self.unlock_screen()
        self.check_crash()
        g_common_obj.back_home()

    def clear_cache(self, activity, packetName):
        """
        clear app cache
        """
        self.logger.info("clear cache")
        self.check_crash()
        os.system("adb shell am start " + activity)
        time.sleep(5)

        g_common_obj.back_home()
        launchapps = "-a android.intent.action.MAIN -n com.android.settings/.applications.ManageApplications"
        os.system("adb shell am start " + launchapps)
        time.sleep(5)
        if self.d(text = packetName).exists:
            self.d(text = packetName).click()
            self._cleardata_forceStop()
        elif self.d(text = "Running").exists:
            self.d(text = "Running").click()
            time.sleep(3)
            self.d(text = "All").click()
            time.sleep(3)
            self.d(packageName = "com.android.settings").child_by_text(packetName , allow_scroll_search=True)
            self.d(text = packetName).click()
            time.sleep(3)
            self._cleardata_forceStop()

    def rotate_device_n(self):
        """
        rotate device to n
        """
        self.logger.info("device rotate n")
        self.d.orientation = "n"
        time.sleep(3)

    def rotate_device_l(self):
        """
        rotate device to l
        """
        self.logger.info("device rotate l")
        self.d.orientation = "n"
        time.sleep(3)

    def _cleardata_forceStop(self):
        """
        clear data and force stop app
        """
        time.sleep(5)
        #clear data
        self.logger.info("Clear data")
        self.d(className="android.widget.Button", resourceId="com.android.settings:id/right_button", text="Clear data").click()
        time.sleep(5)
        self.d(className="android.widget.Button", resourceId="android:id/button1", text="OK").click()
        time.sleep(5)

        #force stop
        self.logger.info("Force stop")
        self.d(className="android.widget.Button", resourceId="com.android.settings:id/left_button", text="Force stop").click()
        time.sleep(3)
        self.d(className="android.widget.Button", resourceId="android:id/button1", text="OK").click()
        time.sleep(3)
        os.system("adb shell am force-stop com.android.settings")

    def check_crash(self):
        """
        check whether crash info appear
        """
        self.logger.info("check java crash")
        time.sleep(3)
        if self.d(text = "OK").exists:
            a = self.d(resourceId="android:id/message").text.encode('ascii')
            if not a.find("Unfortunately"):
                self.d(text = "OK").click()

    def freeze_rotation(self):
        """
        freeze rotation
        """
        self.d.freeze_rotation(False)
