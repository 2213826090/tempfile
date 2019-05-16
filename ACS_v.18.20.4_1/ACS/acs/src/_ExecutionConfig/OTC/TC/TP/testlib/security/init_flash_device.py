# -*- coding: utf-8 -*-
from testlib.util.common import g_common_obj
from testlib.util.config import TestConfig
from testlib.util.log import Logger
from testlib.dut_init.dut_init_impl import Function
import time
import os
import re

d = g_common_obj.get_device()
dsn = d.server.adb.device_serial()

class InitFlashDevices(object):

    def __init__(self):
        formatter = "[%(asctime)s - %(levelname)s] %(message)s"
        self.logger = Logger("Security", formatter)
        self.relay = None
        self.func = Function()
        self.cfg_file = "tests.tablet.security.conf"
        self.conf_path = os.path.join(os.environ.get('TEST_DATA_ROOT', ''), self.cfg_file)
        self.x = d.info["displayWidth"]
        self.y = d.info["displayHeight"]
        self.retry_num = 3

    def init_main(self):
        print "[RunTest]: init DUT setup"
        # after flash eng-build, start init setup
        self.init_AdbRoot()
        self.init_StartRPCServer()
        self.init_ConfigFirstBootWizard()
        self.init_wakeUpUnlock()
        self.init_EnableDeveloperOption()
        self.init_KeepAwake()
        self.init_DisableVerifyApp()
        #self.init_set_date_time_language_US()

    def check_product(self):
        cmd = "getprop ro.product.name"
        product = g_common_obj.adb_cmd_capture_msg(cmd)
        return product

    def check_version(self):
        cmd = "getprop ro.build.version.incremental"
        version = g_common_obj.adb_cmd_capture_msg(cmd)
        return version

    def check_build_release(self):
        cmd = "getprop ro.build.version.release"
        release = g_common_obj.adb_cmd_capture_msg(cmd)
        return release

    def unlock_screen(self):
        if d(description="Unlock").exists:
            d.press("menu")

    def single_swipe_to_app_list(self):
        info = d.info
        x = info["displayWidth"] / 2
        y = info["displayHeight"]
        d.swipe(x, y, x, 0)
        time.sleep(2)

    def check_home_ui_user_safely_O(self):
        if d(text="Drive safely").exists:
            if d(text="Owner").exists:
                d(text="Owner").click.wait()
            if d(text="Owner").exists:
                d(text="Owner").click.wait()
        else:
            if d(text="Owner").exists:
                d(text="Owner").click.wait()
            g_common_obj.adb_cmd_capture_msg("input tap 310 1007")

    def check_home_ui_skip_got_it(self):
        if d(textContains="Welcome").exists:
            if d(textContains="GOT IT").exists:
                d(textContains="GOT IT").click.wait()
        if d(text="OK").exists:
            d(text="OK").click.wait()

    def adb_root(self):
        g_common_obj.root_on_device()

    def init_AdbRoot(self):
        """
        Switch to adb root mode
        """
        print "[RunTest]: init dut root"
        succeed = False
        for i in range(5):
            try:
                time.sleep(3)
                self.func.root_DUT()
                succeed = True
                break
            except Exception as e:
                print e
        #assert succeed

    def init_StartRPCServer(self):
        """
        Start RPC server
        """
        print "[RunTest]: init dut start RPC server"
        try:
            self.func.push_uiautomator_jar()
            d.orientation = 'natural'
            d.freeze_rotation()
        except Exception as e:
            print e

    def init_wakeUpUnlock(self):
        print "start to wakeup and unlock screen"
        d.wakeup()
        if d(resourceId="com.android.keyguard:id/keyguard_selector_view_frame").exists:
            d(resourceId="com.android.keyguard:id/keyguard_selector_view_frame").swipe.down()
        if d(description="Unlock").exists:
            d(description="Unlock").drag.to(resourceId="com.android.systemui:id/clock_view")
        product_name = self.check_product()
        print "[INFO]product_name: %s" % product_name
        if "gordon_peak" in product_name:
            self.check_home_ui_user_safely_O()
        else:
            self.unlock_screen()
            self.check_home_ui_skip_got_it()

    def init_ConfigFirstBootWizard(self):
        """
        Config first boot wizard
        """
        print "[RunTest]: init_ConfigFirstBootWizard"
        d.wakeup()
        d.press("home")
        time.sleep(2)
        if d(textContains="Drive safely").exists:
            d(text="Owner").click.wait()
            return
        if d(textContains="START DRIVING").exists:
            d(textContains="START DRIVING").click.wait()
            return
        release = g_common_obj.adb_cmd_capture_msg("getprop |grep 'version.release'")
        if ('[8.0.0]' in release) or ('[8.1.0]' in release):
            return
        if d(textContains="Chrome").exists or d(textContains="Play Store").exists:
            return
        d.wakeup()
        if d(resourceId="com.android.systemui:id/lock_icon"):
            self.func.close_lock_screen()
            return
        print "[Info]------Wizard is exists, need to click four point..."
        succeed = False
        for i in range(self.retry_num):
            try:
                if d(text="OK").exists:
                    d(text="OK").click.wait()
                if d(resourceId="com.google.android.setupwizard:id/language_picker").exists or \
                        d(resourceId="com.google.android.setupwizard:id/welcome_title").exists or \
                        d(resourceId="com.google.android.setupwizard:id/start").exists:
                    for _ in range(3):
                        d.click(100, 100)
                        d.click(self.x - 100, 100)
                        d.click(self.x - 100, self.y - 100)
                        d.click(100, self.y - 100)
                        time.sleep(2)
                        if not d(resourceId="com.google.android.setupwizard:id/language_picker").exists:
                            break
                    if d(text="OK").exists:
                        d(text="OK").click.wait()
                        d(text="OK").wait.gone(timeout=3000)
                    self.check_home_ui_skip_got_it()
                    self.func.close_lock_screen()
                    succeed = True
                    return
                if d(text="Welcome") or \
                        d(resourceId="com.google.android.setupwizard:id/welcome_title") or \
                        d(resourceId="com.google.android.setupwizard:id/start"):
                    self.func.setup_guideline()
                    if d(text="OK").exists:
                        d(text="OK").click.wait()
                    succeed = True
                self.check_home_ui_skip_got_it()
                break
            except Exception as e:
                print e
        assert succeed

    def init_EnableDeveloperOption(self):
        """
        Enable developer option
        """
        print "[RunTest]: init_EnableDeveloperOption"
        succeed = False
        for i in range(self.retry_num):
            try:
                os.system("adb shell input keyevent 82;adb shell input keyevent 3")
                g_common_obj.launch_app_am("com.android.settings", ".Settings")
                time.sleep(2)
                ui = d.dump()
                if 'scrollable="true"' in ui:
                    d(scrollable=True).scroll.toEnd()
                    if d(textContains="Developer options").exists:
                        return
                    time.sleep(2)
                    ui = d.dump()
                if 'System' in ui:
                    system_or_about = 'System'
                elif 'About ' in ui:
                    system_or_about = 'About'
                print 'click ' + system_or_about + ' ...'
                d(textContains=system_or_about).click.wait()
                if d(textContains="Developer options").exists:
                    return
                if d(textContains="About ").exists:
                    d(textContains="About ").click()
                # Fix ui not detected on bxt-p
                # Change structure to try/except
                try:
                    d(scrollable=True).scroll.vert.to(textContains="Build number")
                    time.sleep(1)
                except:
                    print "No need scroll."
                for _ in range(8):
                    d(textContains="Build number").click()
                    time.sleep(.5)
                succeed = True
                break
            except Exception as e:
                print e
        assert succeed, "Enable Developer Option fail"

    def init_KeepAwake(self):
        """
        Keep system stay awake
        """
        print "[RunTest]: init_KeepAwake"
        succeed = False
        for i in range(self.retry_num):
            try:
                os.system("adb shell am force-stop com.android.settings")
                os.system("adb shell am start -a android.settings.APPLICATION_DEVELOPMENT_SETTINGS")
                self.func.settings_ON_OFF("Stay awake", enable=True)
                succeed = True
                break
            except Exception as e:
                print e
        assert succeed

    def init_DisableVerifyApp(self):
        """
        Disable verify apps
        """
        print "[RunTest]: init_DisableVerifyApp"
        succeed = False
        for i in range(self.retry_num):
            try:
                os.system("adb shell input keyevent 82;adb shell input keyevent 3")
                os.system("adb shell am force-stop com.android.settings")
                os.system("adb shell am start -a android.settings.APPLICATION_DEVELOPMENT_SETTINGS")
                d(scrollable=True).scroll.vert.to(textContains="Verify apps over USB")
                self.func.settings_ON_OFF("Verify apps over USB", enable=False)
                succeed = True
                break
            except Exception as e:
                print e
        assert succeed

    def init_set_date_time_language_US(self):
        print "[RunTest]: init_set_date_time_language_US"
        succeed = False
        for i in range(self.retry_num):
            try:
                self.func.set_language()
                self.func.set_timedate()
                succeed = True
                break
            except Exception as e:
                print e
        assert succeed
